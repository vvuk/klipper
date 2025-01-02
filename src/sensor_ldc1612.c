// Support for eddy current sensor data from ldc1612 chip
//
// Copyright (C) 2023 Alan.Ma <tech@biqu3d.com>
// Copyright (C) 2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "i2ccmds.h" // i2cdev_oid_lookup
#include "sched.h" // DECL_TASK
#include "sensor_bulk.h" // sensor_bulk_report
#include "trsync.h" // trsync_do_trigger
#include "printf.h"

void dprint(const char *fmt, ...);

enum {
    LDC_PENDING = 1<<0, LDC_HAVE_INTB = 1<<1,
    LH_AWAIT_HOMING = 1<<1, LH_AWAIT_TAP = 1<<2,
    LH_CAN_TRIGGER = 1<<3, LH_WANT_TAP = 1<<4,
    LH_V2 = 1<<5,
};

#define REASON_ERROR_SENSOR 0
#define REASON_ERROR_PROBE_TOO_LOW 1
#define REASON_TOUCH 5

#define HCOUNT 64

#define SAMPLE_ERROR(data) ((data) >> 28)

// Configuration
#define WINDOW_SIZE 16 // Moving average window size (can be changed)
#define DERIVATIVE_WINDOW 4 // Number of samples to average the derivative
#define TRIGGER_THRESHOLD -10 // Threshold for the derivative average to trigger

// Static variables for historical data
static uint32_t frequency_buffer[WINDOW_SIZE] = {0}; // Circular buffer for WMA
static int32_t derivative_buffer[DERIVATIVE_WINDOW] = {0}; // Circular buffer for derivative average
static uint32_t wma_sum = 0; // Sum of the last WINDOW_SIZE frequencies
static int current_index = 0; // Index for circular buffer
static int derivative_index = 0; // Index for derivative circular buffer
static uint32_t last_wma = 0; // Last computed WMA


// match data_rate in ldc1612.py
// at 250, this is 4ms per sample: at 5mm/sec probe speed, probe could move 0.02mm in between samples
// at 1000, (1ms/sample) this is 0.0025mm movement per sample; much better. 
//#define SAMPLES_PER_SEC 1000

// how many values to use for a simple average. Must be less than 16, because that's how many bits we
// have left to avoid 64-bit math
#define SIMPLE_AVG_VALUE_COUNT 16

struct ldc1612_v2 {
    // Used from parent:
    // homing_flags
    // ts

    uint32_t freq_buffer[FREQ_WINDOW_SIZE];
    int32_t deriv_buffer[DERIV_WINDOW_SIZE];

    uint32_t wma_sum; // sum of the freq_buffer
    uint32_t wma; // last computed weighted moving average

    // frequency we must pass through to have a valid home/tap
    uint32_t safe_start_freq;
    // and it must happen after this time
    uint32_t safe_start_time;

    // the frequency to trigger on for homing, or
    // the second threshold before we start looking for a tap
    uint32_t homing_trigger_freq;

    uint32_t tap_threshold;

    // current index in freq/deriv buffers
    uint8_t freq_i;
    uint8_t deriv_i;

    // trigger reasons
    uint8_t success_reason;
    uint8_t other_reason_base;
};

static void check_home2(struct ldc1612* ld, uint32_t data);

struct ldc1612 {
    struct timer timer;
    uint32_t rest_ticks;
    struct i2cdev_s *i2c;
    uint8_t flags;
    struct sensor_bulk sb;
    struct gpio_in intb_pin;
    uint16_t prev_status;

    uint32_t last_read_value;

    // homing
    struct trsync *ts;
    uint8_t homing_flags;
    uint8_t trigger_reason, error_reason, pretap_reason;
    uint8_t ema_factor;
    uint32_t sensor_average, sensor_last;
    int32_t change_average;
    int32_t change_average_last;

    uint32_t trigger_threshold;
    uint32_t homing_clock, tap_clock;
    int32_t tap_threshold;
    uint32_t tap_adjust_factor;

    struct ldc1612_v2 v2;
};

static struct task_wake ldc1612_wake;

#define STATUS_ERR_AHE (1<<10)
#define STATUS_ERR_ALE (1<<9)

// Check if the intb line is "asserted"
static int
check_intb_asserted(struct ldc1612 *ld)
{
    return !gpio_in_read(ld->intb_pin);
}

// Query ldc1612 data
static uint_fast8_t
ldc1612_event(struct timer *timer)
{
    struct ldc1612 *ld = container_of(timer, struct ldc1612, timer);
    if (ld->flags & LDC_PENDING)
        ld->sb.possible_overflows++;
    if (!(ld->flags & LDC_HAVE_INTB) || check_intb_asserted(ld)) {
        ld->flags |= LDC_PENDING;
        sched_wake_task(&ldc1612_wake);
    }
    ld->timer.waketime += ld->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_ldc1612(uint32_t *args)
{
    struct ldc1612 *ld = oid_alloc(args[0], command_config_ldc1612
                                   , sizeof(*ld));

    ld->timer.func = ldc1612_event;
    ld->i2c = i2cdev_oid_lookup(args[1]);
    ld->ema_factor = 0;
}
DECL_COMMAND(command_config_ldc1612, "config_ldc1612 oid=%c i2c_oid=%c");

void
command_config_ldc1612_with_intb(uint32_t *args)
{
    command_config_ldc1612(args);
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);
    ld->intb_pin = gpio_in_setup(args[2], 1);
    ld->flags = LDC_HAVE_INTB;
}
DECL_COMMAND(command_config_ldc1612_with_intb,
             "config_ldc1612_with_intb oid=%c i2c_oid=%c intb_pin=%c");

void
command_ldc1612_setup_home2(uint32_t *args)
{
    struct ldc1612 *ld1 = oid_lookup(args[0], command_config_ldc1612);
    struct ldc1612_v2 *ld = &ld1->v2;

    uint32_t trsync_oid = args[1];
    uint8_t trigger_reason = args[2];
    uint8_t other_reason_base = args[3];
    uint32_t trigger_freq = args[4];
    uint32_t start_freq = args[5];
    uint32_t start_time = args[6];
    int32_t tap_threshold = args[7];

    if (trigger_freq == 0 || trsync_oid == 0) {
        dprint("ZZZ resetting homing/tapping");
        ld1->ts = NULL;
        ld1->homing_flags = 0;
        return;
    }

    ld->success_reason = trigger_reason;
    ld->other_reason_base = other_reason_base;

    ld->safe_start_freq = start_freq;
    ld->safe_start_time = start_time;
    ld->homing_trigger_freq = trigger_freq;
    ld->tap_threshold = tap_threshold;

    memset(ld->freq_buffer, 0, sizeof(ld->freq_buffer));
    memset(ld->deriv_buffer, 0, sizeof(ld->deriv_buffer));
    ld->wma_sum = 0;
    ld->wma = 0;
    ld->freq_i = 0;
    ld->deriv_i = 0;

    ld1->ts = trsync_oid_lookup(trsync_oid);
    ld1->homing_flags = LH_V2 | LH_AWAIT_HOMING | LH_CAN_TRIGGER;

    if (tap_threshold) {
        ld1->homing_flags |= LH_WANT_TAP;
    }

    dprint("ZZZ home2 sf=%u tf=%u tap=%d", start_freq, trigger_freq, tap_threshold);
}
DECL_COMMAND(command_ldc1612_setup_home2,
             "ldc1612_setup_home2 oid=%c"
             " trsync_oid=%c trigger_reason=%c other_reason_base=%c"
             " trigger_freq=%u start_freq=%u start_time=%u"
             " tap_threshold=%d");


void
command_ldc1612_setup_home(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);

    ld->trigger_threshold = args[2];
    if (!ld->trigger_threshold) {
        dprint("ZZZ clearing homing!");
        ld->ts = NULL;
        ld->homing_flags = 0;
        return;
    }

    // contains both the time that homing shuold start (ignore samples before this),
    // as well as the trigger time when homing triggers
    ld->homing_clock = args[1];
    ld->ts = trsync_oid_lookup(args[3]);
    ld->trigger_reason = args[4];
    ld->error_reason = args[5];
    ld->tap_threshold = args[6];
    ld->tap_adjust_factor = args[7];
    ld->tap_clock = args[8];
    ld->pretap_reason = args[9];
    ld->sensor_last = 0;
    ld->change_average_last = 0;
    if (ld->tap_threshold)
        // Homing until a nozzle/bed contact is detected
        ld->homing_flags = (LH_AWAIT_HOMING | LH_CAN_TRIGGER
                            | LH_AWAIT_TAP | LH_WANT_TAP);
    else
        // Homing until threshold met
        ld->homing_flags = LH_AWAIT_HOMING | LH_CAN_TRIGGER;

    uint32_t time = timer_read_time();
    dprint("ZZZ home fut=%u", ld->homing_clock-time);
}
DECL_COMMAND(command_ldc1612_setup_home,
             "ldc1612_setup_home oid=%c clock=%u threshold=%u"
             " trsync_oid=%c trigger_reason=%c error_reason=%c"
             " tap_threshold=%i tap_adjust_factor=%u tap_clock=%u"
             " pretap_reason=%c");

// Exponential moving average base factor
#define EMA_BASE 16

void
command_ldc1612_setup_averaging(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);
    ld->ema_factor = args[1];
    if (args[1] >= EMA_BASE)
        shutdown("Invalid ldc1612 ema factor");
}
DECL_COMMAND(command_ldc1612_setup_averaging,
             "ldc1612_setup_averaging oid=%c factor=%c");

void
command_query_ldc1612_home_state(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);
    sendf("ldc1612_home_state oid=%c homing=%c trigger_clock=%u"
          , args[0], !!(ld->homing_flags & LH_CAN_TRIGGER), ld->homing_clock);
}
DECL_COMMAND(command_query_ldc1612_home_state,
             "query_ldc1612_home_state oid=%c");

void
command_query_ldc1612_latched_status(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);
    sendf("ldc1612_latched_status oid=%c status=%u lastval=%u"
          , args[0], ld->prev_status, ld->last_read_value);
}
DECL_COMMAND(command_query_ldc1612_latched_status,
             "query_ldc1612_latched_status oid=%c");

// Notify trsync of event
static void
notify_trigger(struct ldc1612 *ld, uint32_t time, uint8_t reason)
{
    ld->homing_flags = 0;
    ld->homing_clock = time;
    trsync_do_trigger(ld->ts, reason);
    dprint("ZZZ notify_trigger: %u", reason);
}

static void
update_sensor_average(struct ldc1612 *ld, uint32_t data)
{
    // Update simple average
    uint32_t *last_values = ld->last_values;
    last_values[ld->last_values_next_i++] = data;
    if (ld->last_values_next_i == SIMPLE_AVG_VALUE_COUNT)
        ld->last_values_next_i = 0;

    uint32_t simple_avg = 0;
    for (int i = 0; i < SIMPLE_AVG_VALUE_COUNT; i++)
        simple_avg += last_values[i];

    ld->simple_average = DIV_ROUND_CLOSEST(simple_avg, SIMPLE_AVG_VALUE_COUNT);

    // update EMA
    uint32_t ema_factor = ld->ema_factor;

    if (ema_factor == 0) {
        ld->sensor_average = data;
        return;
    }

    uint32_t scaled_prev = ld->sensor_average * ema_factor;
    uint32_t scaled_data = data * (EMA_BASE - ema_factor);
    uint32_t new_avg = DIV_ROUND_CLOSEST(scaled_data + scaled_prev, EMA_BASE);

    ld->sensor_average = new_avg;
}

void
check_home2(struct ldc1612* ld, uint32_t data)
{
    uint32_t time = timer_read_time();

    if (SAMPLE_ERROR(data)) {
        dprint("ZZZ home2 err=%u", data);
        // ignore over-amplitude errors (probe too far)
        if ((ld->prev_status & STATUS_ERR_AHE) != 0)
            return;

        // Sensor reports an issue - cancel homing
        notify_trigger(ld, 0, ld->other_reason_base + REASON_ERROR_SENSOR);
        return;
    }

    uint8_t homing_flags = ld->homing_flags;

    if (homing_flags & LH_AWAIT_HOMING) {
        if (data < ld->homing_start_freq) {
            //dprint("data=%u < %u", data, ld->homing_start_freq);
            return;
        }

        if (timer_is_before(time, ld->homing_start_time)) {
            dprint("ZZZ EARLY! time=%u < %u", time, ld->homing_start_time);
            notify_trigger(ld, 0, ld->other_reason_base + REASON_ERROR_PROBE_TOO_LOW);
            return;
        }

        dprint("ZZZ -AWAIT_HOMING");
        ld->homing_flags = homing_flags = homing_flags & ~LH_AWAIT_HOMING;
    }

    update_sensor_average(ld, data);
    uint32_t avg = ld->sensor_average;

    //dprint("data=%u avg %u", data, avg);
    if (avg > ld->homing_trigger_freq) {
        notify_trigger(ld, time, ld->trigger_reason);
        dprint("ZZZ trig t=%u f=%u d=%u q=%u", time, avg, data, ld->simple_average);
        uint32_t* lv = ld->last_values;
        uint32_t li = ld->last_values_next_i;
        //for (int q = 0; q < 4; q++) {
            int off = 12; //q*4;
            dprint("ZZZ %u %u %u %u", lv[(li+off)&0xf], lv[(li+off+1)&0xf], lv[(li+off+2)&0xf], lv[(li+off+3)&0xf]);
        //}
    }
}

// Check if a sample should trigger a homing event
static void
check_home(struct ldc1612 *ld, uint32_t data)
{
    uint8_t homing_flags = ld->homing_flags;
    if (!(homing_flags & LH_CAN_TRIGGER))
        return;
    if (SAMPLE_ERROR(data)) {
        // ignore over-amplitude errors (probe too far)
        if ((ld->prev_status & STATUS_ERR_AHE) != 0)
            return;

        // Sensor reports an issue - cancel homing
        notify_trigger(ld, 0, ld->error_reason);
        return;
    }
    // Perform sensor averaging
    uint32_t ema_factor = ld->ema_factor;

    update_sensor_average(ld, data);
    uint32_t new_avg = ld->sensor_average;

    // Track rate of change between sensor samples
    int32_t change = data - ld->sensor_last;
    ld->sensor_last = data;

    // EMA
    int32_t scaled_cprev = ld->change_average * ema_factor;
    int32_t scaled_chg = change * (EMA_BASE - ema_factor);
    int32_t new_cavg = DIV_ROUND_CLOSEST(scaled_chg + scaled_cprev, EMA_BASE);

    ld->change_average_last = ld->change_average;
    ld->change_average = new_cavg;

    // Check if should signal a trigger event
    uint32_t time = timer_read_time();

    if (homing_flags & LH_AWAIT_HOMING) {
        if (timer_is_before(time, ld->homing_clock))
            return;
        dprint("ZZZ -AWAIT_HOMING");
        ld->homing_flags = homing_flags = homing_flags & ~LH_AWAIT_HOMING;
    }

    if (!(homing_flags & LH_WANT_TAP)) {
        // Trigger on simple threshold check
        if (new_avg > ld->trigger_threshold)
            notify_trigger(ld, time, ld->trigger_reason);
        return;
    }

    if (homing_flags & LH_AWAIT_TAP) {
        // Check if can start tap detection
        if (timer_is_before(time, ld->tap_clock)) {
            if (new_avg > ld->trigger_threshold) {
                // Sensor too close to bed prior to start of tap detection
                notify_trigger(ld, time, ld->pretap_reason);
            }
            return;
        }
        ld->homing_flags = homing_flags = homing_flags & ~LH_AWAIT_TAP;
    }

    // tap_adjust_factor is (tap_factor / data_rate) broadcast to
    // the 0..2^32-1 range. This is equivalent to doing
    // new_avg * (tap_factor / data_rate). The () value is going to be
    // something like 0.0048. So we're taking how much of the average
    // we want to consider
    int32_t adjust = (int32_t)(((uint64_t)new_avg * (uint64_t)(ld->tap_adjust_factor)) >> 32);

    if (new_cavg < ld->tap_threshold + adjust) {
        // Tap detected (sensor no longer moving closer to bed)
        notify_trigger(ld, time, ld->trigger_reason);
    }
}

// Chip registers
#define REG_DATA0_MSB 0x00
#define REG_DATA0_LSB 0x01
#define REG_STATUS    0x18

// Read a register on the ldc1612
static void
read_reg(struct ldc1612 *ld, uint8_t reg, uint8_t *res)
{
    int ret = i2c_dev_read(ld->i2c, sizeof(reg), &reg, 2, res);
    i2c_shutdown_on_err(ret);
}

// Read the status register on the ldc1612
static uint16_t
read_reg_status(struct ldc1612 *ld)
{
    uint8_t data_status[2];
    read_reg(ld, REG_STATUS, data_status);
    ld->prev_status = (data_status[0] << 8) | data_status[1];
    return ld->prev_status;
}

#define BYTES_PER_SAMPLE 4

// Query ldc1612 data
static void
ldc1612_query(struct ldc1612 *ld, uint8_t oid)
{
    uint16_t status = read_reg_status(ld);
    irq_disable();
    ld->flags &= ~LDC_PENDING;
    irq_enable();
    if (!(status & 0x08)) // UNREADCONV1
        return;

    // Read coil0 frequency
    uint8_t *d = &ld->sb.data[ld->sb.data_count];
    read_reg(ld, REG_DATA0_MSB, &d[0]);
    read_reg(ld, REG_DATA0_LSB, &d[2]);
    ld->sb.data_count += BYTES_PER_SAMPLE;

    uint32_t data =   ((uint32_t)d[0] << 24)
                    | ((uint32_t)d[1] << 16)
                    | ((uint32_t)d[2] << 8)
                    | ((uint32_t)d[3]);

    ld->last_read_value = data;

    if (ld->homing_flags & LH_CAN_TRIGGER) {
        // Check for endstop trigger
        if (ld->homing_flags & LH_V2)
            check_home2(ld, data);
        else
            check_home(ld, data);
    }

    // Flush local buffer if needed
    if (ld->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(ld->sb.data))
        sensor_bulk_report(&ld->sb, oid);
}

void
command_ldc1612_start_stop(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);

    sched_del_timer(&ld->timer);
    ld->flags &= ~LDC_PENDING;
    if (!args[1]) {
        // End measurements
        dprint("ZZZ stop");
        return;
    }

    dprint("ZZZ start");

    // Start new measurements query
    ld->rest_ticks = args[1];
    sensor_bulk_reset(&ld->sb);
    irq_disable();
    ld->timer.waketime = timer_read_time() + ld->rest_ticks;
    sched_add_timer(&ld->timer);
    irq_enable();
}
DECL_COMMAND(command_ldc1612_start_stop, "ldc1612_start_stop oid=%c rest_ticks=%u");

void
command_ldc1612_query_bulk_status(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);

    if (ld->flags & LDC_HAVE_INTB) {
        // Check if a sample is pending in the chip via the intb line
        irq_disable();
        uint32_t time = timer_read_time();
        int p = check_intb_asserted(ld);
        irq_enable();
        sensor_bulk_status(&ld->sb, args[0], time, 0, p ? BYTES_PER_SAMPLE : 0);
    } else {
        // Query sensor to see if a sample is pending
        uint32_t time1 = timer_read_time();
        uint16_t status = read_reg_status(ld);
        uint32_t time2 = timer_read_time();

        uint32_t fifo = status & 0x08 ? BYTES_PER_SAMPLE : 0;
        sensor_bulk_status(&ld->sb, args[0], time1, time2-time1, fifo);
    }
}
DECL_COMMAND(command_ldc1612_query_bulk_status, "ldc1612_query_bulk_status oid=%c");

void
ldc1612_task(void)
{
    if (!sched_check_wake(&ldc1612_wake))
        return;
    uint8_t oid;
    struct ldc1612 *ld;
    foreach_oid(oid, ld, command_config_ldc1612) {
        uint_fast8_t flags = ld->flags;
        if (!(flags & LDC_PENDING))
            continue;
        ldc1612_query(ld, oid);
    }
}
DECL_TASK(ldc1612_task);

void dprint(const char *fmt, ...)
{
    char buf[60];

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf)-1, fmt, args);
    va_end(args);

    sendf("debug_print m=%*s", len, buf);
}

// Configuration
#define WINDOW_SIZE 16 // Moving average window size (can be changed)
#define DERIVATIVE_WINDOW 4 // Number of samples to average the derivative
#define TRIGGER_THRESHOLD -10 // Threshold for the derivative average to trigger

// Static variables for historical data
static uint32_t frequency_buffer[WINDOW_SIZE] = {0}; // Circular buffer for WMA
static int32_t derivative_buffer[DERIVATIVE_WINDOW] = {0}; // Circular buffer for derivative average
static uint32_t wma_sum = 0; // Sum of the last WINDOW_SIZE frequencies
static int current_index = 0; // Index for circular buffer
static int derivative_index = 0; // Index for derivative circular buffer
static uint32_t last_wma = 0; // Last computed WMA

// Trigger function
bool check_trigger(uint32_t freq) {
    // Update the WMA buffer
    wma_sum -= frequency_buffer[current_index]; // Subtract the oldest value
    wma_sum += freq;                            // Add the new frequency
    frequency_buffer[current_index] = freq;     // Update the buffer
    current_index = (current_index + 1) % WINDOW_SIZE; // Move the index in circular manner

    // Compute the Weighted Moving Average
    uint32_t wma = wma_sum / WINDOW_SIZE;

    // Compute the derivative of the WMA
    int32_t derivative = (int32_t)wma - (int32_t)last_wma;
    last_wma = wma;

    // Update the derivative buffer
    derivative_buffer[derivative_index] = derivative;
    derivative_index = (derivative_index + 1) % DERIVATIVE_WINDOW;

    // Compute the average of the derivative buffer
    int32_t derivative_sum = 0;
    for (int i = 0; i < DERIVATIVE_WINDOW; i++) {
        derivative_sum += derivative_buffer[i];
    }
    int32_t derivative_avg = derivative_sum / DERIVATIVE_WINDOW;

    // Check if the derivative average drops below the threshold
    if (derivative_avg < TRIGGER_THRESHOLD) {
        return true; // Trigger detected
    }

    return false; // No trigger
}

static uint32_t compute_weight_sum(int size) {
    return (size * (size + 1)) / 2;
}

void
check_home2(struct ldc1612* ld1, uint32_t data)
{
    static uint32_t wma_weight_sum = compute_weight_sum(FREQ_WINDOW_SIZE);

    struct ldc1612_v2 *ld = &ld1->v2;
    uint8_t homing_flags = ld1->homing_flags;
    bool is_tap = homing_flags & LH_WANT_TAP;

    uint32_t time = timer_read_time();

    if (SAMPLE_ERROR(data)) {
        dprint("ZZZ home2 err=%u", data);
        // ignore amplitude errors (likely probe too far),
        // unless we're tapping, in which case we should consider
        // all errors for safety -- it means the tap wasn't started
        // at an appropriate distance
        if ((!is_tap && ld1->prev_status & STATUS_ERR_AHE) != 0)
            return;

        // Sensor reports an issue - cancel homing
        notify_trigger(ld1, 0, ld->other_reason_base + REASON_ERROR_SENSOR);
        return;
    }

    //
    // Update the sensor averages and derivatives
    //

    ld->freq_buffer[ld->freq_i] = data;
    ld->freq_i = (ld->freq_i + 1) % FREQ_WINDOW_SIZE;

    // TODO: We can avoid 64-bit integers here by just offseting
    // the numbers -- it should be safe to subtract the safe_start_freq
    // and just deal with offsets above that. But do 64-bit math here
    // for now
    uint64_t wma_numerator = 0;
    for (int i = 0; i < FREQ_WINDOW_SIZE; i++) {
        int weight = i + 1;
        int buffer_index = (current_index + i) % FREQ_WINDOW_SIZE; // Circular buffer index
        wma_numerator += frequency_buffer[buffer_index] * weight;
    }

    // WMA + derivative of the WMA
    uint32_t wma = (uint32_t)(wma_numerator / weight_sum);
    int32_t wma_d = (int32_t)wma - (int32_t)ld->wma;
    ld->wma = wma;

    ld->deriv_buffer[ld->deriv_i] = wma_d;
    ld->deriv_i = (ld->deriv_i + 1) % DERIV_WINDOW_SIZE;

    // Then a simple average of the derivative buffer
    int32_t deriv_sum = 0;  
    for (int i = 0; i < DERIV_WINDOW_SIZE; i++) {
        deriv_sum += ld->deriv_buffer[i];
    }

    int32_t deriv_avg = DIV_ROUND_CLOSEST(deriv_sum, DERIV_WINDOW_SIZE);

    // Safety threshold check
    // We need to pass through this frequency threshold to be a valid dive.
    // We just use the simple data values, not the averages.
    if (homing_flags & LH_AWAIT_HOMING) {
        if (data < ld->safe_start_freq)
            return;

        // And we need to do it _after_ this time, to make sure we didn't
        // start below the thershold
        if (timer_is_before(time, ld->safe_start_time)) {
            dprint("ZZZ EARLY! time=%u < %u", time, ld->homing_start_time);
            notify_trigger(ld, 0, ld->other_reason_base + REASON_ERROR_PROBE_TOO_LOW);
            return;
        }

        dprint("ZZZ safe start");

        if (is_tap && ld->homing_trigger_freq != 0) {
            // If we're tapping, then make the homing trigger freq a second thershold
            ld->safe_start_freq = ld->homing_trigger_freq;
            ld->homing_trigger_freq = 0;
            return;
        }

        // Ok, we've passed all the safety thresholds. Values from this point on
        // will be considered for homing/tapping
        ld1->homing_flags = homing_flags = homing_flags & ~LH_AWAIT_HOMING;
    }


    uint32_t avg = ld->sensor_average;

    //dprint("data=%u avg %u", data, avg);
    if (avg > ld->homing_trigger_freq) {
        notify_trigger(ld, time, ld->trigger_reason);
        dprint("ZZZ trig t=%u f=%u d=%u q=%u", time, avg, data, ld->simple_average);
        uint32_t* lv = ld->last_values;
        uint32_t li = ld->last_values_next_i;
        //for (int q = 0; q < 4; q++) {
            int off = 12; //q*4;
            dprint("ZZZ %u %u %u %u", lv[(li+off)&0xf], lv[(li+off+1)&0xf], lv[(li+off+2)&0xf], lv[(li+off+3)&0xf]);
        //}
    }
}

static void
check_tap2(struct ldc1612* ld, uint32_t data)
{
    uint32_t time = timer_read_time();

    if (SAMPLE_ERROR(data)) {
        // Sensor reports an issue - cancel tapping for any kind of error
        dprint("ZZZ tap2 err=%u", data);
        notify_trigger(ld, 0, ld->other_reason_base + REASON_ERROR_SENSOR);
        return;
    }

    uint8_t homing_flags = ld->homing_flags;

    if (homing_flags & LH_AWAIT_HOMING) {
        if (data < ld->homing_start_freq)
            return;

        // Did we hit this threshold too early? Make sure we actually move through
        // it, and not just start past it.
        if (timer_is_before(time, ld->homing_start_time)) {
            dprint("ZZZ EARLY! time=%u < %u", time, ld->homing_start_time);
            notify_trigger(ld, 0, ld->other_reason_base + REASON_ERROR_PROBE_TOO_LOW);
            return;
        }

        dprint("ZZZ -AWAIT_HOMING");
        ld->homing_flags = homing_flags = homing_flags & ~LH_AWAIT_HOMING;
    }

    update_sensor_average(ld, data);
    uint32_t avg = ld->sensor_average;

    //dprint("data=%u avg %u", data, avg);
    if (avg > ld->homing_trigger_freq) {
        notify_trigger(ld, time, ld->trigger_reason);
        dprint("ZZZ trig t=%u f=%u d=%u q=%u", time, avg, data, ld->simple_average);
        uint32_t* lv = ld->last_values;
        uint32_t li = ld->last_values_next_i;
        //for (int q = 0; q < 4; q++) {
            int off = 12; //q*4;
            dprint("ZZZ %u %u %u %u", lv[(li+off)&0xf], lv[(li+off+1)&0xf], lv[(li+off+2)&0xf], lv[(li+off+3)&0xf]);
        //}
    }
}
