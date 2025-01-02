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

#define BYTES_PER_SAMPLE 4

#define REASON_ERROR_SENSOR 0
#define REASON_ERROR_PROBE_TOO_LOW 1
#define REASON_TOUCH 5

// Chip registers
#define REG_DATA0_MSB 0x00
#define REG_DATA0_LSB 0x01
#define REG_STATUS    0x18

#define SAMPLE_ERR(data) ((data) >> 28)
// Error flags reported in samples: undeer range, over range, watchdog, amplitude 
#define SAMPLE_ERR_UR 0x8
#define SAMPLE_ERR_OR 0x4
#define SAMPLE_ERR_WD 0x2
#define SAMPLE_ERR_AE 0x1

// conversion under range
#define STATUS_ERR_UR 0x2000
// conversion over range
#define STATUS_ERR_OR 0x1000
// watchdog timeout
#define STATUS_ERR_WD 0x0800
// amplitude high error
#define STATUS_ERR_AHE 0x0400
// amplitude low error
#define STATUS_ERR_ALE 0x0200

// match data_rate in ldc1612.py
// at 250, this is 4ms per sample: at 5mm/sec probe speed, probe could move 0.02mm in between samples
// at 1000, (1ms/sample) this is 0.0025mm movement per sample; much better. 
//#define SAMPLES_PER_SEC 1000

// Configuration
#define FREQ_WINDOW_SIZE 16
#define DERIV_WINDOW_SIZE 8

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

struct ldc1612 {
    struct timer timer;
    uint32_t rest_ticks;
    struct i2cdev_s *i2c;
    uint8_t flags;
    struct sensor_bulk sb;
    struct gpio_in intb_pin;
    uint16_t prev_status;

    // Samples per second (configured 
    uint32_t data_rate;
    uint32_t 

    uint32_t last_read_value;

    // homing
    struct trsync *ts;
    uint8_t homing_flags;
    uint8_t trigger_reason, error_reason;

    uint32_t trigger_threshold;
    uint32_t homing_clock;

    struct ldc1612_v2 v2;
};

static void check_home2(struct ldc1612* ld, uint32_t data);
static void read_reg(struct ldc1612 *ld, uint8_t reg, uint8_t *res);
static uint16_t read_reg_status(struct ldc1612 *ld);

static struct task_wake ldc1612_wake;

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
    ld->homing_flags = LH_AWAIT_HOMING | LH_CAN_TRIGGER;
}
DECL_COMMAND(command_ldc1612_setup_home,
             "ldc1612_setup_home oid=%c clock=%u threshold=%u"
             " trsync_oid=%c trigger_reason=%c error_reason=%c");

// Exponential moving average base factor
#define EMA_BASE 16

void
command_ldc1612_setup_averaging(uint32_t *args)
{
    //struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);
    // TODO: reuse
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

    uint32_t status = ld->prev_status;
    uint32_t lastval = ld->last_read_value;

    // If we're not actively running, then read the status and
    // value directly
    if (ld->rest_ticks == 0) {
        irq_disable();
        status = read_reg_status(ld);
        uint8_t d[4];
        read_reg(ld, REG_DATA0_MSB, &d[0]);
        read_reg(ld, REG_DATA0_LSB, &d[2]);
        irq_enable();

        lastval =   ((uint32_t)d[0] << 24)
                  | ((uint32_t)d[1] << 16)
                  | ((uint32_t)d[2] << 8)
                  | ((uint32_t)d[3]);
    }

    sendf("ldc1612_latched_status oid=%c status=%u lastval=%u"
          , args[0], status, lastval);
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
    dprint("ZZZ notify_trigger: %u at %u", reason, time);
}

// Check if a sample should trigger a homing event
static void
check_home(struct ldc1612 *ld, uint32_t data)
{
    uint8_t homing_flags = ld->homing_flags;

    if (SAMPLE_ERR(data)) {
        // Sensor reports an issue - cancel homing
        notify_trigger(ld, 0, ld->error_reason);
        return;
    }

    uint32_t time = timer_read_time();

    // Check if should signal a trigger event
    if (homing_flags & LH_AWAIT_HOMING) {
        if (timer_is_before(time, ld->homing_clock))
            return;
        dprint("ZZZ -AWAIT_HOMING");
        ld->homing_flags = homing_flags = homing_flags & ~LH_AWAIT_HOMING;
    }

    // Trigger on simple threshold check
    if (data > ld->trigger_threshold)
        notify_trigger(ld, time, ld->trigger_reason);
}

// Read a register on the ldc1612
void
read_reg(struct ldc1612 *ld, uint8_t reg, uint8_t *res)
{
    int ret = i2c_dev_read(ld->i2c, sizeof(reg), &reg, 2, res);
    i2c_shutdown_on_err(ret);
}

// Read the status register on the ldc1612
uint16_t
read_reg_status(struct ldc1612 *ld)
{
    uint8_t data_status[2];
    read_reg(ld, REG_STATUS, data_status);
    ld->prev_status = (data_status[0] << 8) | data_status[1];
    return ld->prev_status;
}

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
    ld->rest_ticks = args[1];

    if (ld->rest_ticks == 0) {
        // End measurements
        dprint("ZZZ stop");
        return;
    }

    dprint("ZZZ start");

    // Start new measurements query
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
             " tap_threshold=%i");

#define WEIGHT_SUM(size) ((size * (size + 1)) / 2)

void
check_home2(struct ldc1612* ld1, uint32_t data)
{
    // WTB constexpr
    static uint32_t s_freq_weight_sum = WEIGHT_SUM(FREQ_WINDOW_SIZE);
    static uint32_t s_deriv_weight_sum = WEIGHT_SUM(DERIV_WINDOW_SIZE);

    struct ldc1612_v2 *ld = &ld1->v2;
    uint8_t homing_flags = ld1->homing_flags;
    uint8_t is_tap = !!(homing_flags & LH_WANT_TAP);

    uint32_t time = timer_read_time();

    if (SAMPLE_ERR(data)) {
        // TODO: test homing from very high up
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
    // We use a windowed moving average for both the frequencies
    // and their derivatives. This seemed to give a better signal
    // after staring at a jupyter notebook with plotly plots for
    // too long. Because the values are always increasing as we
    // probe, the WMA undershoots the true value by just a tiny bit,
    // but it does a great job of smoothing out the noise in the sensor.
    //
    // Because the sensor is always going to be used at the same ranges,
    // we can also look at the true values and calculate a small fixed amount
    // to add to the WMA to make it more accurate in the ranges we care about.

    ld->freq_buffer[ld->freq_i] = data;
    ld->freq_i = (ld->freq_i + 1) % FREQ_WINDOW_SIZE;

    // TODO: We can avoid 64-bit integers here by just offseting
    // the numbers -- it should be safe to subtract the safe_start_freq
    // and just deal with offsets above that, because ultimately we
    // only care about the derivative. But do 64-bit math here
    // for now
    uint64_t wma_numerator = 0;
    for (int i = 0; i < FREQ_WINDOW_SIZE; i++) {
        int weight = i + 1;
        int j = (ld->freq_i + i) % FREQ_WINDOW_SIZE;
        wma_numerator += ((uint64_t)ld->freq_buffer[j]) * weight;
    }

    // WMA and derivative of the WMA
    uint32_t wma = (uint32_t)(wma_numerator / s_freq_weight_sum);
    int32_t wma_d = (int32_t)wma - (int32_t)ld->wma;

    ld->wma = wma;
    ld->deriv_buffer[ld->deriv_i] = wma_d;
    ld->deriv_i = (ld->deriv_i + 1) % DERIV_WINDOW_SIZE;

    // Then WMA of the derivative buffer
    wma_numerator = 0;
    for (int i = 0; i < DERIV_WINDOW_SIZE; i++) {
        int weight = i + 1;
        int j = (ld->deriv_i + i) % DERIV_WINDOW_SIZE;
        wma_numerator += ((uint64_t)ld->deriv_buffer[j]) * weight;
    }

    int32_t wma_d_wma = (uint32_t)(wma_numerator / s_deriv_weight_sum);

    // Safety threshold check
    // We need to pass through this frequency threshold to be a valid dive.
    // We just use the simple data values, not the averages.
    if (homing_flags & LH_AWAIT_HOMING) {
        if (data < ld->safe_start_freq)
            return;

        // And we need to do it _after_ this time, to make sure we didn't
        // start below the thershold
        if (timer_is_before(time, ld->safe_start_time)) {
            dprint("ZZZ EARLY! time=%u < %u", time, ld->safe_start_time);
            notify_trigger(ld1, 0, ld->other_reason_base + REASON_ERROR_PROBE_TOO_LOW);
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

    //dprint("data=%u avg %u", data, avg);
    if (wma > ld->homing_trigger_freq) {
        notify_trigger(ld1, time, ld->success_reason);
        dprint("ZZZ trig t=%u a=%u (d=%u)", time, wma, data);
    }
}