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
    LH_CAN_TRIGGER = 1<<3, LH_WANT_TAP = 1<<4
};

#define HCOUNT 64

struct ldc1612 {
    struct timer timer;
    uint32_t rest_ticks;
    struct i2cdev_s *i2c;
    uint8_t flags;
    struct sensor_bulk sb;
    struct gpio_in intb_pin;
    uint16_t prev_status;

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

    uint8_t hnext;
    uint32_t htime[HCOUNT];
    int32_t havg[HCOUNT];
    int32_t hcavg[HCOUNT];
    int32_t hadj[HCOUNT];
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
        ld->ts = NULL;
        ld->homing_flags = 0;
        return;
    }
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
    ld->hnext = 0;
    memset(ld->htime, 0, 4*HCOUNT);
    memset(ld->havg, 0, 4*HCOUNT);
    memset(ld->hcavg, 0, 4*HCOUNT);
    memset(ld->hadj, 0, 4*HCOUNT);
    if (ld->tap_threshold)
        // Homing until a nozzle/bed contact is detected
        ld->homing_flags = (LH_AWAIT_HOMING | LH_CAN_TRIGGER
                            | LH_AWAIT_TAP | LH_WANT_TAP);
    else
        // Homing until threshold met
        ld->homing_flags = LH_AWAIT_HOMING | LH_CAN_TRIGGER;

    dprint("ZZZ setup home");
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
command_query_ldc1612_hack(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);
    int idx = ld->hnext;
    sendf("ldc1612_hack oid=%c time=%u avg=%i cavg=%i adj=%i",
          args[0], ld->htime[idx], ld->havg[idx], ld->hcavg[idx], ld->hadj[idx]);
    ld->hnext = (idx + 1) % HCOUNT;
}
DECL_COMMAND(command_query_ldc1612_hack,
             "query_ldc1612_hack oid=%c");


void
command_query_ldc1612_latched_status(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);
    sendf("ldc1612_latched_status oid=%c status=%u"
          , args[0], ld->prev_status);
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
}

// Check if a sample should trigger a homing event
static void
check_home(struct ldc1612 *ld, uint32_t data)
{
    uint8_t hidx = ld->hnext++;

    uint8_t homing_flags = ld->homing_flags;
    if (!(homing_flags & LH_CAN_TRIGGER))
        return;
    if (data > 0x0fffffff) {
        ld->havg[hidx] = ld->prev_status;
        ld->hcavg[hidx] = data;
        ld->hadj[hidx] = 900033;

        // ignore over-amplitude errors (probe too far)
        if ((ld->prev_status & STATUS_ERR_AHE) != 0)
            return;

        // Sensor reports an issue - cancel homing
        notify_trigger(ld, 0, ld->error_reason);
        return;
    }
    // Perform sensor averaging
    uint32_t ema_factor = ld->ema_factor;

    // EMA
    uint32_t scaled_prev = ld->sensor_average * ema_factor;
    uint32_t scaled_data = data * (EMA_BASE - ema_factor);
    uint32_t new_avg = DIV_ROUND_CLOSEST(scaled_data + scaled_prev, EMA_BASE);

    ld->sensor_average = new_avg;
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

    ld->htime[hidx] = time;
    ld->havg[hidx] = new_avg;
    ld->hcavg[hidx] = new_cavg;

    if (ld->hnext == HCOUNT)
        ld->hnext = 0;

    if (homing_flags & LH_AWAIT_HOMING) {
        if (timer_is_before(time, ld->homing_clock))
            return;
        ld->homing_flags = homing_flags = homing_flags & ~LH_AWAIT_HOMING;
    }

    if (!(homing_flags & LH_WANT_TAP)) {
        // Trigger on simple threshold check
        ld->hadj[hidx] = 900077;
        if (new_avg > ld->trigger_threshold)
            notify_trigger(ld, time, ld->trigger_reason);
        return;
    }

    if (homing_flags & LH_AWAIT_TAP) {
        // Check if can start tap detection
        if (timer_is_before(time, ld->tap_clock)) {
            ld->hadj[hidx] = 900099;
            if (new_avg > ld->trigger_threshold) {
                ld->hadj[hidx] = 900088;
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
    ld->hadj[hidx] = adjust;

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
static uint32_t
ldc1612_query_one(struct ldc1612 *ld)
{
    // Check if data available (and clear INTB line)
    uint16_t status = read_reg_status(ld);
    irq_disable();
    ld->flags &= ~LDC_PENDING;
    irq_enable();
    if (!(status & 0x08))
        return 0;

    // Read coil0 frequency
    uint8_t d[4];
    read_reg(ld, REG_DATA0_MSB, &d[0]);
    read_reg(ld, REG_DATA0_LSB, &d[2]);

    return ((uint32_t)d[0] << 24)
        | ((uint32_t)d[1] << 16)
        | ((uint32_t)d[2] << 8)
        | ((uint32_t)d[3]);
}

static void
ldc1612_query(struct ldc1612 *ld, uint8_t oid)
{
    uint32_t data = ldc1612_query_one(ld);
    if (data == 0)
        return;

    // Store data in streaming buffer    
    uint8_t *d = &ld->sb.data[ld->sb.data_count];
    memcpy(d, &data, BYTES_PER_SAMPLE);
    ld->sb.data_count += BYTES_PER_SAMPLE;

    // Check for endstop trigger
    check_home(ld, data);

    // Flush local buffer if needed
    if (ld->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(ld->sb.data))
        sensor_bulk_report(&ld->sb, oid);
}

void
command_query_ldc1612(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);

    sched_del_timer(&ld->timer);
    ld->flags &= ~LDC_PENDING;
    if (!args[1])
        // End measurements
        return;

    // Start new measurements query
    ld->rest_ticks = args[1];
    sensor_bulk_reset(&ld->sb);
    irq_disable();
    ld->timer.waketime = timer_read_time() + ld->rest_ticks;
    sched_add_timer(&ld->timer);
    irq_enable();
}
DECL_COMMAND(command_query_ldc1612, "query_ldc1612 oid=%c rest_ticks=%u");

void
command_query_status_ldc1612(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);

    if (ld->flags & LDC_HAVE_INTB) {
        // Check if a sample is pending in the chip via the intb line
        irq_disable();
        uint32_t time = timer_read_time();
        int p = check_intb_asserted(ld);
        irq_enable();
        sensor_bulk_status(&ld->sb, args[0], time, 0, p ? BYTES_PER_SAMPLE : 0);
        return;
    }

    // Query sensor to see if a sample is pending
    uint32_t time1 = timer_read_time();
    uint16_t status = read_reg_status(ld);
    uint32_t time2 = timer_read_time();

    uint32_t fifo = status & 0x08 ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&ld->sb, args[0], time1, time2-time1, fifo);
}
DECL_COMMAND(command_query_status_ldc1612, "query_status_ldc1612 oid=%c");

void
command_ldc1612_read(uint32_t *args)
{
    struct ldc1612 *ld = oid_lookup(args[0], command_config_ldc1612);

    uint32_t time1 = timer_read_time();
    uint32_t value = ldc1612_query_one(ld);

    sendf("ldc1612_read_reply oid=%c time=%u val=%u", args[0], time1, value);
}
DECL_COMMAND(command_ldc1612_read, "ldc1612_read oid=%c");

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
