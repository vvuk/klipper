#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>

#define dprint(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")

#define SAMPLE_ERR(data) ((data) >> 28)
enum {
    LDC_PENDING = 1<<0, LDC_HAVE_INTB = 1<<1,
    LH_AWAIT_HOMING = 1<<1, LH_AWAIT_TAP = 1<<2,
    LH_CAN_TRIGGER = 1<<3, LH_WANT_TAP = 1<<4,
    LH_V2 = 1<<5,
};

// Configuration
#define FREQ_WINDOW_SIZE 16

struct ldc1612_v2 {
    // Used from parent:
    // homing_flags
    // ts

    // Note: this entire struct is zeroed in setup_home2. 
    // If this becomes used for persistent config data,
    // fix that by moving the home-state tracking to its
    // own struct.

    uint32_t freq_buffer[FREQ_WINDOW_SIZE];
    // current index in freq/deriv buffers
    uint8_t freq_i;

    uint32_t wma; // last computed weighted moving average
    uint32_t wma_d; // last computed wma derivative
    // where we keep track
    uint32_t tap_accum;
    // the earliest start of this tap
    float tap_start_time;

    // the time we fired a trigger (same as homing_clock in parent struct,
    // due to code structure)
    float trigger_time;

    //
    // input parameters
    //

    // frequency we must pass through to have a valid home/tap
    uint32_t safe_start_freq;
    // and it must happen after this time
    uint32_t safe_start_time;

    // the frequency to trigger on for homing, or
    // the second threshold before we start looking for a tap
    uint32_t homing_trigger_freq;

    // the tap detection threshold: specifically, the total downward
    // change in the frequency derivative before we see a direction
    // reveral (the windowed moving average of the derivative of the wmd
    // to be exact)
    uint32_t tap_threshold;

    // trigger reasons
    uint8_t success_reason;
    uint8_t other_reason_base;

};

void check_home2(struct ldc1612_v2* ld, uint32_t data, float time);

FILE* s_out;

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <csv_file>\n", argv[0]);
        return 1;
    }

    const char *filename = argv[1];
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        return 1;
    }

    char line[1024];
    // Skip the header line
    if (!fgets(line, sizeof(line), file)) {
        fprintf(stderr, "Error reading file or empty file\n");
        fclose(file);
        return 1;
    }

    struct ldc1612_v2 ld2;
    memset(&ld2, 0, sizeof(ld2));
    struct ldc1612_v2 *ld = &ld2;

    ld->safe_start_freq = 70873421;
    ld->homing_trigger_freq = 71180949;
    ld->tap_threshold = 1500;

    float tfirst = 0.0;

    sprintf(line, "proc-%s", filename);
    s_out = fopen(line, "w");
    fprintf(s_out, "time,freq,wma,wma_d,accum\n");

    // Read and process each subsequent line
    while (fgets(line, sizeof(line), file)) {
        float time, frequency, z, kin_z, kin_v;
        uint32_t raw_f;
        if (!sscanf(line, "%f,%f,%f,%f,%f,%u", &time, &frequency, &z, &kin_z, &kin_v, &raw_f) == 6) {
            fprintf(stderr, "Malformed line: %s", line);
            continue;
        }
        if (tfirst == 0.0) {
            tfirst = time;
        }
        time -= tfirst;

        check_home2(ld, raw_f, time);
    }

    fclose(file);
    return 0;
}

#define WEIGHT_SUM(size) ((size * (size + 1)) / 2)

void
check_home2(struct ldc1612_v2* ld, uint32_t data, float time)
{
    // WTB constexpr
    static uint64_t s_freq_weight_sum = WEIGHT_SUM(FREQ_WINDOW_SIZE);

    static uint8_t homing_flags = LH_WANT_TAP | LH_AWAIT_HOMING | LH_V2;
    uint8_t is_tap = !!(homing_flags & LH_WANT_TAP);

    if (SAMPLE_ERR(data)) {
        // TODO: test homing from very high up
        dprint("ZZZ home2 err=%u", data);
        // ignore amplitude errors (likely probe too far),
        // unless we're tapping, in which case we should consider
        // all errors for safety -- it means the tap wasn't started
        // at an appropriate distance
        //if ((!is_tap && ld1->prev_status & STATUS_ERR_AHE) != 0)
        //    return;

        // Sensor reports an issue - cancel homing
        //notify_trigger(ld1, 0, ld->other_reason_base + REASON_ERROR_SENSOR);
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
        int j = (ld->freq_i + i) % FREQ_WINDOW_SIZE;
        uint64_t weight = i + 1;
        uint64_t val = ld->freq_buffer[j];
        wma_numerator += val * weight;
    }

    // WMA and derivative of the WMA
    uint32_t wma = (uint32_t)(wma_numerator / s_freq_weight_sum);
    int32_t wma_d = (int32_t)wma - (int32_t)ld->wma;

    if (wma_d < ld->wma_d) {
        // derivative is decreasing; track it
        ld->tap_accum += ld->wma_d - wma_d;
    } else {
        // derivative is increasing; reset the accumulator,
        // and reset the tap time
        ld->tap_accum = 0;
        ld->tap_start_time = time;
    }

    ld->wma = wma;
    ld->wma_d = wma_d;

    fprintf(s_out, "%f,%u,%u,%d,%d\n", time, data, wma, wma_d, ld->tap_accum);

    // Safety threshold check
    // We need to pass through this frequency threshold to be a valid dive.
    // We just use the simple data values, not the averages.
    if (homing_flags & LH_AWAIT_HOMING) {
        if (data < ld->safe_start_freq)
            return;

        // And we need to do it _after_ this time, to make sure we didn't
        // start below the thershold
        //if (timer_is_before(time, ld->safe_start_time)) {
        //    dprint("ZZZ EARLY! time=%f < %f", time, ld->safe_start_time);
        //    return;
        //}

        dprint("ZZZ safe start");

        if (is_tap && ld->homing_trigger_freq != 0) {
            // If we're tapping, then make the homing trigger freq a second thershold.
            // These would typically be set to something like the 3.0mm freq for the first,
            // then the 2.0mm homing freq.
            ld->safe_start_freq = ld->homing_trigger_freq;
            ld->homing_trigger_freq = 0;
            return;
        }

        // Ok, we've passed all the safety thresholds. Values from this point on
        // will be considered for homing/tapping
        homing_flags &= ~LH_AWAIT_HOMING;
        ld->tap_accum = 0;
    }

    //dprint("data=%u avg %u", data, avg);

    if (!is_tap) {
        if (wma > ld->homing_trigger_freq) {
            ld->trigger_time = time;
            dprint("ZZZ trig t=%f a=%u (f=%u)", time, wma, data);
        }
    } else {
        if (ld->tap_accum > ld->tap_threshold) {
            ld->trigger_time = time;
            dprint("ZZZ tap t=%f n=%f l=%u (f=%u)", ld->tap_start_time, time, ld->tap_accum, data);
        }
    }
}