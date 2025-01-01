from __future__ import annotations

import logging, math, bisect
import mcu
import numpy as np
import numpy.polynomial as npp

import pins

from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, List, Optional, TypedDict, final

from clocksync import SecondarySync
from configfile import ConfigWrapper
from gcode import GCodeCommand
from klippy import Printer
from mcu import MCU, MCU_trsync
from stepper import MCU_stepper
from webhooks import WebRequest
from toolhead import ToolHead

from .temperature_probe import TemperatureProbe
from .temperature_sensor import PrinterSensorGeneric

from . import ldc1612, probe, manual_probe

from .ldc1612 import SETTLETIME as LDC1612_SETTLETIME

# How many seconds from the start of probing must we be below the
# trigger start frequency (above the start homing height)
HOME_TRIGGER_START_TIME_OFFSET = 0.200

@final
class ProbeEddyFrequencyMap:
    def __init__(self, eddy: ProbeEddy):
        self._eddy = eddy
        self._sensor = eddy._sensor
        self._temp_sensor = eddy._temp_sensor

    def load_from_config(self, config: ConfigWrapper):
        ftoh = config.getfloatlist("freq_to_height_p", default=None)
        htof = config.getfloatlist("height_to_freq_p", default=None)

        if ftoh and htof:
            self._freq_to_height = npp.Polynomial(ftoh)
            self._height_to_freq = npp.Polynomial(htof)
            logging.info(f"Loaded polynomial freq-to-height: {self._coefs(self._freq_to_height)}\n" + \
                f"                  height-to-freq: {self._coefs(self._height_to_freq)}")
        else:
            self._freq_to_height = None
            self._height_to_freq = None

    # helper. numpy.Polynomial .coef returns coefficients with domain/range mapping baked in.
    # until we store those, those are no good for round-tripping. convert() gives us
    # the unscaled values.
    def _coefs(self, p):
        return p.convert().coef.tolist()

    def calibrate_from_values(self, raw_freqs: List[float], raw_heights: List[float], gcmd: Optional[GCodeCommand] = None):
        if len(raw_freqs) != len(raw_heights):
            raise ValueError("freqs and heights must be the same length")
        raw_freqs = np.array(raw_freqs)
        raw_heights = np.array(raw_heights)

        # Group frequencies by heights and compute the median for each height
        heights = np.unique(raw_heights)
        freqs = np.array([np.median(raw_freqs[raw_heights == h]) for h in heights])

        def r2(p, x, y):
            y_hat = p(x)
            ss_res = np.sum((y - y_hat)**2)
            ss_tot = np.sum((y - np.mean(y))**2)
            return 1 - (ss_res / ss_tot)
        
        r2_tolerance = 0.95

        # empirically, freq-to-position results need a 3D polynomial to hit R^2=1
        # where position-to-frequency fits cleanly with a 2D polynomial. But find
        # the best r^2 
        for i in range(3, 9):
            self._freq_to_height = npp.Polynomial.fit(freqs, heights, i)
            r2_fth = r2(self._freq_to_height, freqs, heights)
            if r2_fth >= r2_tolerance:
                break
        
        for i in range(3, 9):
            self._height_to_freq = npp.Polynomial.fit(heights, freqs, i)
            r2_htf = r2(self._height_to_freq, heights, freqs)
            if r2_htf >= r2_tolerance:
                break

        msg = f"Calibrated polynomial freq-to-height (R^2={r2_fth:.3f}): {self._coefs(self._freq_to_height)}\n" + \
              f"                      height-to-freq (R^2={r2_htf:.3f}): {self._coefs(self._height_to_freq)}"
        logging.info(msg)
        if gcmd:
            gcmd.respond_info(msg, gcmd)
            if r2_htf < r2_tolerance or r2_fth < r2_tolerance:
                gcmd.respond_raw("!! R^2 values are below tolerance; calibration may not be accurate.\n")

        return (r2_fth, r2_htf)


    def save_calibration(self, gcmd: Optional[GCodeCommand] = None):
        if not self._freq_to_height or not self._height_to_freq:
            return

        ftohs = str(self._coefs(self._freq_to_height)).replace("[","").replace("]","")
        htofs = str(self._coefs(self._height_to_freq)).replace("[","").replace("]","")

        configfile = self._eddy._printer.lookup_object('configfile')
        # why is there a floatarray getter, but not a setter?
        configfile.set(self._eddy._full_name, "freq_to_height_p", ftohs)
        configfile.set(self._eddy._full_name, "height_to_freq_p", htofs)

        if gcmd:
            gcmd.respond_info("Calibration saved. Issue a SAVE_CONFIG to write the values to your config file and restart Klipper.")

    def raw_freqval_to_height(self, raw_freq: int, temp: float = None) -> float:
        if raw_freq > 0x0fffffff:
            return -math.inf #error bits set
        return self.freq_to_height(self._sensor.from_ldc_freqval(raw_freq))

    def height_to_raw_freqval(self, height: float, temp: float = None) -> int:
        if self._height_to_freq is None:
            return 0
        return self._sensor.to_ldc_freqval(self._height_to_freq(height))

    def freq_to_height(self, freq: float, temp: float = None) -> float:
        if self._freq_to_height is None:
            return math.inf
        return float(self._freq_to_height(freq))

    def height_to_freq(self, height: float, temp: float = None) -> float:
        if self._height_to_freq is None:
            return math.inf
        return float(self._height_to_freq(height))

    def calibrated(self) -> bool:
        return self._freq_to_height is not None and self._height_to_freq is not None

@final
class ProbeEddy:
    def __init__(self, config: ConfigWrapper):
        logging.info("Hello from ProbeEddy")

        self._printer: Printer = config.get_printer()
        self._full_name = config.get_name()
        self._name = self._full_name.split()[-1]

        sensors = { "ldc1612": ldc1612.LDC1612 }
        sensor_type = config.getchoice('sensor_type', {s: s for s in sensors})

        self._sensor = sensors[sensor_type](config)
        self._mcu = self._sensor.get_mcu()
        self._temp_sensor: TemperatureProbe = config.printer.load_object(config, f'temperature_probe {self._name}')

        # these are defaults
        self.params = {
            'speed': 5.0,
            'lift_speed': 5.0,
            'probe_speed': 5.0,
            'backlash_comp': 0.5,

            # what height to trigger homing at
            'home_trigger_height': 2.0,
            # What height above home_trigger_height to allow homing to start
            'home_trigger_height_start_offset': 1.5,
            'trigger_freq_slop': 0.006,

            'calibration_z_max': 5.0,
            'calibration_step': 0.040,
        }

        for k in self.params:
            if 'speed' in k:
                self.params[k] = config.getfloat(k, self.params[k], above=0.0)
            else:
                self.params[k] = config.getfloat(k, self.params[k])

        self._validate_params()

        # at what minimum physical height to start homing
        self._home_start_height = self.params['calibration_z_max']

        # physical offsets between probe and nozzle
        self.offset = {
            "x": config.getfloat("x_offset", 0.0),
            "y": config.getfloat("y_offset", 0.0),
        }

        self._fmap = ProbeEddyFrequencyMap(self)
        self._fmap.load_from_config(config)

        self._endstop_wrapper = ProbeEddyEndstopWrapper(self)

        # We are a "PrinterProbe". We use some existing helpers.
        # ProbeSessionHelper creates a HomingViaProbeHelper which registers the probe
        # z endstop pin. They all connect to _endstop_wrapper, which is the "mcu_probe".
        self._probe_session = probe.ProbeSessionHelper(config, self._endstop_wrapper)
        logging.info(f"EDDY probe_session: {self._probe_session}")
        self._printer.add_object('probe', self)

        # update some local param copies. We need to do this after creating a ProbeSessionHelper,
        # because it pulls some things from the probe_params
        self._endstop_wrapper.pull_params()

        # define PROBE, PROBE_ACCURACY, etc.
        self._cmd_helper = probe.ProbeCommandHelper(config, self, self._endstop_wrapper.query_endstop)

        # define our own commands
        gcode = self._printer.lookup_object('gcode')
        self.define_commands(gcode)

    def _validate_params(self):
        home_trigger_height = self.params['home_trigger_height']
        home_trigger_height_start_offset = self.params['home_trigger_height_start_offset']
        calibration_z_max = self.params['calibration_z_max']

        # verify a few things
        if home_trigger_height < 0.0:
            raise self._printer.config_error("home_trigger_height must be >= 0 (and really should be > 1)")
        if home_trigger_height_start_offset < 0.5:
            raise self._printer.config_error("home_trigger_height_start_offset must be >= 0.5")
        req_cal_z_max = home_trigger_height_start_offset + 2*home_trigger_height_start_offset
        if calibration_z_max < req_cal_z_max:
            raise self._printer.config_error(f"calibration_z_max must be at least 2*home_trigger_height_start_offset+home_trigger_height (2*{home_trigger_height_start_offset:.3f}+{home_trigger_height_start_offset:.3f}={req_cal_z_max:.3f})")

    def define_commands(self, gcode):
        gcode.register_command("PROBE_EDDY_QUERY", self.cmd_QUERY)
        gcode.register_command("PROBE_EDDY_CALIBRATE", self.cmd_CALIBRATE)
        gcode.register_command("PROBE_EDDY_PROBE_STATIC", self.cmd_PROBE_STATIC)
    
    def cmd_QUERY(self, gcmd: GCodeCommand):
        reactor = self._printer.get_reactor()

        self._sensor._start_measurements()
        systime = reactor.monotonic()
        reactor.pause(systime + 0.050)
        status, freqval, freq = self._sensor.read_one_value()
        self._sensor._finish_measurements()

        height = self._fmap.freq_to_height(freq)
        gcmd.respond_info(f"Last coil value: {freq:.2f} ({height:.3f}mm) (raw: {hex(freqval)} {self._sensor.status_to_str(status)} {hex(status)})")

    def cmd_PROBE_STATIC(self, gcmd: GCodeCommand):
        if not self._fmap.calibrated():
            gcmd.respond_raw("!! Probe not calibrated\n")
            return

        duration = gcmd.get_float('DURATION', 0.100, above=0.0)
        mean_or_median = gcmd.get('METHOD', 'mean').lower()

        reactor = self._printer.get_reactor()

        with ProbeEddySampler(self) as sampler:
            sampler.wait_for_samples(max_wait_time=duration*2, min_samples=50)
            sampler.finish()

            samples = sampler.get_samples()
            if len(samples) == 0:
                gcmd.respond_raw("!! No samples collected\n")
                return

            # skip LDC1612_SETTLETIME samples at start and end by looking
            # at the time values
            stime = samples[0][0]
            etime = samples[-1][0]
            samples = [s for s in samples if s[0] > stime+LDC1612_SETTLETIME and s[0] < etime-LDC1612_SETTLETIME]

            mean = np.mean([s[2] for s in samples])
            median = np.median([s[2] for s in samples])

            if mean_or_median == 'mean':
                height = mean
            elif mean_or_median == 'median':
                height = median
            else:
                gcmd.respond_raw("!! Invalid METHOD\n")
                return

            gcmd.respond_info(f"Collected {len(samples)} samples, height: {height:.3f} (mean: {mean:.3f}, median: {median:.3f})")
            self._cmd_helper.last_z_result = height

    def read_current_freq_and_height(self):
        self._sensor._start_measurements()
        status, freqval, freq = self._sensor.read_one_value()
        self._sensor._finish_measurements()

        # if in error, return -inf height to make endstop checks easier
        if freqval > 0x0fffffff:
            return None, -math.inf

        height = self._fmap.freq_to_height(freq)
        return freq, height

    def cmd_CALIBRATE(self, gcmd: GCodeCommand):
        # z-hop so that manual probe helper doesn't complain if we're already
        # at the right place
        toolhead: ToolHead = self._printer.lookup_object('toolhead')
        curpos = toolhead.get_position()
        curpos[2] = curpos[2] + 5
        toolhead.manual_move(curpos, self.params['probe_speed'])

        manual_probe.ManualProbeHelper(self._printer, gcmd,
                                       lambda kin_pos: self.cmd_CALIBRATE_next(gcmd, kin_pos))

    def cmd_CALIBRATE_next(self, gcmd: GCodeCommand, kin_pos: float):
        if kin_pos is None:
            return

        # We just did a ManualProbeHelper; so we want to tell the printer
        # the current position is actually zero.
        kin_pos[2] = 0.0

        toolhead: ToolHead = self._printer.lookup_object('toolhead')
        curpos = toolhead.get_position()
        logging.info(f"CALIBRATE: kin_pos: {kin_pos} curpos: {curpos}")

        sample_time = 0.200
        sample_pad = 0.050

        with ToolheadMovementHelper(self) as th:
            # We're going to set the kinematic position to kin_height to make
            # the following code easier, so it can assume z=0 is actually real zero.
            # The Eddy sensor calibration is done to nozzle height (not sensor height).
            th.set_absolute_position(*kin_pos)

            # away from bed, over nozzle position, then back to bed

            #  FIXME FIXME FIXME -- PUT THIS BACK! -- FIXME FIXME FIXME
            # move the probe to where the nozzle was (i.e. where we set nozzle zero)
            # TODO -- this might not actually matter. We're going to do tap,
            # so we just need a "good enough"
            #th.move(0, 0, 5.0)
            #th.move(-self.offset['x'], -self.offset['y'], 0)
            # or in one go:
            #th.move_by(-self.offset['x'], -self.offset['y'], 5.0)
            th.move_by(0, 0, 5.0)
            th.dwell(0.5)

            #th.move(0, 0, -5.0)

            # now do the calibration move.
            # move to the start of calibration
            cal_z_max = self.params['calibration_z_max']
            th.move_to_z(cal_z_max)

            mapping = None
            toolhead_positions = []
            first_sample_time = None
            last_sample_time = None

            with ProbeEddySampler(self) as sampler:
                # move down in steps, taking samples
                # note: np.arange is exclusive of the stop value; this will _not_ include z=0
                for z in np.arange(cal_z_max, 0, -self.params['calibration_step']):
                    # hop up, then move downward to z to reduce backlash
                    #th.move_to_z(z + 1.000)
                    th.move_to_z(z)

                    th_tstart, th_tend, th_z = th.note_time_and_position(sample_time)
                    logging.info(f"th: {th_tstart} {th_z:.3f}")

                    first_sample_time = first_sample_time or th_tstart
                    last_sample_time = th_tend

            # back to a safe spot
            th.move_to_z(cal_z_max)

        # verify historical movements
        # TODO -- figure out if get_kin_z(at=) actually gives useful values
        for th_past_time, th_recorded_z in toolhead_positions:
            th_past_z = th.get_kin_z(at=th_past_time)
            logging.info(f"EDDY calibration: time: {th_past_time} z: {th_recorded_z:.3f} past_z: {th_past_z:.3f}")

        # the samples are a list of [print_time, freq] pairs.
        # in toolhead_positions, we have a list of [print_time, kin_z] pairs.
        samples = sampler.get_samples()
        movement_notes = th.notes()

        freqs = []
        heights = []

        data_file = open("/tmp/eddy-samples.csv", "w")
        data_file.write(f"time,frequency,z\n")

        # we know that both the movement notes and the samples are going to be in increasing time.
        # so we can walk through both in parallel
        si = iter(samples)
        try:
            for ni in range(len(movement_notes)):
                n_start, n_end, n_z = movement_notes[ni]

                s_t, s_freq, _ = next(si)
                while s_t < (n_start+sample_pad):
                    data_file.write(f"{s_t},{s_freq},\n")
                    s_t, s_freq, _ = next(si)
                
                while s_t < (n_end-sample_pad):
                    data_file.write(f"{s_t},{s_freq},{n_z}\n")
                    freqs.append(s_freq)
                    heights.append(n_z)
                    s_t, s_freq, _ = next(si)
        except StopIteration:
            pass

        data_file.close()
        gcmd.respond_info(f"Wrote {len(samples)} samples to /tmp/eddy-samples.csv")

#        for s_t, s_freq, _ in samples:
#            logging.info(f"calibration: sample: {s_t} {s_freq:.2f}")
#            # TODO: make this more elegant
#            for n_start, n_end, n_z in movement_notes:
#                if (n_start+sample_pad) <= s_t <= (n_end-sample_pad):
#                    freqs.append(s_freq)
#                    heights.append(n_z)
#                    data_file.write(f"{s_t},{s_freq},{n_z}\n")
#
#                    logging.info(f"calibration: toolhead: {s_t} {s_freq:.2f} {n_z:.3f}")

        gcmd.respond_info(f"Collected {len(samples)} samples, filtered to {len(freqs)} for {len(movement_notes)} steps")

        if len(freqs) == 0:
            gcmd.respond_raw("!! No samples collected\n")
            return

        # and build a map
        mapping = ProbeEddyFrequencyMap(self)
        r2_fth, r2_htf = mapping.calibrate_from_values(freqs, heights, gcmd)
        mapping.save_calibration(gcmd)
        self._fmap = mapping

        gcmd.respond_info(f"Calibration complete, R^2: freq-to-height={r2_fth:.3f}, height-to-freq={r2_htf:.3f}")

    # helpers to forward to the map
    def height_to_freq(self, height: float, temp: float = None) -> float:
        return self._fmap.height_to_freq(height, temp)
    
    def freq_to_height(self, freq: float, temp: float = None) -> float:
        return self._fmap.freq_to_height(freq, temp)

    def calibrated(self) -> bool:
        return self._fmap.calibrated()

    # PrinterProbe interface
    # I am seriously lost with PrinterProbe, ProbeEndstopWrapper, ProbeSessionWrapper, etc.
    # it's not clear at all to me what the relationships are between all of these.
    # PrinterProbe is what `probe` typically is, so we're instead emulating it here.
    def get_offsets(self):
        # the z offset is the trigger height, because the probe will trigger
        # at z=trigger_height (not at z=0)
        return self.offset["x"], self.offset["y"], self.params['home_trigger_height']
    
    def get_probe_params(self, gcmd=None):
        probe_params = self._probe_session.get_probe_params(gcmd)
        sample_retract_dist = probe_params['sample_retract_dist']
        # retract to at least the home start height
        if sample_retract_dist + self.params['home_trigger_height'] < self._home_start_height + 0.500:
            # give ourselves a bit of room to make sure we hit it, due to stepper precision;
            # also because the toolhead will keep moving down before the probe trigger so it'll
            # miss the home_trigger_height
            probe_params['sample_retract_dist'] = self._home_start_height + 0.500 - self.params['home_trigger_height']
        return probe_params

    def start_probe_session(self, gcmd):
        method = gcmd.get('METHOD', 'automatic').lower()
        if method in ('scan', 'rapid_scan'):
            z_offset = self.get_offsets()[2]
            raise NotImplementedError("scan and rapid_scan not implemented")
            #return EddyScanningProbe(self.printer, self.sensor_helper, self.calibration, z_offset, gcmd)
        return self._probe_session.start_probe_session(gcmd)
    
    def get_status(self, eventtime):
        status = self._cmd_helper.get_status(eventtime)
        status.update({ 'name': self._full_name, 'home_trigger_height': self.params['home_trigger_height'] })
        return status


# Helper to gather samples and convert them to probe positions
@final
class ProbeEddySampler:
    def __init__(self, eddy: ProbeEddy):
        self.eddy = eddy
        self._sensor = eddy._sensor
        self._reactor = self.eddy._printer.get_reactor()
        self._mcu = self._sensor.get_mcu()
        # this will hold the raw samples coming in from the sensor,
        # with an empty 3rd (height) value
        self._raw_samples = None
        self._stopped = False
        self._started = False
        self._errors = 0

        # this will hold samples with a height filled in
        self._samples = None
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.finish()

    # bulk sample callback for when new data arrives
    # from the probe
    def _add_hw_measurement(self, msg):
        if self._stopped:
            return False
        
        self._errors += msg['errors']
        self._raw_samples.extend(msg['data'])
        return True

    def start(self):
        if not self._started:
            self._raw_samples = []
            self._sensor.add_client(self._add_hw_measurement)
            self._started = True
    
    def finish(self):
        if not self._started:
            raise Exception("ProbeEddySampler.finish() called without start()")
        self._stopped = True

    def _update_samples(self):
        if self._samples is None:
            self._samples = []
        start_idx = len(self._samples)
        new_samples = [(t, f, self.eddy.freq_to_height(f)) for t, f, _ in self._raw_samples[start_idx:]]
        self._samples.extend(new_samples)

    def get_raw_samples(self):
        return self._raw_samples.copy()

    def get_samples(self):
        self._update_samples()
        return self._samples.copy()
    
    def get_error_count(self):
        return self._errors

    # get the last sampled height 
    def get_last_height(self):
        self._update_samples()
        if len(self._samples) == 0:
            return None
        return self._samples[-1][2]

   # Wait until a sample for the given time arrives
    def wait_for_sample_at_time(self, sample_print_time, max_wait_time=0.250) -> bool:
        if self._stopped:
            # if we're not getting any more samples, we can check directly
            if len(self._raw_samples) == 0:
                return False;
            return self._raw_samples[-1][0] >= sample_print_time

        wait_start_time = self._mcu.estimated_print_time(self._reactor.monotonic())

        # if sample_print_time is in the future, make sure to wait max_wait_time
        # past the expected time
        if sample_print_time > wait_start_time:
            max_wait_time = max_wait_time + (sample_print_time - wait_start_time)

        logging.info(f"ProbeEddyFrequencySampler: waiting for sample at {sample_print_time:.3f} (now: {wait_start_time:.3f}, max_wait_time: {max_wait_time:.3f})")
        while len(self._raw_samples) == 0 or self._raw_samples[-1][0] < sample_print_time:
            now = self._mcu.estimated_print_time(self._reactor.monotonic())
            if now - wait_start_time > max_wait_time:
                return False
            self._reactor.pause(now + 0.010)
        
        return True

    # Wait for some samples to be collected, even if errors
    def wait_for_samples(self, max_wait_time=0.250, count_errors=False, min_samples=1, new_only=False, raise_error=True):
        # Make sure enough samples have been collected
        wait_start_time = self._mcu.estimated_print_time(self._reactor.monotonic())

        if new_only:
            start_count = len(self._raw_samples) + (self._errors if count_errors else 0)
        else:
            start_count = 0

        while (len(self._raw_samples) + (self._errors if count_errors else 0)) - start_count < min_samples:
            now = self._mcu.estimated_print_time(self._reactor.monotonic())
            if now - wait_start_time > max_wait_time:
                if raise_error:
                    raise self.eddy._printer.command_error(f"possible probe_eddy sensor outage; no samples for {max_wait_time:.2f}s")
                else:
                    return False
            self._reactor.pause(now + 0.010)
        
        return True

    def find_closest_height_at_time(self, time):
        self._update_samples()
        if len(self._samples) == 0:
            return None
        # find the closest sample to the given time
        idx = np.argmin([abs(t - time) for t, _, _ in self._samples])
        return self._samples[idx][2]

    def find_height_at_time(self, start_time, end_time):
        if end_time < start_time:
            raise ValueError("find_height_at_time: end_time is before start_time")

        self._update_samples()

        logging.info(f"find_height_at_time: searching between {start_time:.3f} and {end_time:.3f}")
        if len(self._samples) == 0:
            logging.info(f"    zero samples!")
            return None

        logging.info(f"find_height_at_time: {len(self._samples)} samples, time range {self._samples[0][0]:.3f} to {self._samples[-1][0]:.3f}")

        # find the first sample that is >= start_time
        start_idx = bisect.bisect_left([t for t, _, _ in self._samples], start_time)
        if start_idx >= len(self._samples):
            logging.info(f"    nothing after start_time!")
            # nothing in range
            return None
        
        # find the last sample that is <= end_time
        end_idx = bisect.bisect_right([t for t, _, _ in self._samples], end_time)
        if end_idx == 0:
            raise ValueError("found something at start_time, but not before end_time?")
        
        # average the heights of the samples in the range
        heights = [h for _, _, h in self._samples[start_idx:end_idx]]
        hmin, hmax = np.min(heights), np.max(heights)
        mean = np.mean(heights)
        median = np.median(heights)
        logging.info(f"find_height_at_time: {len(heights)} samples, mean: {mean:.3f} median: {median:.3f} (range {hmin:.3f}-{hmax:.3f})")

        return float(median)


# This is a ProbeEndstopWrapper-compatible class,
# which also forwards the "mcu_probe" methods.
@final
class ProbeEddyEndstopWrapper:
    REASON_BASE = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 1
    REASON_SENSOR_ERROR = REASON_BASE + 1
    REASON_PROBE_TOO_LOW = REASON_BASE + 2
    REASON_PROBE_TOUCH = REASON_BASE + 5

    def __init__(self, eddy: ProbeEddy):
        self.eddy = eddy
        self._sensor = eddy._sensor
        self._mcu = eddy._mcu
        self._reactor = eddy._printer.get_reactor()

        self._dispatch = mcu.TriggerDispatch(self._mcu)
        self._trigger_time = 0.

    def pull_params(self):
        self._home_trigger_height = self.eddy.params['home_trigger_height']
        self._home_trigger_height_start_offset = self.eddy.params['home_trigger_height_start_offset']
        self._home_start_height = self.eddy._home_start_height
        self._probe_speed = self.eddy.get_probe_params()['probe_speed']
        self._lift_speed = self.eddy.get_probe_params()['lift_speed']

    # these are the "MCU Probe" methods
    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper: MCU_stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self._dispatch.get_steppers()

    def get_position_endstop(self):
        return self._home_trigger_height

    # The Z homing sequence is:
    #   - multi_probe_begin
    #   - probe_prepare
    #   - home_start
    #   - home_wait
    #   - probe_finish
    #   - multi_probe_end

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        if not self._gather._started:
            raise self.eddy._printer.command_error("home_start called without a sampler active")

        self._trigger_time = 0.

        ## XXX FIXME -- +2.0 for debugging so I don't crash the toolhead
        debug_offset = 0.0

        trigger_height = self._home_trigger_height + debug_offset
        start_height = trigger_height + self._home_trigger_height_start_offset

        trigger_freq = self.eddy.height_to_freq(trigger_height)
        start_freq = self.eddy.height_to_freq(start_height)

        trigger_completion = self._dispatch.start(print_time)

        logging.info(f"EDDY home_start: {print_time:.3f} freq: {trigger_freq:.2f} start: {start_freq:.2f}")
        # setup homing -- will start scanning and trigger when we hit
        # trigger_freq
        self._sensor.setup_home2(self._dispatch.get_oid(),
                                 mcu.MCU_trsync.REASON_ENDSTOP_HIT, self.REASON_BASE,
                                 trigger_freq, start_freq, print_time + HOME_TRIGGER_START_TIME_OFFSET)

        return trigger_completion

    def home_wait(self, home_end_time):
        #logging.info(f"EDDY home_wait {home_end_time} cur {curtime} ept {est_print_time} ehe {est_he_time}")
        self._dispatch.wait_end(home_end_time)

        # make sure homing is stopped, and grab the trigger_time from the mcu
        trigger_time = self._sensor.clear_home()
        logging.info(f"EDDY clear_home trigger_time {trigger_time} (mcu: {self._mcu.print_time_to_clock(trigger_time)})")

        # nb: _dispatch.stop() will treat anything >= REASON_COMMS_TIMEOUT as an error,
        # and will only return those results. Fine for us since we only have one trsync,
        # but annoying in general.
        res = self._dispatch.stop()

        # success?
        if res == mcu.MCU_trsync.REASON_ENDSTOP_HIT:
            self._trigger_time = trigger_time
            return trigger_time

        # various errors
        if res == mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            raise self.eddy._printer.command_error("Communication timeout during homing")
        if res == self.REASON_SENSOR_ERROR:
            status = self._sensor.latched_status_str()
            raise self.eddy._printer.command_error(f"Eddy current sensor error; status: {status}")
        if res == self.REASON_PROBE_TOO_LOW:
            raise self.eddy._printer.command_error(f"Probe too low at start of homing, or moved too fast to start trigger position.")

        raise self.eddy._printer.command_error(f"Unknown homing error: {res}")

    def query_endstop(self, print_time):
        curtime = self._reactor.monotonic()
        est_print_time = self._mcu.estimated_print_time(curtime)
        if est_print_time < (print_time-0.050) or est_print_time > (print_time+0.050):
            logging.warning(f"query_endstop: print_time {print_time} is too far from current {est_print_time}")
            return True

        _, height = self.eddy.read_current_freq_and_height()
        return height < self._home_trigger_height

    # these are the ProbeEndstopWrapper methods

    # This is called before the start of a series of probe measurements (1 or more)
    def multi_probe_begin(self):
        logging.info("EDDY multi_probe_begin >>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        self._gather = ProbeEddySampler(self.eddy)
        self._gather.start()
    # The end of a series of measurements
    def multi_probe_end(self):
        logging.info("EDDY multi_probe_end <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
        self._gather.finish()
        self._gather = None

    # Perform a move to pos at speed, stopping when the probe triggers.
    # Return the full toolhead position where the probe triggered....
    # ... or where the toolhead is now?
    def probing_move(self, pos, speed):
        logging.info(f"EDDY performing phoming.probing_move: {pos} {speed}")
        phoming = self.eddy._printer.lookup_object('homing')
        trigger_position = phoming.probing_move(self, pos, speed)

        logging.info(f"EDDY probing_move finished: trigger_position {trigger_position} trigger_time {self._trigger_time}")
        if not self._trigger_time:
            raise self.eddy._printer.command_error("Somehow finished probing_move without a trigger_time?")

        # We know we should have triggered at exactly height=2.0 (home_trigger_height),
        # but because of the averaging that we do on the probe, we probably overshot it
        # a little bit. But we have the raw data, so we can figure out by how much, and
        # adjust the trigger position here.

        start_time = self._trigger_time - LDC1612_SETTLETIME
        end_time = self._trigger_time + LDC1612_SETTLETIME*3
        wait_success = self._gather.wait_for_sample_at_time(end_time, max_wait_time=0.250)
        if not wait_success:
            raise self.eddy._printer.command_error("Waited for probe trigger end time in samples, but didn't find it within timeout!")

        height = self._gather.find_height_at_time(start_time, end_time)
        if height is None:
            raise self.eddy._printer.command_error("Probe trigger time not found in samples! (Shoudn't happen)")

        # We claim that trigger_position[2] is truly height = 2.0,
        # but instead it's the offset above higher; adjust.
        trigger_offset = self._home_trigger_height - height
        trigger_position[2] += trigger_offset
        logging.info(f"EDDY: pulled trigger height {height:.3f} (offset {trigger_offset:.3f}): new z: {trigger_position[2]:.3f}")
        return trigger_position

    def probe_prepare(self, hmove):
        logging.info(f"EDDY probe_prepare ....................................")
        self.probe_to_start_position()

    # This function attempts to raise the toolhead to self.eddy._home_start_height
    # based on the sensor reading, if it's not already above that.
    def probe_to_start_position(self):
        logging.info(f"EDDY probe_to_start_position")
        if self._gather is None:
            raise Exception("probe_prepare called without multi_probe_begin")

        # wait for the toolhead to finish any in-progress moves -- for example,
        # there may already be a homing z-hop    
        toolhead = self.eddy._printer.lookup_object('toolhead')
        toolhead.wait_moves()

        logging.info(f"EDDY gather started: {self._gather._started}")

        # Wait for some new samples to show up. We might get all error samples,
        # which is OK; assume those are too-high out of range
        # TODO: only track appropriate amplitude too low errors
        if not self._gather.wait_for_samples(max_wait_time=1.0, new_only=True, count_errors=True, raise_error=False):
            if self._gather.get_error_count() == 0:
                raise self.eddy._printer.command_error("Waited for new samples and got no samples");

        # grab the last probed height. we don't need a highly accurate value,
        # so just the last one will do
        height = self._gather.get_last_height()
        if height is None:
            # if last_height is None, we got some errors and we can just assume we're "high enough"
            # TODO -- I don't love this. There is a chance this would crash the toolhead if we had some
            # other kind of errors, and we'd still start a probing move. However, the homing hardware
            # implementation will at least only ignore over-amplitude errors.
            logging.info(f"EDDY probe_to_start_position: last_height is None, error count should be >0: {self._gather.get_error_count()}")
            return

        logging.info(f"EDDY probe_to_start_position: current height {height:.3f}")
        z_increase = self.eddy._home_start_height - height
        if z_increase <= 0:
            # we don't need to increase z, we're already above our physical start height
            return

        # We need to move up by z_increase.
        curtime = self._reactor.monotonic()
        th_pos = toolhead.get_position()
        toolhead_kin = toolhead.get_kinematics()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        z_homed = 'z' in kin_status['homed_axes']
        axis_max_z = kin_status['axis_maximum'].z

        logging.info(f"EDDY toolhead z_homed: {z_homed}, cur toolhead z: {th_pos[2]:.3f} z_increase: {z_increase:.3f} max_z {axis_max_z:.3f}")

        # if we're not z-homed, then ask the user to add a z-hop to their homing sequence.
        # TODO -- we could just hop here.
        if not z_homed or th_pos[2] >= axis_max_z:
            raise self.eddy._printer.command_error("Z not homed and probe too low to home; implement a z-hop before homing")

        th_pos[2] += z_increase
        toolhead.manual_move(th_pos, self.eddy.params['probe_speed'])
        toolhead.wait_moves()
        logging.info(f"EDDY probe_to_start_position: moved toolhead up by {z_increase:.3f} to {th_pos[2]:.3f}")

    def probe_finish(self, hmove):
        logging.info(f"EDDY probe_finish ....................................")


@final
class ToolheadMovementHelper:
    def __init__(self, eddy: ProbeEddy, back_to_start: bool = False):
        self._eddy = eddy
        self._speed = eddy.params['probe_speed']
        self._printer = eddy._printer
        self._toolhead = eddy._printer.lookup_object('toolhead')
        self._toolhead_kin = self._toolhead.get_kinematics()
        self._notes = []
        self._startpos = None
        self._return = back_to_start
    
    def __enter__(self):
        self._startpos = self._toolhead.get_position()
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        if self._return:
            pos = self._startpos[:3]
            self.move_to(*pos, speed=5.0)

    def no_return(self):
        self._return = False

    def dwell(self, dur: float):
        self._toolhead.dwell(dur)

    def set_absolute_position(self, x: float, y: float, z: float):
        newpos = np.array(self._toolhead.get_position())
        newpos[:3] = [x, y, z]
        self._toolhead.set_position(newpos, homing_axes=(0, 1, 2))
        curpos = np.array(self._toolhead.get_position())
        # TODO just do the mapping but whatever
        self._return = False
        logging.info(f"set_absolute_position, th now at: {curpos}")

    def get_position(self):
        curpos = np.array(self._toolhead.get_position())
        if self._startpos:
            curpos[:3] -= self._startpos[:3]
        return curpos.tolist()
    
    def get_last_move_time(self):
        return self._toolhead.get_last_move_time()

    def move_to(self, x: float, y: float, z: float):
        newpos = np.array(self._toolhead.get_position())
        oldpos = newpos
        newpos[:3] = [x, y, z]
        logging.info(f"move_to, manual_move to {newpos} from {oldpos}")
        self._toolhead.manual_move(newpos, self._speed)
        self._toolhead.wait_moves()

    def move_by(self, xoffs: float, yoffs: float, zoffs: float):
        newpos = np.array(self._toolhead.get_position()[:3])
        newpos += [xoffs, yoffs, zoffs]
        self.move_to(*newpos)

    def move_to_z(self, z: float):
        newpos = np.array(self._toolhead.get_position()[:3])
        newpos[2] = z
        self.move_to(*newpos)

    def note_time_and_position(self, dwell: float = 0.100):
        self._toolhead.wait_moves()
        time = self._toolhead.get_last_move_time()
        kin_z = self.get_kin_z()
        self.dwell(dwell)
        self._toolhead.wait_moves()

        note = (time, time + dwell, kin_z)
        self._notes.append(note)
        return note

    def notes(self):
        return self._notes

    def get_kin_z(self, at=None):
        return get_toolhead_kin_z(self._toolhead, at=at)


def get_toolhead_kin_z(toolhead, at=None):
    toolhead_kin = toolhead.get_kinematics()
    toolhead.flush_step_generation()
    if at is None:
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in toolhead_kin.get_steppers()}
    else:
        kin_spos = {s.get_name(): s.mcu_to_commanded_position(s.get_past_mcu_position(at))
                    for s in toolhead_kin.get_steppers()}
    return toolhead_kin.calc_position(kin_spos)[2]

def load_config_prefix(config: ConfigWrapper):
    return ProbeEddy(config)