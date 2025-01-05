from __future__ import annotations

import logging, math, bisect
import numpy as np
import numpy.polynomial as npp
import traceback

import mcu
import pins

from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Dict, List, Optional, TypedDict, final

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

# In this file, a couple of conventions are used (for sanity).
# Variables are named according to:
# - "height" is always a physical height as detected by the probe in mm
# - "z" is always a z axis position (which may or may not match height)
# - "freq" is always a frequency value (float)
# - "freqval" is always an encoded frequency value, as communicated to/from the sensor (int)

# How many seconds from the start of probing must we be below the
# trigger start frequency (above the start homing height)
HOME_TRIGGER_START_TIME_OFFSET = 0.200

def log_traceback(limit=None):
    for line in traceback.format_stack(limit=limit):
        logging.info(line.strip())

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
            # TODO what is speed vs probe_speed
            'speed': 5.0,
            'lift_speed': 5.0,
            'probe_speed': 5.0,
            'backlash_comp': 0.5,  #TODO

            # what height to trigger homing at
            'home_trigger_height': 2.0,
            # What height above home_trigger_height to allow homing to start
            'home_trigger_height_start_offset': 1.5,
            'trigger_freq_slop': 0.006,

            'calibration_z_max': 5.0,
            'calibration_step': 0.040,

            'reg_drive_current': 0,
            'tap_drive_current': 0,
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

        # drive current to frequency map
        self._dc_to_fmap: Dict[int, ProbeEddyFrequencyMap] = {}
        calibrated_drive_currents = config.getintlist('calibrated_drive_currents', [])

        for dc in calibrated_drive_currents:
            self._dc_to_fmap[dc] = ProbeEddyFrequencyMap(self)
            self._dc_to_fmap[dc].load_from_config(config, dc)

        # Our virtual endstop and all-around "do things" class. The majority
        # of probing functionality happens here.
        # TODO: rework this; pull methods into this class, and have EndstopWrapper
        # just forward
        self._endstop_wrapper = ProbeEddyEndstopWrapper(self)

        # This class emulates "PrinterProbe". We use some existing helpers to implement
        # functionality like start_session
        self._printer.add_object('probe', self)
        self._probe_session = probe.ProbeSessionHelper(config, self._endstop_wrapper)
        # define PROBE, PROBE_ACCURACY, etc.
        self._cmd_helper = probe.ProbeCommandHelper(config, self, self._endstop_wrapper.query_endstop)

        # update some local param copies. We need to do this after creating a ProbeSessionHelper,
        # because it pulls some things from the probe_params
        self._endstop_wrapper.pull_params()

        # define our own commands
        gcode = self._printer.lookup_object('gcode')
        self.define_commands(gcode)

    def _validate_params(self):
        home_trigger_height = self.params['home_trigger_height']
        home_trigger_height_start_offset = self.params['home_trigger_height_start_offset']
        calibration_z_max = self.params['calibration_z_max']

        if self.params['tap_drive_current'] == 0:
            self.params['tap_drive_current'] = self.params['reg_drive_current']
        
        self.params['tap_drive_current'] = int(self.params['tap_drive_current'])
        self.params['reg_drive_current'] = int(self.params['reg_drive_current'])

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
        gcode.register_command("PROBE_EDDY_CLEAR_CALIBRATION", self.cmd_CLEAR_CALIBRATION)
        gcode.register_command("PROBE_EDDY_PROBE_STATIC", self.cmd_PROBE_STATIC)
        gcode.register_command("PROBE_EDDY_TAP", self.cmd_TAP)

        # some handy aliases while I'm debugging things to save my fingers
        gcode.register_command("PEPS", self.cmd_PROBE_STATIC)
        gcode.register_command("PEQ", self.cmd_QUERY)
        gcode.register_command("PETAP", self.cmd_TAP)

    def current_drive_current(self) -> int:
        return self._sensor.get_drive_current()

    def map_for_drive_current(self, dc: int = None) -> ProbeEddyFrequencyMap:
        if dc is None:
            dc = self.current_drive_current()
        if dc not in self._dc_to_fmap:
            raise self._printer.command_error(f"Drive current {dc} not calibrated")
        return self._dc_to_fmap[dc]

    # helpers to forward to the map
    def height_to_freq(self, height: float, drive_current: int = None, temp: float = None) -> float:
        if drive_current is None:
            drive_current = self.current_drive_current()
        return self.map_for_drive_current(drive_current).height_to_freq(height, temp)

    def freq_to_height(self, freq: float, drive_current: int = None, temp: float = None) -> float:
        if drive_current is None:
            drive_current = self.current_drive_current()
        return self.map_for_drive_current(drive_current).freq_to_height(freq, temp)

    def calibrated(self, drive_current: int = None) -> bool:
        if drive_current is None:
            drive_current = self.current_drive_current()
        return drive_current in self._dc_to_fmap and self._dc_to_fmap[drive_current].calibrated()

    def cmd_QUERY(self, gcmd: GCodeCommand):
        status, freqval, freq = self._sensor.read_one_value()
        height = self.freq_to_height(freq) if self.calibrated() else -math.inf

        err = ""
        if freqval > 0x0fffffff:
            height = -math.inf
            freq = 0.0
            err = f"ERROR: {bin(freqval >> 28)} "
        if not self.calibrated():
            err += "(Not calibrated) "

        gcmd.respond_info(f"Last coil value: {freq:.2f} ({height:.3f}mm) raw: {hex(freqval)} {err}status: {hex(status)} {self._sensor.status_to_str(status)}")

    # TODO: use this in implementing cmd_QUERY in above
    def read_current_freq_and_height(self):
        self._sensor._start_measurements()
        status, freqval, freq = self._sensor.read_one_value()
        self._sensor._finish_measurements()

        # if in error, return -inf height to make endstop checks easier
        if freqval > 0x0fffffff:
            return None, -math.inf

        height = self.freq_to_height(freq)
        return freq, height

    def cmd_CLEAR_CALIBRATION(self, gcmd: GCodeCommand):
        self._dc_to_fmap = {}
        self.save_config(gcmd)
        gcmd.respond_info(f"Cleared calibration for all drive currents")

    def cmd_PROBE_STATIC(self, gcmd: GCodeCommand):
        if not self.calibrated():
            gcmd.respond_raw("!! Probe not calibrated\n")
            return

        duration = gcmd.get_float('DURATION', 0.100, above=0.0)
        mean_or_median = gcmd.get('METHOD', 'mean').lower()

        reactor = self._printer.get_reactor()
        now = self._mcu.estimated_print_time(reactor.monotonic())

        with ProbeEddySampler(self) as sampler:
            sampler.wait_for_sample_at_time(now + duration)
            sampler.finish()
            cend = self._mcu.estimated_print_time(reactor.monotonic())

            samples = sampler.get_samples()
            if len(samples) == 0:
                gcmd.respond_raw("!! No samples collected\n")
                return

            # skip LDC1612_SETTLETIME samples at start and end by looking
            # at the time values
            stime = samples[0][0]
            etime = samples[-1][0]
            orig_samplecount = len(samples)
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

            gcmd.respond_info(f"Height: {height:.3f} (mean: {mean:.3f}, median: {median:.3f}), {orig_samplecount} samples {etime-stime:.3f} sec")
            #gcmd.respond_info(f"Collection started at {now:.3f}, sample time range {stime:.3f} - {etime:.3f}, finished at {cend:.3f}")

            self._cmd_helper.last_z_result = height

    def cmd_CALIBRATE(self, gcmd: GCodeCommand):
        # z-hop so that manual probe helper doesn't complain if we're already
        # at the right place
        toolhead: ToolHead = self._printer.lookup_object('toolhead')
        curpos = toolhead.get_position()
        curpos[2] = curpos[2] + 5
        toolhead.manual_move(curpos, self.params['probe_speed'])

        manual_probe.ManualProbeHelper(self._printer, gcmd,
                                       lambda kin_pos: self.cmd_CALIBRATE_next(gcmd, kin_pos))

    def cmd_CALIBRATE_next(self, gcmd: GCodeCommand, kin_pos: List[float]):
        if kin_pos is None:
            return

        old_drive_current = self.current_drive_current()    
        drive_current: int = gcmd.get_int('DRIVE_CURRENT', old_drive_current, minval=0, maxval=31)
        self._sensor.set_drive_current(drive_current)

        Z_TARGET = 0.010

        with ToolheadMovementHelper(self) as th:
            # We just did a ManualProbeHelper, so we're going to zero the z-axis
            # to make the following code easier, so it can assume z=0 is actually real zero.
            # The Eddy sensor calibration is done to nozzle height (not sensor or trigger height).
            kin_pos[2] = 0.0
            th.set_absolute_position(*kin_pos)
            th.dwell(0.100)

            # move to the start of calibration
            cal_z_max = self.params['calibration_z_max']
            #th.move_by(-self.offset['x'], -self.offset['y'], cal_z_max)
            th.move_to_z(cal_z_max)

            mapping = None
            toolhead_positions = []
            first_sample_time = None
            last_sample_time = None

            with ProbeEddySampler(self, calculate_heights=False) as sampler:
                first_sample_time = th.get_last_move_time()
                th.move_to_z(Z_TARGET)
                last_sample_time = th.get_last_move_time()
                sampler.finish()

            # back to a safe spot
            th.move_to_z(cal_z_max)
            self._sensor.set_drive_current(old_drive_current)

        # the samples are a list of [print_time, freq] pairs.
        # in toolhead_positions, we have a list of [print_time, kin_z] pairs.
        samples = sampler.get_samples()
        if len(samples) == 0:
            gcmd.respond_raw("!! No samples collected\n")
            return

        freqs = []
        heights = []

        data_file = open("/tmp/eddy-samples.csv", "w")
        data_file.write(f"time,frequency,z,used\n")

        for s_t, s_freq, _ in samples:
            s_z = th.get_kin_z(at=s_t)
            used = 0
            if s_t > (first_sample_time + 0.050) and s_t < last_sample_time and s_z > Z_TARGET:
                freqs.append(s_freq)
                heights.append(s_z)
                used = 1
            data_file.write(f"{s_t},{s_freq},{s_z},{used}\n")
            # we write everything for debugging, but only use the samples in our
            # time range
        
        data_file.close()
        gcmd.respond_info(f"Wrote {len(samples)} samples to /tmp/eddy-samples.csv")

        # and build a map
        mapping = ProbeEddyFrequencyMap(self)
        r2_fth, r2_htf = mapping.calibrate_from_values(drive_current, freqs, heights, gcmd)

        if r2_fth is None or r2_htf is None:
            gcmd.respond_raw("!! Calibration failed\n")
            return

        self._dc_to_fmap[drive_current] = mapping
        self.save_config(gcmd)

        gcmd.respond_info(f"Calibration complete, R^2: freq-to-height={r2_fth:.3f}, height-to-freq={r2_htf:.3f}")

    def save_config(self, gcmd: GCodeCommand):
        for _, fmap in self._dc_to_fmap.items():
            fmap.save_calibration(gcmd)

        configfile = self._printer.lookup_object('configfile')
        configfile.set(self._full_name, f"calibrated_drive_currents", str.join(', ', [str(dc) for dc in self._dc_to_fmap.keys()]))

        if gcmd:
            gcmd.respond_info("Calibration saved. Issue a SAVE_CONFIG to write the values to your config file and restart Klipper.")

    #
    # PrinterProbe interface
    #

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
            #z_offset = self.get_offsets()[2]
            session = ProbeEddyScanningProbe(self, gcmd)
            session.start_session()
            return session

        return self._probe_session.start_probe_session(gcmd)

    def get_status(self, eventtime):
        status = self._cmd_helper.get_status(eventtime)
        status.update({ 'name': self._full_name, 'home_trigger_height': self.params['home_trigger_height'] })
        return status

    # 
    # Tap probe
    #
    def cmd_TAP(self, gcmd: GCodeCommand):
        drive_current = self._sensor.get_drive_current()
        try:
            self.tapping_move_homing(gcmd)
        finally:
            self._sensor.set_drive_current(drive_current)

    def tapping_move_homing(self, gcmd: GCodeCommand):
        logging.info("\n\nTapping move: homing")
        tap_drive_current = gcmd.get_int(name='DRIVE_CURRENT', default=self.params['tap_drive_current'], minval=0, maxval=31)
        self._sensor.set_drive_current(tap_drive_current)

        TAP_MOVE_SPEED=3.0
        TAP_RETRACT_SPEED=5.0
        TAP_THRESHOLD=550
        TARGET_Z=-0.250

        tap_move_speed = gcmd.get_float('SPEED', TAP_MOVE_SPEED, above=0.0)
        tap_threshold = gcmd.get_int('TTAP', TAP_THRESHOLD, minval=0)
        do_retract = gcmd.get_int('RETRACT', 1)
        no_adjust = gcmd.get_int('NO_ADJUST', 0)
        tap_start_z = gcmd.get_float('START_Z', 3.5, above=2.0)
        target_z = gcmd.get_float('TARGET_Z', TARGET_Z)

        reactor = self._printer.get_reactor()
        th = self._printer.lookup_object('toolhead')
        kin = th.get_kinematics()

        home_trigger_height = self.params['home_trigger_height']
        home_trigger_height_start_offset = self.params['home_trigger_height_start_offset']

        curtime_r = reactor.monotonic()
        kin_status = kin.get_status(curtime_r)
        z_homed = 'z' in kin_status['homed_axes']
        if not z_homed:
            raise self._printer.command_error("Z axis must be homed before tapping")

        th.manual_move([None, None, tap_start_z], TAP_RETRACT_SPEED)
        th.dwell(0.100)
        th.wait_moves()

        initial_position = th.get_position()
        initial_z = initial_position[2]
        target_position = initial_position[:]
        target_position[2] = target_z

        phoming = self._printer.lookup_object('homing')
        try:
            self._endstop_wrapper.save_samples_path = "/tmp/tap-samples.csv"
            self._endstop_wrapper.tap_threshold = tap_threshold
            # this does a HomingMove.homing_move with a probe_pos = True
            probe_position = phoming.probing_move(self._endstop_wrapper, target_position, tap_move_speed)
        except self._printer.command_error as e:
            raise
        finally:
            self._endstop_wrapper.tap_threshold = 0

        th_pos = th.get_position()

        probe_trigger_z = probe_position[2]
        now_z = th_pos[2]

        # we're at now_z, but probe_z is actual zero. We expect now_z
        # to be below or equal to probe_z, i.e. it's the overshoot
        if now_z > probe_trigger_z:
            gcmd.respond_raw(f"!! warning: now_z {now_z:.3f} is above probe_z {probe_trigger_z:.3f}\n")

        # How much we overshot the tap start time (which is what probe_trigger_z is based on;
        # the trigger fires later)
        overshoot = probe_trigger_z - now_z

        logging.info(f"Probe triggered at: {probe_trigger_z:.4f}, now z: {now_z:.4f}, overshoot: {overshoot:.4f}")

        # to get back to true zero, we need to move the toolhead up by 'overshoot'
        # which means

        real_z = - overshoot

        th_adjust = now_z - real_z

        gcmd.respond_info(f"Adjusting z by {th_adjust:.4f}")
        logging.info(f"Changing current toolhead z from {now_z:.3f} to {real_z:.3f} {'(no adjust)' if no_adjust else ''}")

        if not no_adjust:
            th_pos[2] = real_z
            th.set_position(th_pos)

        if initial_z < th_pos[2]:
            raise self._printer.command_error(f"Initial position {initial_z:.3f} is below current, not moving down!")

        if do_retract:
            th_pos[2] = initial_z
            th.manual_move(th_pos, TAP_RETRACT_SPEED)
            th.wait_moves()
            th.flush_step_generation()

        #th_pos[2] = th_pos[2] + overshoot + initial_z
        #th.manual_move(th_pos, TAP_RETRACT_SPEED)
        #th.wait_moves()
        #logging.info(f"After move1: {th.get_position()}")
        ## now this is true initial_z
        #th_pos[2] = initial_z
        #th.set_position(th_pos)

        #th.set_position(initial_position)
        #th.wait_moves()
        #th.flush_step_generation()
        logging.info("Tapping move: homing end\n\n")


#
# Bed scan specific probe session interface
#
@final
class ProbeEddyScanningProbe:
    def __init__(self, eddy: ProbeEddy, gcmd: GCodeCommand):
        self.eddy = eddy
        self._toolhead = eddy._printer.lookup_object('toolhead')
        self._toolhead_kin = self._toolhead.get_kinematics()

        self._home_trigger_height = eddy.params['home_trigger_height']

        # how much to dwell at each sample position in addition to sample_time
        self._sample_time_delay = 0.050
        self._sample_time = gcmd.get_float("SAMPLE_TIME", 0.100, above=0.0)
        self._is_rapid = gcmd.get("METHOD", "scan") == 'rapid_scan'

        self._gather = ProbeEddySampler(self.eddy)

        self._notes = []

    def start_session(self):
        self._gather.start()

    def _lookup_toolhead_pos(self, time):
        kin_spos = {s.get_name(): s.mcu_to_commanded_position(
                                      s.get_past_mcu_position(time))
                    for s in self._toolhead_kin.get_steppers()}
        return self._toolhead_kin.calc_position(kin_spos)

    def _rapid_lookahead_cb(self, time):
        # is this is a fudge factor?
        start_time = time - self._sample_time / 2
        toolhead_pos = self._lookup_toolhead_pos(start_time)
        self._notes.append((start_time, toolhead_pos))

    def run_probe(self, gcmd):
        if self._is_rapid:
            # this callback is attached to the last move in the queue, so that
            # we can grab the toolhead position when the toolhead actually hits it
            self._toolhead.register_lookahead_callback(self._rapid_lookahead_cb)
            return

        time = self._toolhead.get_last_move_time()
        self._toolhead.dwell(self._sample_time + self._sample_time_delay)

        start_time = time + self._sample_time_delay
        toolhead_pos = self._lookup_toolhead_pos(start_time)
        self._notes.append((start_time, toolhead_pos))

    def pull_probed_results(self):
        if self._is_rapid:
            # Flush lookahead (so all lookahead callbacks are invoked)
            self._toolhead.get_last_move_time()

        # make sure we get the sample for the final move
        self._gather.wait_for_sample_at_time(self._notes[-1][0] + self._sample_time)
        self._gather.finish()

        # the results of this pull_probed are an array of full toolhead positions,
        # one for each sample time range
        results = []
        for start_time, toolhead_pos in self._notes:
            end_time = start_time + self._sample_time
            height = self._gather.find_height_at_time(start_time, end_time)

            # the delta between where the toolhead thinks it should be (since it
            # should be homed), and the actual physical offset (height)
            z_deviation = toolhead_pos[2] - height
            toolhead_pos[2] = self._home_trigger_height + z_deviation
            results.append(toolhead_pos)

        # Allow axis_twist_compensation to update results
        for epos in results:
            self.eddy._printer.send_event("probe:update_results", epos)
        return results

    def end_probe_session(self):
        self._gather.finish()
        self._gather = None

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

        self.tap_threshold = 0
        self.save_samples_path = None

        self._multi_probe_in_progress = False

        self._dispatch = mcu.TriggerDispatch(self._mcu)
        self._trigger_time = 0.
        self._tap_start_time = 0.0

        self._gather: Optional[ProbeEddySampler] = None

    def pull_params(self):
        self._home_trigger_height = self.eddy.params['home_trigger_height']
        self._home_trigger_height_start_offset = self.eddy.params['home_trigger_height_start_offset']
        self._home_start_height = self.eddy._home_start_height
        self._probe_speed = self.eddy.get_probe_params()['probe_speed']
        self._lift_speed = self.eddy.get_probe_params()['lift_speed']

        self._dive_height = 5.0
        self._tap_speed = 10.0
        self._tap_threshold = 1500
        self._tap_z_target = -0.250
        self._tap_start_height = 3.2
        # difference between toolhead z and sensor height at which to force
        # a probing move
        self._toolhead_delta_threshold = 0.500

    # these are the "MCU Probe" methods
    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper: MCU_stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self._dispatch.get_steppers()

    def get_position_endstop(self):
        logging.info(f"EDDY get_position_endstop -> {0.0 if self.tap_threshold > 0.0 else self._home_trigger_height}")
        if self.tap_threshold > 0:
            return 0.0
        else:
            return self._home_trigger_height

    # The Z homing sequence is:
    #   - multi_probe_begin
    #   - probe_prepare
    #   - home_start
    #   - home_wait
    #   - probe_finish
    #   - multi_probe_end
    #
    # Note no probing_move during homing! `HomingMove.homing_move` drives the above sequence.
    # probing_move is only used during actual probe operations. 

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        if not self._gather._started:
            raise self.eddy._printer.command_error("home_start called without a sampler active")

        self._trigger_time = 0.
        self._tap_time = 0.

        trigger_height = self._home_trigger_height
        start_height = trigger_height + self._home_trigger_height_start_offset

        trigger_freq = self.eddy.height_to_freq(trigger_height)
        start_freq = self.eddy.height_to_freq(start_height)

        #start_time = print_time + HOME_TRIGGER_START_TIME_OFFSET
        #start_time = 0.025 #print_time + 0.025
        start_time = 0 if self.tap_threshold > 0 else print_time + HOME_TRIGGER_START_TIME_OFFSET

        trigger_completion = self._dispatch.start(print_time)

        logging.info(f"EDDY home_start: {print_time:.3f} freq: {trigger_freq:.2f} start: {start_freq:.2f}")
        # setup homing -- will start scanning and trigger when we hit
        # trigger_freq
        self._sensor.setup_home2(self._dispatch.get_oid(),
                                 mcu.MCU_trsync.REASON_ENDSTOP_HIT, self.REASON_BASE,
                                 trigger_freq, start_freq, start_time,
                                 tap_threshold=self.tap_threshold)

        return trigger_completion

    def home_wait(self, home_end_time):
        logging.info(f"EDDY home_wait until {home_end_time:.3f}")
        #logging.info(f"EDDY home_wait {home_end_time} cur {curtime} ept {est_print_time} ehe {est_he_time}")
        self._dispatch.wait_end(home_end_time)

        # make sure homing is stopped, and grab the trigger_time from the mcu
        active, trigger_time, tap_start_time, tap_amount = self._sensor.finish_home2()
        logging.info(f"EDDY trigger_time {trigger_time} (mcu: {self._mcu.print_time_to_clock(trigger_time)}) tap_time: {tap_start_time}")

        # nb: _dispatch.stop() will treat anything >= REASON_COMMS_TIMEOUT as an error,
        # and will only return those results. Fine for us since we only have one trsync,
        # but annoying in general.
        res = self._dispatch.stop()
        is_tap = self.tap_threshold > 0

        if is_tap:
            logging.info(f"EDDY mcu_probe: resetting tap_threshold")
            self._tap_threshold = 0

        self._gather.wait_for_sample_at_time(trigger_time)

        # success?
        if res == mcu.MCU_trsync.REASON_ENDSTOP_HIT:
            self._trigger_time = trigger_time
            self._tap_start_time = tap_start_time
            # TODO: change C side to have "tap end time", so that trigger time is always
            # the correct thing
            return tap_start_time if is_tap else trigger_time

        # various errors
        if res == mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            raise self.eddy._printer.command_error("Communication timeout during homing")
        if res == self.REASON_SENSOR_ERROR:
            status = self._sensor.latched_status_str()
            raise self.eddy._printer.command_error(f"Sensor error; status: {status}")
        if res == self.REASON_PROBE_TOO_LOW:
            raise self.eddy._printer.command_error(f"Probe too low at start of homing, or moved too fast to start trigger position.")

        raise self.eddy._printer.command_error(f"Unknown homing error: {res}")

    def query_endstop(self, print_time):
        if self.use_tap:
            # XXX unclear how to do this with tap? It's basically always going to be false
            # unless the toolhead is somehow at 0.0, and we will never know if that's true 0.0
            # unless we tap.  We could remember the tap freq?
            return False

        curtime = self._reactor.monotonic()
        est_print_time = self._mcu.estimated_print_time(curtime)
        if est_print_time < (print_time-0.050) or est_print_time > (print_time+0.050):
            logging.warning(f"query_endstop: print_time {print_time} is too far from current {est_print_time}")
            return True

        _, height = self.eddy.read_current_freq_and_height()
        return height < self._home_trigger_height

    def _setup_gather(self):
        self._gather = ProbeEddySampler(self.eddy)
        self._gather.start()

    def _finish_gather(self):
        self._gather.finish()

        if self.save_samples_path is not None:
            with open(self.save_samples_path, "w") as data_file:
                samples = self._gather.get_samples()
                raw_samples = self._gather.get_raw_samples()
                data_file.write(f"time,frequency,z,kin_z,kin_v,raw_f,tap_start_time,trigger_time\n")
                for i in range(len(samples)):
                    tap_start_time = self._tap_start_time if i == 0 else ''
                    trigger_time = self._trigger_time if i == 0 else ''
                    s_t, s_freq, s_z = samples[i]
                    _, raw_f , _= raw_samples[i]
                    past_k_z, past_v = get_past_toolhead_goal_z(self.eddy._printer, s_t)
                    if past_k_z is None or past_v is None:
                        past_k_z = ""
                        past_v = ""
                    data_file.write(f"{s_t},{s_freq},{s_z},{past_k_z},{past_v},{raw_f},{tap_start_time},{trigger_time}\n")
            logging.info(f"Wrote {len(samples)} samples to {self.save_samples_path}")
            self.save_samples_path = None

        self._gather = None


    # these are the ProbeEndstopWrapper methods

    # This is called before the start of a series of probe measurements (1 or more)
    def multi_probe_begin(self):
        logging.info("EDDY multi_probe_begin >>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        self._multi_probe_in_progress = True
        self._setup_gather()

    # The end of a series of measurements
    def multi_probe_end(self):
        if not self._multi_probe_in_progress:
            raise self.eddy._printer.command_error("multi_probe_end called without a multi_probe_begin")
        self._finish_gather()
        self._multi_probe_in_progress = False
        logging.info("EDDY multi_probe_end <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")

    def sample_now(self, start_time=None):
        if start_time is None:
            start_time = self.eddy._mcu.estimated_print_time(self._reactor.monotonic())
        end_time = start_time + LDC1612_SETTLETIME*4

        self._gather.wait_for_sample_at_time(end_time, max_wait_time=0.250)
        height = self._gather.find_height_at_time(start_time, end_time)

        return height


    # Perform a move to pos at speed, stopping when the probe triggers.
    # Report the actual toolhead position _now_ (not when the probe triggered,
    # which might be earlier).
    def probing_move(self, pos, speed):
        logging.info(f"EDDY probing_move start: {pos} {speed}")
        toolhead = self.eddy._printer.lookup_object('toolhead')
        phoming = self.eddy._printer.lookup_object('homing')

        curtime = self.eddy._printer.get_reactor().monotonic()

        # this may seem like an odd requirement, but the idea is that
        # you do a generic home first with just a regular 2.0 height trigger
        # to get into the ballpark, and then you can fine-tune with this
        # TODO -- revisit this
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.eddy._printer.command_error("Z axis must be homed before probing (probing_move)")

        toolhead_z = toolhead.get_position()[2]
        trigger_position = None

        height = self.sample_now()

        # are we too high? if so, just do a simple probing_move to get us into the ballpark;
        # same if the toolhead z and sensor height disagree by more than 0.5mm
        if height > self._dive_height or abs(toolhead_z - height) > self._toolhead_delta_threshold:
            # this probing move just gets us into the ballpark of probing coordinates
            trigger_position = phoming.probing_move(self, pos, speed)
            logging.info(f"EDDY did homing.probing_move: trigger_position {trigger_position} trigger_time {self._trigger_time}")
        elif toolhead_z < self._home_start_height:
            toolhead.manual_move([None, None, self._home_start_height + 1.0], speed)
            toolhead.manual_move([None, None, self._home_start_height], speed)
            toolhead.wait_moves()

        # We know we should have triggered at exactly height=2.0 (home_trigger_height),
        # but because of the averaging that we do on the probe, we probably overshot it
        # a little bit. But we have the raw data, so we can figure out by how much, and
        # adjust the trigger position here.

        toolhead.wait_moves() # should be a no-op
        #start_time = self._trigger_time - LDC1612_SETTLETIME
        # we're actually going to recheck the height now that toolhead is static
        start_time = self.eddy._mcu.estimated_print_time(self._reactor.monotonic())
        end_time = start_time + LDC1612_SETTLETIME*4

        self._gather.wait_for_sample_at_time(end_time, max_wait_time=0.250)
        height = self._gather.find_height_at_time(start_time, end_time)

        toolhead_pos = toolhead.get_position()
        trigger_z = trigger_position[2] if trigger_position is not None else 0.0
        toolhead_z = toolhead_pos[2]

        # this is the offset from where the toolhead is now relative to... 
        # well it can't be height, we don't care about the trigger time, do we?
        z_deviation = toolhead_z - height

        logging.info(f"EDDY: toolhead_pos {toolhead_pos}")
        logging.info(f"EDDY: toolhead trigger z {trigger_z:.3f}")
        logging.info(f"EDDY: toolhead current z {toolhead_z:.3f}")
        logging.info(f"EDDY:      actual height {height:.3f} (deviation from current {z_deviation:.3f})")

        # the z coordinate of this is supposed to be relative to the trigger position
        # (our z_offset)
        # i.e. at where the trigger would trigger at this point, if a trigger
        # was triggered.  We may not have used the trigger position to probe though!

        toolhead_pos[2] = self._home_trigger_height + z_deviation
        logging.info(f"EDDY:           reported {toolhead_pos[2]:.3f}")

        return toolhead_pos

    def probe_prepare(self, hmove):
        logging.info(f"EDDY probe_prepare ....................................")
        if not self._multi_probe_in_progress:
            self._setup_gather();
        self.probe_to_start_position()

    # This function attempts to raise the toolhead to self.eddy._home_start_height
    # based on the sensor reading, if it's not already above that.
    def probe_to_start_position(self):
        logging.info(f"EDDY probe_to_start_position (tt: {self.tap_threshold})")
        if self._gather is None:
            raise Exception("probe_to_start_position: no gather?")

        # wait for the toolhead to finish any in-progress moves -- for example,
        # there may already be a homing z-hop
        toolhead = self.eddy._printer.lookup_object('toolhead')
        toolhead.wait_moves()

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

        start_height = self._home_start_height if self.tap_threshold == 0 else self._tap_start_height

        logging.info(f"EDDY probe_to_start_position: current height {height:.3f}, start_height: {start_height:.3f}")
        z_increase = start_height - height
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

        # We may be too low or too high.
        # if we're not z-homed, then ask the user to add a z-hop to their homing sequence.
        # TODO -- we could just hop here.
        if not z_homed or th_pos[2] >= axis_max_z:
            raise self.eddy._printer.command_error("Z not homed and probe too low to home; implement a z-hop before homing")

        th_pos[2] += z_increase
        toolhead.manual_move(th_pos, self.eddy.params['probe_speed'])
        toolhead.wait_moves()
        logging.info(f"EDDY probe_to_start_position: moved toolhead up by {z_increase:.3f} to {th_pos[2]:.3f}")

    def probe_finish(self, hmove):
        if not self._multi_probe_in_progress:
            self._finish_gather();
        logging.info(f"EDDY probe_finish ....................................")

# Helper to gather samples and convert them to probe positions
@final
class ProbeEddySampler:
    def __init__(self, eddy: ProbeEddy, calculate_heights: bool = True, trace: bool = False):
        self.eddy = eddy
        self._sensor = eddy._sensor
        self._reactor = self.eddy._printer.get_reactor()
        self._mcu = self._sensor.get_mcu()
        self._stopped = False
        self._started = False
        self._errors = 0
        self._trace = trace
        self._fmap = eddy.map_for_drive_current() if calculate_heights else None

        # this will hold the raw samples coming in from the sensor,
        # with an empty 3rd (height) value
        self._raw_samples = []
        # this will hold samples with a height filled in (if we're doing that)
        self._samples = []

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
            raise self.eddy._printer.command_error("ProbeEddySampler.finish() called without start()")
        self.eddy._sensor.clear_home()
        self._stopped = True

    def _update_samples(self):
        if len(self._samples) == len(self._raw_samples):
            return

        start_idx = len(self._samples)
        conv_ratio = self._sensor.conversion_ratio()
        if self._fmap is not None:
            new_samples = [(
                t,
                round(conv_ratio * f, ndigits=3),
                self._fmap.freq_to_height(round(conv_ratio * f, ndigits=3)))
                for t, f, _ in self._raw_samples[start_idx:]]
            self._samples.extend(new_samples)
        else:
            self._samples.extend(self._raw_samples[start_idx:])

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
    def wait_for_sample_at_time(self, sample_print_time, max_wait_time=0.250, raise_error=True) -> bool:
        def report_no_samples():
            if raise_error:
                log_traceback(3)
                raise self.eddy._printer.command_error(f"No samples received for time {sample_print_time:.3f} (waited for {max_wait_time:.3f}")
            return False

        if self._stopped:
            # if we're not getting any more samples, we can check directly
            if len(self._raw_samples) == 0:
                return report_no_samples()
            return self._raw_samples[-1][0] >= sample_print_time

        wait_start_time = self._mcu.estimated_print_time(self._reactor.monotonic())

        # if sample_print_time is in the future, make sure to wait max_wait_time
        # past the expected time
        if sample_print_time > wait_start_time:
            max_wait_time = max_wait_time + (sample_print_time - wait_start_time)

        # this is just a sanity check, there's no reason to ever wait this long
        if max_wait_time > 5.0:
            raise self.eddy._printer.command_error(f"ProbeEddyFrequencySampler: max_wait_time {max_wait_time:.3f} is too far into the future")


        logging.info(f"ProbeEddyFrequencySampler: waiting for sample at {sample_print_time:.3f} (now: {wait_start_time:.3f}, max_wait_time: {max_wait_time:.3f})")
        while len(self._raw_samples) == 0 or self._raw_samples[-1][0] < sample_print_time:
            now = self._mcu.estimated_print_time(self._reactor.monotonic())
            if now - wait_start_time > max_wait_time:
                return report_no_samples()
            self._reactor.pause(now + 0.010)

        return True

    # Wait for some samples to be collected, even if errors
    # TODO: there's a minimum wait time -- we need to fill up the buffer before data is sent, and that
    # depends on the data rate
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
                    log_traceback(5)
                    raise self.eddy._printer.command_error(f"possible probe_eddy sensor outage; no samples for {max_wait_time:.2f}s")
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

        if self._trace:
            logging.info(f"find_height_at_time: searching between {start_time:.3f} and {end_time:.3f}")
        if len(self._samples) == 0:
            raise self.eddy._printer.command_error("No samples found for time range")

        if self._trace:
            logging.info(f"find_height_at_time: {len(self._samples)} samples, time range {self._samples[0][0]:.3f} to {self._samples[-1][0]:.3f}")

        # find the first sample that is >= start_time
        start_idx = bisect.bisect_left([t for t, _, _ in self._samples], start_time)
        if start_idx >= len(self._samples):
            raise self.eddy._printer.command_error("Nothing after start_time?")

        # find the last sample that is <= end_time
        end_idx = bisect.bisect_right([t for t, _, _ in self._samples], end_time)
        if end_idx == 0:
            raise self.eddy._printer.command_error("found something at start_time, but not before end_time?")

        # average the heights of the samples in the range
        heights = [h for _, _, h in self._samples[start_idx:end_idx]]
        hmin, hmax = np.min(heights), np.max(heights)
        mean = np.mean(heights)
        median = np.median(heights)
        if self._trace:
            logging.info(f"find_height_at_time: {len(heights)} samples, mean: {mean:.3f} median: {median:.3f} (range {hmin:.3f}-{hmax:.3f})")

        return float(median)

@final
class ProbeEddyFrequencyMap:
    def __init__(self, eddy: ProbeEddy):
        self._eddy = eddy
        self._sensor = eddy._sensor
        self._temp_sensor = eddy._temp_sensor

        self.drive_current = 0
        self._freq_to_height: Optional[npp.Polynomial] = None
        self._height_to_freq: Optional[npp.Polynomial] = None

    def load_from_config(self, config: ConfigWrapper, drive_current: int):
        ftoh = config.getfloatlist(f"freq_to_height_p_{drive_current}", default=None)
        htof = config.getfloatlist(f"height_to_freq_p_{drive_current}", default=None)

        if ftoh and htof:
            self.drive_current = drive_current
            self._freq_to_height = npp.Polynomial(ftoh)
            self._height_to_freq = npp.Polynomial(htof)
            logging.info(f"Loaded polynomials for drive current {drive_current}:" + \
                         f"   freq-to-height: {self._coefs(self._freq_to_height)}\n" + \
                         f"   height-to-freq: {self._coefs(self._height_to_freq)}")
        else:
            self.drive_current = 0
            self._freq_to_height = None
            self._height_to_freq = None

    # helper. numpy.Polynomial .coef returns coefficients with domain/range mapping baked in.
    # until we store those, those are no good for round-tripping. convert() gives us
    # the unscaled values.
    def _coefs(self, p):
        return p.convert().coef.tolist()

    def calibrate_from_values(self, drive_current: int, raw_freqs: List[float], raw_heights: List[float], gcmd: Optional[GCodeCommand] = None):
        if len(raw_freqs) != len(raw_heights):
            raise ValueError("freqs and heights must be the same length")

        def rolling_mean(data, window, center=True):
            half_window = (window - 1) // 2 if center else 0
            result = np.empty(len(data), dtype=float)

            for i in range(len(data)):
                # Define the start and end of the window
                start = max(0, i - half_window)
                end = min(len(data), i + half_window + 1)
                result[i] = np.mean(data[start:end])  # Mean over the valid window
   
            return result

        # smooth out the data; a simple rolling average does the job,
        # but centered to give more accurate data (since we have the full
        # data set. The edges just use a smaller window.
        freqs = rolling_mean(raw_freqs, 16)
        heights = rolling_mean(raw_heights, 16)

        SAMPLE_TARGET = 200
        R2_TOLERANCE = 0.95

        indices = np.unique(freqs, return_index=True)[1]
        decimate = int(round(len(indices) / SAMPLE_TARGET))
        freqs = freqs[indices][::decimate]
        heights = heights[indices][::decimate]

        with open("/tmp/fit-cal.csv", "w") as fp:
            fp.write("freq,height\n")
            for f, h in zip(freqs, heights):
                fp.write(f"{f},{h}\n")

        def r2(p, x, y):
            y_hat = p(x)
            ss_res = np.sum((y - y_hat)**2)
            ss_tot = np.sum((y - np.mean(y))**2)
            return 1 - (ss_res / ss_tot)

        def best_fit(x, y, rawx, rawy):
            best_r2 = 0
            best_p = None
            for i in range(3, 9):
                p = npp.Polynomial.fit(x, y, i)
                r2_val = r2(p, rawx, rawy)
                if r2_val > best_r2:
                    best_r2 = r2_val
                    if r2_val > R2_TOLERANCE:
                        best_p = p
            return best_p, best_r2

        fth, r2_fth = best_fit(freqs, heights, raw_freqs, raw_heights)
        htf, r2_htf = best_fit(heights, freqs, raw_heights, raw_freqs)

        if fth is None or htf is None:
            msg = f"Calibration failed; unable to find a good fit. (R^2 {r2_fth:.3f}, {r2_htf:.3f})"
            if gcmd:
                gcmd.respond_raw(f"!! {msg}\n")
                return None, None
            raise self._eddy._printer.command_error(msg)

        msg = f"Calibrated eddy current polynomials for drive current {drive_current}:\n" + \
             f"   freq-to-height (R^2={r2_fth:.3f}): {self._coefs(fth)}\n" + \
             f"        domain: {fth.domain}\n" + \
             f"   height-to-freq (R^2={r2_htf:.3f}): {self._coefs(htf)}\n" + \
             f"        domain: {htf.domain}\n"

        logging.info(msg)
        if gcmd:
            gcmd.respond_info(msg, gcmd)

        self._freq_to_height = fth
        self._height_to_freq = htf
        self.drive_current = drive_current

        return (r2_fth, r2_htf)


    def save_calibration(self, gcmd: Optional[GCodeCommand] = None):
        if not self._freq_to_height or not self._height_to_freq:
            return

        def floatlist_to_str(vals):
            return str.join(', ', [str(v) for v in vals])

        ftoh_coefs = self._coefs(self._freq_to_height)
        htof_coefs = self._coefs(self._height_to_freq)

        configfile = self._eddy._printer.lookup_object('configfile')
        # why is there a floatarray getter, but not a setter?
        configfile.set(self._eddy._full_name, f"freq_to_height_p_{self.drive_current}", floatlist_to_str(ftoh_coefs))
        configfile.set(self._eddy._full_name, f"height_to_freq_p_{self.drive_current}", floatlist_to_str(htof_coefs))

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
        # TODO! this isn't really safe, we didn't home 0 and 1
        # fix this when we remove ToolheadMovementHelper
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
    # this is blowing up?
    toolhead.flush_step_generation()
    if at is None:
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in toolhead_kin.get_steppers()}
    else:
        kin_spos = {s.get_name(): s.mcu_to_commanded_position(s.get_past_mcu_position(at))
                    for s in toolhead_kin.get_steppers()}
    return toolhead_kin.calc_position(kin_spos)[2]

def get_past_toolhead_goal_z(printer, at):
    motion_report = printer.lookup_object("motion_report")
    dump_trapq = motion_report.trapqs.get("toolhead")
    if dump_trapq is None:
        raise printer.command_error("No dump trapq for toolhead")

    position, velocity = dump_trapq.get_trapq_position(at)
    if position is None:
        return None, None
    return position[2], velocity

def load_config_prefix(config: ConfigWrapper):
    return ProbeEddy(config)