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

        self.save_calibration()

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
        logging.info(str(self._coefs(self._freq_to_height)))
        logging.info(str(self._freq_to_height))
        logging.info(str(self._freq_to_height(freq)))
        return float(self._freq_to_height(freq))

    def height_to_freq(self, height: float, temp: float = None) -> float:
        if self._height_to_freq is None:
            return math.inf
        return float(self._height_to_freq(height))

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

            'home_trigger_z': 2.0,
            'trigger_freq_slop': 0.006,

            'calibration_z_max': 5.0,
            'calibration_step': 0.040,
        }

        for k in self.params:
            if 'speed' in k:
                self.params[k] = config.getfloat(k, self.params[k], above=0.0)
            else:
                self.params[k] = config.getfloat(k, self.params[k])

        self.offset = {
            "x": config.getfloat("x_offset", 0.0),
            "y": config.getfloat("y_offset", 0.0),
        }

        self._fmap = ProbeEddyFrequencyMap(self)
        self._fmap.load_from_config(config)

        self._endstop_wrapper = ProbeEddyEndstopWrapper(self)

        self._printer.lookup_object('pins').register_chip('probe', self)
        self._printer.add_object('probe', self)

        gcode = self._printer.lookup_object('gcode')
        self.define_commands(gcode)
    
    def define_commands(self, gcode):
        gcode.register_command("PROBE_EDDY_QUERY", self.cmd_QUERY)
        gcode.register_command("PROBE_EDDY_CALIBRATE", self.cmd_CALIBRATE)
    
    def cmd_QUERY(self, gcmd: GCodeCommand):
        reactor = self._printer.get_reactor()

        self._sensor._start_measurements()
        systime = reactor.monotonic()
        reactor.pause(systime + 0.050)
        status, freqval, freq = self._sensor.read_one_value()
        self._sensor._finish_measurements()

        height = self._fmap.freq_to_height(freq)
        gcmd.respond_info(f"Last coil value: {freq:.2f} ({hex(freqval)} ({height:.3f}mm) @ status: {self._sensor.status_to_str(status)} {hex(status)}")

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
            # we're going to set the kinematic position to kin_height to make
            # the following code easier, so it can assume z=0 is actually real zero
            th.set_absolute_position(*kin_pos)

            # away from bed, over nozzle position, then back to bed
            #th.move(0, 0, 5.0)
            #th.move(-self.offset['x'], -self.offset['y'], 0)

            #  FIXME FIXME FIXME -- PUT THIS BACK! -- FIXME FIXME FIXME
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

            with ProbeEddySampler(self._printer, self._sensor) as sampler:
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
        for th_past_time, th_recorded_z in toolhead_positions:
            th_past_z = th.get_kin_z(at=th_past_time)
            logging.info(f"calibration: time: {th_past_time} z: {th_recorded_z:.3f} past_z: {th_past_z:.3f}")

        # the samples are a list of [print_time, freq] pairs.
        # in toolhead_positions, we have a list of [print_time, kin_z] pairs.
        samples = sampler.get_samples()
        logging.info(f"samples:\n{samples}")
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
                
                # move on to the next note range
        except StopIteration:
            pass

        data_file.close()

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

        gcmd.respond_info(f"Collected {len(samples)} samples, filtered to {len(freqs)}")

        if len(freqs) == 0:
            gcmd.respond_raw("!! No samples collected\n")
            return

        # and build a map
        mapping = ProbeEddyFrequencyMap(self)
        r2_fth, r2_htf = mapping.calibrate_from_values(freqs, heights, gcmd)
        self._fmap = mapping

        gcmd.respond_info(f"Calibration complete, R^2: freq-to-height={r2_fth:.3f}, height-to-freq={r2_htf:.3f}")

    # Virtual endstop

    def setup_pin(self, pin_type, pin_params):
        if pin_type != "endstop" or pin_params["pin"] != "z_virtual_endstop":
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        if pin_params["invert"] or pin_params["pullup"]:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self._endstop_wrapper

    # Probe interface

    def multi_probe_begin(self):
        pass

    def multi_probe_end(self):
        pass

    def get_offsets(self):
        return self.offset["x"], self.offset["y"], self.params['home_trigger_z']

    def get_lift_speed(self, gcmd: Optional[GCodeCommand] = None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        return self.params['lift_speed']

    def get_samples(self, gcmd: Optional[GCodeCommand] = None):
        if gcmd is not None:
            return gcmd.get_int("SAMPLES", self.samples_config["samples"], minval=1)
        return 1 # self.samples_config["samples"]

    def get_sample_retract_dist(self, gcmd: Optional[GCodeCommand] = None):
        if gcmd is not None:
            return gcmd.get_float(
                "SAMPLE_RETRACT_DIST", self.samples_config["retract_dist"], above=0.0
            )
        return 0 # self.samples_config["retract_dist"]

    def get_samples_tolerance(self, gcmd: Optional[GCodeCommand] = None):
        if gcmd is not None:
            return gcmd.get_float(
                "SAMPLES_TOLERANCE", self.samples_config["tolerance"], minval=0.0
            )
        return 0.001

    def get_samples_tolerance_retries(self, gcmd: Optional[GCodeCommand] = None):
        if gcmd is not None:
            return gcmd.get_int(
                "SAMPLES_TOLERANCE_RETRIES",
                self.samples_config["tolerance_retries"],
                minval=0,
            )
        return 5

    def get_samples_result(self, gcmd: Optional[GCodeCommand] = None):
        if gcmd is not None:
            return gcmd.get("SAMPLES_RESULT", self.samples_config["result"])
        return self.samples_config["result"]

    def run_probe(self, gcmd: GCodeCommand):
        raise gcmd.error("can't run_probe")


# Tool to gather samples and convert them to probe positions
@final
class ProbeEddySampler:
    def __init__(self, printer, sensor):
        self._printer = printer
        self._sensor = sensor
        # Results storage
        self._samples = []
        self._probe_times = []
        self._probe_results = []
        self._need_stop = False
        self._errors = 0

    def __enter__(self):
        self._sensor.add_client(self._add_hw_measurement)
        #logging.info("ProbeEddySampler: __enter__")
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        #logging.info("ProbeEddySampler: __exit__")
        self._need_stop = True

    def _add_hw_measurement(self, msg):
        if self._need_stop:
            #logging.info(f"ProbeEddySampler: stopping in _add_hw_measurement")
            return False
        
        #logging.info(f"ProbeEddySampler: adding {len(msg['data'])} samples ({msg['errors']} errors)")
        self._errors += msg['errors']
        self._samples.extend(msg['data'])
        return True

    def finish(self):
        self._need_stop = True

    def get_samples(self):
        return self._samples.copy()
    
    def get_error_count(self):
        return self._errors

@final
class ProbeEddyEndstopWrapper:
    def __init__(self, eddy: ProbeEddy):
        self.eddy = eddy
        self._mcu = eddy._mcu

    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper: MCU_stepper):
        pass

    def get_steppers(self):
        return []

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        raise self.eddy._printer.error("Can't home yet")

    def home_wait(self, home_end_time):
        raise self.eddy._printer.error("Can't home yet")

    def query_endstop(self, print_time):
        return 1

    def get_position_endstop(self):
        return self.eddy.params['home_trigger_z']

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

    def get_kin_z(self, at = None):
        self._toolhead.flush_step_generation()
        if at is None:
            kin_spos = {s.get_name(): s.get_commanded_position()
                        for s in self._toolhead_kin.get_steppers()}
        else:
            kin_spos = {s.get_name(): s.mcu_to_commanded_position(s.get_past_mcu_position(at))
                        for s in self._toolhead_kin.get_steppers()}
        return self._toolhead_kin.calc_position(kin_spos)[2]


def load_config_prefix(config: ConfigWrapper):
    return ProbeEddy(config)