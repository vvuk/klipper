import logging, math, bisect
import mcu
import numpy as np
import numpy.polynomial as npp

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
    def __init__(self, config: ConfigWrapper, sensor: ldc1612.LDC1612, temp_sensor: PrinterSensorGeneric):
        self._sensor = sensor
        self._temp_sensor = temp_sensor

        ftoh = config.getfloatlist("freq_to_height_p", default=None)
        htof = config.getfloatlist("height_to_freq_p", default=None)

        # TODO: save domain and window
        if ftoh and htof:
            self.freq_to_height = npp.Polynomial(ftoh)
            self.height_to_freq = npp.Polynomial(htof)
        else:
            self.freq_to_height = None
            self.height_to_freq = None

    def calibrate_from_values(self, freqs: List[float], heights: List[float]):
        if len(freqs) != len(heights):
            raise ValueError("freqs and heights must be the same length")
        freqs = np.array(freqs)
        heights = np.array(heights)

        # empirically, freq-to-position results need a 3D polynomial to hit R^2=1
        # where position-to-frequency fits cleanly with a 2D polynomial
        self.freq_to_height = npp.Polynomial.fit(freqs, heights, 3)
        self.height_to_freq = npp.Polynomial.fit(heights, freqs, 2)

        # estimate R^2 for both
        def r2(p, x, y):
            y_hat = p(x)
            ss_res = np.sum((y - y_hat)**2)
            ss_tot = np.sum((y - np.mean(y))**2)
            return 1 - (ss_res / ss_tot)
        r2_fth = r2(self.freq_to_height, freqs, heights)
        r2_htf = r2(self.height_to_freq, heights, freqs)

        logging.info(f"Calibrated polynomial R^2: freq-to-height={r2_fth:.3f}, height-to-freq={r2_htf:.3f}")
        return (r2_fth, r2_htf)

    def save_calibration(self, cfile: ConfigWrapper):
        if self.freq_to_height:
            cfile.set("freq_to_height_p", self.freq_to_height.coef)
        if self.height_to_freq:
            cfile.set("height_to_freq_p", self.height_to_freq.coef)

    def raw_freqval_to_height(self, raw_freq: int, temp: float = None) -> float:
        return self.freq_to_height(self._sensor.from_ldc_freqval(raw_freq))
    def height_to_raw_freqval(self, height: float, temp: float = None) -> int:
        return self._sensor.to_ldc_freqval(self.height_to_freq(height))
    def freq_to_height(self, freq: float, temp: float = None) -> float:
        if self.freq_to_height is None:
            return math.inf
        return float(self.freq_to_height(freq))
    def height_to_freq(self, height: float, temp: float = None) -> float:
        if self.height_to_freq is None:
            return math.inf
        return float(self.height_to_freq(height))

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

        self._fmap = ProbeEddyFrequencyMap(config, self._sensor, self._temp_sensor)

        self._endstop_wrapper = ProbeEddyEndstopWrapper(self)

        self._printer.lookup_object('pins').register_chip('probe', self)
        self._printer.add_object('probe', self)

        gcode = self._printer.lookup_object('gcode')
        self.define_commands(gcode)
    
    def define_commands(self, gcode):
        gcode.register_command("PROBE_EDDY_QUERY", self.cmd_QUERY)
        gcode.register_command("PROBE_EDDY_CALIBRATE", self.cmd_CALIBRATE)
    
    def cmd_QUERY(self, gcmd: GCodeCommand):
        freq = self._sensor.read_one_value()[1]
        height = self._fmap.raw_freqval_to_height(freq)
        gcmd.respond_info(f"Current coil value: {freq} ({height:.3f}mm)")
    
    def cmd_CALIBRATE(self, gcmd: GCodeCommand):
        manual_probe.ManualProbeHelper(self._printer, gcmd,
                                       lambda kin_pos: self.cmd_CALIBRATE_next(gcmd, kin_pos))
    def cmd_CALIBRATE_next(self, gcmd: GCodeCommand, kin_pos: float):
        toolhead: ToolHead = self._printer.lookup_object('toolhead')
        toolhead_kin = toolhead.get_kinematics()

        # we're going to set the kinematic position to kin_height to make
        # the following code easier, so it can assume z=0 is actually real zero
        curpos = list(kin_pos)
        toolhead.set_position(curpos, homing_axes=(0, 1, 2))

        def move_to(x: float, y: float, z: float):
            toolhead.manual_move((x, y, z), self.params['probe_speed'])
            toolhead.wait_moves()
            curpos = toolhead.get_position()

        def move_by(xoffs: float, yoffs: float, zoffs: float):
            move_to(curpos[0] + xoffs, curpos[1] + yoffs, curpos[2] + zoffs)
            curpos = toolhead.get_position()

        def move_to_z(z: float):
            move_to(curpos[0], curpos[1], z)
            curpos = toolhead.get_position()

        def toolhead_kin_z(at = None):
            toolhead.flush_step_generation()
            if at is None:
                kin_spos = {s.get_name(): s.get_commanded_position()
                            for s in toolhead_kin.get_steppers()}
            else:
                kin_spos = {s.get_name(): s.mcu_to_commanded_position(s.get_past_mcu_position(at))
                            for s in toolhead_kin.get_steppers()}
            return toolhead_kin.calc_position(kin_spos)[2]

        # away from bed, over nozzle position, then back to bed
        #move(0, 0, 5.0)
        #move(-self.offset['x'], -self.offset['y'], 0)

        move_by(-self.offset['x'], -self.offset['y'], 5.0)

        #move(0, 0, -5.0)

        # now do the calibration move.
        # move to the start of calibration
        cal_z_max = self.params['calibration_z_max']
        move_to_z(cal_z_max)

        mapping = None
        toolhead_positions = []
        first_sample_time = None
        last_sample_time = None

        with ProbeEddySampler(self._printer, self._sensor) as sampler:
            # move down in steps, taking samples
            for z in np.arange(cal_z_max, 0, -self.params['calibration_step']):
                # hop up, then move downward to z to reduce backlash
                move_to_z(z + 1.000)
                move_to_z(z)
                toolhead_time = toolhead.get_last_move_time()
                toolhead.dwell(0.200)

                first_sample_time = first_sample_time or toolhead_time
                last_sample_time = toolhead_time

                toolhead_positions.append((toolhead_time, toolhead_kin_z()))

        move_to_z(5.0)

        # verify historical movements
        for th_past_time, th_recorded_z in toolhead_positions:
            th_past_z = toolhead_kin_z(at=th_past_time)
            if abs(th_past_z - th_recorded_z) > 0.001:
                gcmd.respond_error(f"Error in historical movement at {th_past_time}: {th_past_z} != {th_recorded_z}")


        # the samples are a list of [print_time, freq] pairs.
        # in toolhead_positions, we have a list of [print_time, kin_z] pairs.
        samples = sampler.get_samples()
        # delete samples that come before first_time or after last_time
        samples = [s for s in samples if first_sample_time <= s[0] <= last_sample_time]

        gcmd.respond_info(f"Collected {len(samples)} samples")

        # now pull out the freqs, and the kin_z values based on the sample time
        # from the movement history.
        # TODO: verify with toolhead_positions, but I don't see why this would be wrong
        freqs = [s[1] for s in samples]
        heights = [toolhead_kin_z(at=s[0]) for s in samples]

        # and build a map
        mapping = ProbeEddyFrequencyMap(self._printer, self._sensor, self._temp_sensor)
        r2_fth, r2_htf = mapping.calibrate_from_values(freqs, heights)

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
    
    def __exit__(self, exc_type, exc_value, traceback):
        self._need_stop = True

    def _add_hw_measurement(self, msg):
        if self._need_stop:
            del self._samples[:]
            return False
        
        self._errors += msg['errors']
        # make sure we store a copy here, no idea who owns the data
        # data: [time, freq]
        self._samples.append(list(msg['data']))
        return True

    def finish(self):
        self._need_stop = True

    def get_samples(self):
        return list(self._samples)
    
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

def load_config_prefix(config: ConfigWrapper):
    return ProbeEddy(config)