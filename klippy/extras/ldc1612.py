# Support for reading frequency samples from ldc1612
#
# Copyright (C) 2020-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus, bulk_sensor

MIN_MSG_TIME = 0.100

BATCH_UPDATES = 0.100

LDC1612_ADDR = 0x2a

LDC1612_FREQ = 12000000
SETTLETIME = 0.005
DRIVECUR = 15

# TODO: configure these as part of calibration
DEGLITCH_1_0MHZ = 0x01
DEGLITCH_3_3MHZ = 0x04
DEGLITCH_10MHZ = 0x05
DEGLITCH_33MHZ = 0x07

DEGLITCH = DEGLITCH_10MHZ

LDC1612_MANUF_ID = 0x5449
LDC1612_DEV_ID = 0x3055

REG_RCOUNT0 = 0x08
REG_OFFSET0 = 0x0c
REG_SETTLECOUNT0 = 0x10
REG_CLOCK_DIVIDERS0 = 0x14
REG_ERROR_CONFIG = 0x19
REG_CONFIG = 0x1a
REG_MUX_CONFIG = 0x1b
REG_DRIVE_CURRENT0 = 0x1e
REG_MANUFACTURER_ID = 0x7e
REG_DEVICE_ID = 0x7f

# Tool for determining appropriate DRIVE_CURRENT register
class DriveCurrentCalibrate:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.sensor = sensor
        self.drive_cur = config.getint("reg_drive_current", DRIVECUR,
                                       minval=0, maxval=31)
        self.name = config.get_name()
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("LDC_CALIBRATE_DRIVE_CURRENT",
                                   "CHIP", self.name.split()[-1],
                                   self.cmd_LDC_CALIBRATE,
                                   desc=self.cmd_LDC_CALIBRATE_help)
        gcode.register_mux_command("LDC_SET_DRIVE_CURRENT",
                                   "CHIP", self.name.split()[-1],
                                   self.cmd_LDC_SET,
                                   desc=self.cmd_LDC_SET_help)
    def get_drive_current(self):
        return self.drive_cur
    cmd_LDC_SET_help = "Set LDC1612 DRIVE_CURRENT register (idrive value only)"
    def cmd_LDC_SET(self, gcmd):
        drive_cur = gcmd.get_int('VAL', self.drive_cur, minval=0, maxval=31)
        self.sensor.set_drive_current(drive_cur)
        self.drive_cur = drive_cur
        gcmd.respond_info("%s: ldc drive current: %d" % (self.name, drive_cur))
    cmd_LDC_CALIBRATE_help = "Calibrate LDC1612 DRIVE_CURRENT register"
    def cmd_LDC_CALIBRATE(self, gcmd):
        is_in_progress = True
        def handle_batch(msg):
            return is_in_progress
        self.sensor.add_client(handle_batch)
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(0.100)
        toolhead.wait_moves()
        old_config = self.sensor.read_reg(REG_CONFIG)
        self.sensor.set_reg(REG_CONFIG, 0x001 | (1<<9))
        toolhead.wait_moves()
        toolhead.dwell(0.100)
        toolhead.wait_moves()
        reg_drive_current0 = self.sensor.read_reg(REG_DRIVE_CURRENT0)
        self.sensor.set_reg(REG_CONFIG, old_config)
        is_in_progress = False
        # Report found value to user
        drive_cur = (reg_drive_current0 >> 6) & 0x1f
        gcmd.respond_info(
            "%s: reg_drive_current: %d\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (self.name, drive_cur))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'reg_drive_current', "%d" % (drive_cur,))

# Interface class to LDC1612 mcu support
class LDC1612:
    def __init__(self, config, calibration=None):
        self.printer = config.get_printer()
        self.calibration = calibration
        self.dccal = DriveCurrentCalibrate(config, self)
        self._drive_current = self.dccal.get_drive_current()
        self.data_rate = config.getint("samples_per_second", 250, minval=50)
        self._start_count = 0
        # Setup mcu sensor_ldc1612 bulk query code
        self.i2c = bus.MCU_I2C_from_config(config,
                                           default_addr=LDC1612_ADDR,
                                           default_speed=400000)
        self.mcu = mcu = self.i2c.get_mcu()
        self.oid = oid = mcu.create_oid()
        self.ldc1612_start_stop_cmd = None
        self.ldc1612_setup_home_cmd = self.ldc1612_setup_home2_cmd = self.query_ldc1612_home_state_cmd = None
        if config.get('intb_pin', None) is not None:
            ppins = config.get_printer().lookup_object("pins")
            pin_params = ppins.lookup_pin(config.get('intb_pin'))
            if pin_params['chip'] != mcu:
                raise config.error("ldc1612 intb_pin must be on same mcu")
            mcu.add_config_cmd(
                "config_ldc1612_with_intb oid=%d i2c_oid=%d intb_pin=%s"
                % (oid, self.i2c.get_oid(), pin_params['pin']))
        else:
            mcu.add_config_cmd("config_ldc1612 oid=%d i2c_oid=%d"
                               % (oid, self.i2c.get_oid()))

        # TODO setup averaging for ldc2
        #mcu.add_config_cmd("ldc1612_setup_averaging oid=%d factor=%d" % (oid, factor))
        mcu.add_config_cmd("ldc1612_start_stop oid=%d rest_ticks=0" % (oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        # Bulk sample message reading
        chip_smooth = self.data_rate * BATCH_UPDATES * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, ">I")
        self.last_error_count = 0
        self.last_err_kind = 0
        self._chip_initialized = False
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch,
            self._start_measurements, self._finish_measurements, BATCH_UPDATES)
        self.name = config.get_name().split()[-1]
        hdr = ('time', 'frequency', 'z')
        self.batch_bulk.add_mux_endpoint("ldc1612/dump_ldc1612", "sensor",
                                         self.name, {'header': hdr})
    def _build_config(self):
        cmdqueue = self.i2c.get_command_queue()
        self.ldc1612_start_stop_cmd = self.mcu.lookup_command(
            "ldc1612_start_stop oid=%c rest_ticks=%u", cq=cmdqueue)
        self.ffreader.setup_query_command("ldc1612_query_bulk_status oid=%c",
                                          oid=self.oid, cq=cmdqueue)
        self.ldc1612_setup_home_cmd = self.mcu.lookup_command(
            "ldc1612_setup_home oid=%c clock=%u threshold=%u"
            " trsync_oid=%c trigger_reason=%c error_reason=%c", cq=cmdqueue)
        self.query_ldc1612_home_state_cmd = self.mcu.lookup_query_command(
            "query_ldc1612_home_state oid=%c",
            "ldc1612_home_state oid=%c homing=%c trigger_clock=%u",
            oid=self.oid, cq=cmdqueue)
        self.query_ldc1612_latched_status_cmd = self.mcu.lookup_query_command(
            "query_ldc1612_latched_status oid=%c",
            "ldc1612_latched_status oid=%c status=%u lastval=%u",
            oid=self.oid, cq=cmdqueue)
        self.mcu.register_response(self._handle_debug_print, "debug_print")
        self.ldc1612_setup_home2_cmd = self.mcu.lookup_command(
             "ldc1612_setup_home2 oid=%c"
             " trsync_oid=%c trigger_reason=%c other_reason_base=%c"
             " trigger_freq=%u start_freq=%u start_time=%u tap_threshold=%i", cq=cmdqueue);
        self.ldc1612_finish_home2_cmd = self.mcu.lookup_query_command(
             "ldc1612_finish_home2 oid=%c",
             "ldc1612_finish_home2_reply oid=%c homing=%c trigger_clock=%u tap_start_clock=%u tap_amount=%u",
             oid=self.oid, cq=cmdqueue)
    def _handle_debug_print(self, params):
        logging.info(params["m"])
    def get_mcu(self):
        return self.i2c.get_mcu()
    def read_reg(self, reg):
        params = self.i2c.i2c_read([reg], 2)
        response = bytearray(params['response'])
        return (response[0] << 8) | response[1]
    def set_reg(self, reg, val, minclock=0):
        self.i2c.i2c_write([reg, (val >> 8) & 0xff, val & 0xff],
                           minclock=minclock)
    def add_client(self, cb):
        self.batch_bulk.add_client(cb)
    def latched_status(self):
        response = self.query_ldc1612_latched_status_cmd.send([self.oid])
        return response['status']

    def latched_status_str(self):
        s = self.latched_status()
        return self.status_to_str(s)

    def status_to_str(self, s: int):
        result = []
        if (s & (1<<15)) != 0: result.append('ERR_CH1')
        if (s & (1<<13)) != 0: result.append('ERR_UR')
        if (s & (1<<12)) != 0: result.append('ERR_OR')
        if (s & (1<<11)) != 0: result.append('ERR_WD')
        if (s & (1<<10)) != 0: result.append('ERR_AHE')
        if (s & (1<<9)) != 0: result.append('ERR_ALE')
        if (s & (1<<8)) != 0: result.append('ERR_ZC')
        if (s & (1<<6)) != 0: result.append('DRDY')
        if (s & (1<<3)) != 0: result.append('UNREADCONV1')
        return str.join(' ', result)

    def read_one_value(self):
        self._init_chip()
        res = self.query_ldc1612_latched_status_cmd.send([self.oid])
        return (res['status'], res['lastval'], self.from_ldc_freqval(res['lastval'], ignore_err=True))
    # Homing
    def to_ldc_freqval(self, freq):
        return int(freq * (1<<28) / float(LDC1612_FREQ) + 0.5)
    def from_ldc_freqval(self, val, ignore_err = False):
        if val >= 0x0fffffff and not ignore_err:
            raise self.printer.command_error(f"LDC1612 frequency value has error bits: {hex(val)}")
        return round(val * (float(LDC1612_FREQ) / (1<<28)), 3)

    def setup_home(self, print_time, trigger_freq,
                   trsync_oid, hit_reason, err_reason):
        clock = self.mcu.print_time_to_clock(print_time)
        tfreq = self.to_ldc_freqval(trigger_freq) #int(trigger_freq * (1<<28) / float(LDC1612_FREQ) + 0.5)
        logging.info(f"LD1612 setup_home: {clock} {trigger_freq:.2f} ({hex(tfreq)}), trsync {trsync_oid}")
        self.ldc1612_setup_home_cmd.send(
            [self.oid, clock, tfreq, trsync_oid, hit_reason, err_reason,
             0, 0, 0, 0])

    def setup_home2(self, trsync_oid, hit_reason, reason_base,
                    trigger_freq, start_freq, start_time, tap_threshold=0):
        t_freqvl = self.to_ldc_freqval(trigger_freq)
        s_freqval = self.to_ldc_freqval(start_freq)
        start_time_mcu = self.mcu.print_time_to_clock(start_time) if start_time > 0 else 0
        logging.info(f"LD1612 setup_home2: trigger: {trigger_freq:.2f} ({t_freqvl}) safe: {start_freq:.2f} ({s_freqval}) @ {start_time:.2f} ({start_time_mcu}) trsync: {trsync_oid} {hit_reason} {reason_base} TAP: {tap_threshold}")
        self.ldc1612_setup_home2_cmd.send([self.oid, trsync_oid, hit_reason,
                                           reason_base, t_freqvl, s_freqval, start_time_mcu, tap_threshold])

    def finish_home2(self):
        # "ldc1612_finish_home2_reply oid=%c homing=%c trigger_clock=%u tap_start_clock=%u tap_amount=%u",
        reply = self.ldc1612_finish_home2_cmd.send([self.oid])
        active = reply['homing'] is not 0
        trigger_time = self.mcu.clock_to_print_time(self.mcu.clock32_to_clock64(reply['trigger_clock']))
        tap_start_time = self.mcu.clock_to_print_time(self.mcu.clock32_to_clock64(reply['tap_start_clock']))
        tap_amount = reply['tap_amount']
        return active, trigger_time, tap_start_time, tap_amount

    def clear_home(self):
        self.ldc1612_setup_home_cmd.send([self.oid, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        if self.mcu.is_fileoutput():
            return 0.
        params = self.query_ldc1612_home_state_cmd.send([self.oid])
        tclock = self.mcu.clock32_to_clock64(params['trigger_clock'])
        return self.mcu.clock_to_print_time(tclock)

    def conversion_ratio(self):
        return float(LDC1612_FREQ) / (1<<28)

    # Measurement decoding
    def _convert_samples(self, samples):
        freq_conv = float(LDC1612_FREQ) / (1<<28)
        count = 0
        for ptime, val in samples:
            if val > 0x0fffffff: # high nibble indicates an error
                if self.last_err_kind != (val >> 28):
                    logging.info(f"LDC1612 error: {hex(val)}")
                    self.last_err_kind = val >> 28
                self.last_error_count += 1
            else:
                #samples[count] = (round(ptime, 6), round(freq_conv * val, 3), 999.9)
                samples[count] = (round(ptime, 6), val, 999.9)
                count += 1
        # remove the error samples
        del samples[count:]

    def _verify_chip(self):
        # In case of miswiring, testing LDC1612 device ID prevents treating
        # noise or wrong signal as a correctly initialized device
        manuf_id = self.read_reg(REG_MANUFACTURER_ID)
        dev_id = self.read_reg(REG_DEVICE_ID)
        if manuf_id != LDC1612_MANUF_ID or dev_id != LDC1612_DEV_ID:
            raise self.printer.command_error(
                "Invalid ldc1612 id (got %x,%x vs %x,%x).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty ldc1612 chip."
                % (manuf_id, dev_id, LDC1612_MANUF_ID, LDC1612_DEV_ID))

    def _init_chip(self):
        if self._chip_initialized:
            return

        self._verify_chip()

        ldc_fref = 12_000_000

        # samples/sec
        data_rate = self.data_rate
        # TODO: use freq_max to pick deglitch
        freq_max = 3_200_000
        deglitch = DEGLITCH_3_3MHZ
        # this is the settle time for the initial conversion (and initial conversion only),
        # there's no reason for this to be small
        settle_time = SETTLETIME


        val_settle_count = 0xffff # int(SETTLETIME * freq / 16. + .5)


        # This is the TI-recommended register configuration order
        # Setup chip in requested query rate
        rcount0 = LDC1612_FREQ / (16. * (self.data_rate - 4))
        self.set_reg(REG_RCOUNT0, int(rcount0 + 0.5))
        self.set_reg(REG_OFFSET0, 0)
        self.set_reg(REG_SETTLECOUNT0, int(SETTLETIME*LDC1612_FREQ/16. + .5))
        self.set_reg(REG_CLOCK_DIVIDERS0, (1 << 12) | 1)
        self.set_reg(REG_ERROR_CONFIG, 0b1111_1100_1111_1001) # report everything to STATUS and INTB except ZC
        self.set_reg(REG_MUX_CONFIG, 0x0208 | DEGLITCH)
        # RP_OVERRIDE_EN | AUTO_AMP_DIS | REF_CLK_SRC=clkin | reserved
        self.set_reg(REG_CONFIG, (1<<12) | (1<<10) | (1<<9) | 0x001)
        self.set_reg(REG_DRIVE_CURRENT0, self._drive_current << 11)
        self._initialized = True

    def get_drive_current(self) -> int:
        return self._drive_current

    def set_drive_current(self, cval: int):
        if cval < 0 or cval > 31:
            raise self.printer.command_error("Drive current must be between 0 and 31")
        if self._drive_current == cval:
            return

        self._drive_current = cval
        self.set_reg(REG_DRIVE_CURRENT0, cval << 11)

    # Start, stop, and process message batches
    def _start_measurements(self):
        self._init_chip()
        self._start_count += 1

        if self._start_count > 1:
            logging.info("LDC1612 start count: %d", self._start_count)
            return

        # Start bulk reading
        rest_ticks = self.mcu.seconds_to_clock(0.5 / self.data_rate)
        self.ldc1612_start_stop_cmd.send([self.oid, rest_ticks])
        logging.info("LDC1612 starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.ffreader.note_start()
        self.last_error_count = 0
    def _finish_measurements(self):
        self._start_count -= 1

        if self._start_count > 0:
            logging.info("LDC1612 stop, start count now: %d", self._start_count)
            return

        # Halt bulk reading
        self.ldc1612_start_stop_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("LDC1612 finished '%s' measurements", self.name)
    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        if not samples:
            return {}
        if self.calibration is not None:
            self.calibration.apply_calibration(samples)
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}
