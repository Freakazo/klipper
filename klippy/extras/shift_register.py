import logging

from klippy import mcu


class ShiftRegisterPin:
    def __init__(self, pin_params):
        self._pin_params = pin_params

    def setup_max_duration(self, duration):
        pass

    def set_digital(self, print_time, value):
        pass

class ShiftRegister:
    _oid = None
    _set_cmd = None
    def __init__(self, config):
        logging.info(config.get_name() + " : Generic Shift Register Setup")

        printer = config.get_printer()
        self._mcu = mcu.get_printer_mcu(printer, config.get('mcu', 'mcu'))
        self._num_registers = config.getint('num_registers', 1)

        ppins = printer.lookup_object('pins')
        ppins.register_chip(config.get_name().split()[1], self)
        self._data_pin_params = ppins.lookup_pin(config.get('data_pin'))
        self._latch_pin_params = ppins.lookup_pin(config.get('latch_pin'))
        self._clock_pin_params = ppins.lookup_pin(config.get('clock_pin'))

        self._mcu.register_config_callback(self._build_config)

    def _build_config(self):
        # Hack to force shift register oids to always be greater than 1
        logging.info('The mcu is %s', self._mcu.get_name())
        self._mcu.create_oid()
        self._oid = self._mcu.create_oid()
        self._mcu.add_config_cmd(
            "config_shift_register oid=%d data_pin=%s clock_pin=%s latch_pin=%s num_registers=%d"
            % (self._oid, self._data_pin_params['pin'], self._clock_pin_params['pin'],
                                  self._latch_pin_params['pin'], self._num_registers))

    def setup_pin(self, pin_type, pin_params):
        return ShiftRegisterPin(pin_params)
        pass
        # todo



    @staticmethod
    def is_sreg():
        return True

    def get_oid(self):
        return self._oid

    def get_mcu(self):
        return self._mcu

def load_config_prefix(config):
    return ShiftRegister(config)

