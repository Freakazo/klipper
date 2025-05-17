import logging

from klippy import pins
from .bus import MCU_SPI_from_config


class Shift595Pin:
    def __init__(self, pin_params):
        pin = pin_params['pin']
        self._index = int(pin.strip())
        self._shifter = pin_params['chip']

    def setup_pin(self, enable_value):
        self.set_digital(enable_value)

    def setup_max_duration(self, _):
        pass

    def setup_start_value(self, start_value, shutdown_value):
        self.set_digital(0, start_value)

    def get_mcu(self):
        return self._shifter.get_mcu()

    def set_digital(self, print_time, value):
        self._shifter.set_output(self._index, value)

    def set_pwm(self, value):
        self.set_digital(0, value >= 0.5)

    def set_analog(self, value):
        self.set_pwm(value)



    def get_digital(self):
        byte_index = self._index // 8
        bit_index = self._index % 8
        return bool(self._shifter.state[byte_index] & (1 << bit_index))


class Shift595:
    def __init__(self, config):
        logging.info(config.get_name() + " : Shift595 Setup")

        self.spi = MCU_SPI_from_config(config, 0)
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.latch_pin = ppins.setup_pin('digital_out', config.get('latch_pin'))
        ppins.register_chip(config.get_name().split()[1], self)
        self.num_outputs = config.getint("num_outputs", minval=1)
        self.num_bytes = (self.num_outputs + 7) // 8
        self.state = bytearray(self.num_bytes)
        self.need_update = True
        self.reactor = self.printer.get_reactor()
        self.timer_handler = self.reactor.register_timer(self._handle_update)
        self._pins = {}
        logging.info(config.get_name() + " : Shift595 Setup done")


    def setup_pin(self, pin_type, pin_params):
        pin = pin_params['pin']
        if pin_type != 'digital_out':
            raise pins.error('Shift595 pin %s is not valid' % pin)
        pin_instance = Shift595Pin(pin_params)
        self._pins[pin] = pin_instance
        return pin_instance

    def _handle_update(self, print_time):
        if not self.need_update:
            return self.reactor.NEVER
        if self.latch_pin._set_cmd is None:
            logging.info('Not enabled yet so ignoring')
            return self.reactor.NEVER
        # Send MSB first (highest index byte first)
        logging.info('Allo', self.latch_pin)
        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        print_time = self.get_mcu().estimated_print_time(curtime + 0.1)
        # self.latch_pin.set_digital(print_time + 0.100, 0)
        logging.info('Allo - %s %s %s', self.get_mcu().get_name(), curtime, print_time)

        self.spi.spi_send(list(reversed(self.state)))
        reactor.pause(0.100)
        self.latch_pin.update_digital(1)
        reactor.pause(0.100)
        self.latch_pin.update_digital(0)

        # self.latch_pin.set_digital(print_time + 0.102, 0)
        self.need_update = False
        return self.reactor.NEVER

    def set_output(self, index, value):
        logging.info('Should be updating the values now %s %s', index, value)
        byte_index = index // 8
        bit_index = index % 8
        if value:
            self.state[byte_index] |= (1 << bit_index)
        else:
            self.state[byte_index] &= ~(1 << bit_index)
        self.need_update = True
        self.reactor.update_timer(self.timer_handler, self.reactor.NOW)

    def get_mcu(self):
        return self.spi.get_mcu()

def load_config_prefix(config):
    return Shift595(config)

