from numpy import interp

from rclpy.logging import get_logger

from control.devices.base import VehicleState
from control.devices.communications import Communications
from control.can_utils.common import make_can_frame


class Throttle:

    def __init__(self, communications: Communications = None, dev_range=None, log_level=10):
        if dev_range is None:
            dev_range = [0.9, 3.3]
        self.name = 'Throttle'
        self.communications = communications
        self.device_range = dev_range
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.is_enable = False

    def set_pedal_pressure(self, press):
        if self.communications.AIO.is_connected():
            if VehicleState.b_velocidad_request and VehicleState.marchas_real != 'N':
                self.logger.debug(f'Set press {press}')
                value = interp(max(min(press, 1), 0), [0, 1], self.device_range)
                if value > 0.7:
                    self.communications.AIO.put_queue([[4.92, 1], [value, 0]])
                    self.communications.AIO.put_queue([[4.92, 1], [value, 0]])
                else:
                    self.communications.AIO.put_queue([[0.1, 1], [value, 0]])
                    self.communications.AIO.put_queue([[0.1, 1], [value, 0]])
            else:
                self.raise_maximum()
        else:
            self.logger.debug('AIO is not connected throttle didnt send')

    def raise_maximum(self):
        self.communications.AIO.put_queue([[0.1, 1], [0.6, 0]])  # sleep final 0.05

    def set_enable(self):
        self.logger.debug(f'{self.name}: Enable')
        if self.communications.AIO.is_connected():
            VehicleState.b_acelerador = True
            self.communications.AIO.put_queue([[0.1, 1, 0.1], [0.6, 0, 0.05], [3.0, 2, 0.1, 0.1]])
            self.is_enable = True

    def set_disable(self):
        self.logger.debug(f'{self.name}: Disable')
        if self.communications.AIO.is_connected():
            VehicleState.b_acelerador = False
            self.communications.AIO.put_queue([[0.0, 2, 0.1], [0.1, 1, 0.1], [0.6, 0, 0.05], [0.0, 2, 0.1, 0.1]])
            self.is_enable = False

    def shutdown(self):
        self.set_disable()

    def enable(node: int):
        return [make_can_frame(node=node, index=0x0002, data=0x01)]

    def disable(node: int):
        return [make_can_frame(node=node, index=0x0002, data=0x00)]

    def set_tension_value(node: int, tension: int):
        return [
            make_can_frame(node=node, index=0x0001, data=tension)
        ]
