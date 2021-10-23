from numpy import interp

from rclpy.logging import get_logger

from control.devices.base import VehicleState
from control.devices.communications import Communications
from control.can_utils.common import make_can_frame


class Throttle:

    def __init__(self, communications: Communications = None, dev_range=None, log_level=10):
        if dev_range is None:
            dev_range = [0.6, 4.8]
        self.cobid = 5
        self.name = 'Throttle'
        self.communications = communications
        self.device_range = dev_range
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.is_enable = False

    def set_pedal_pressure(self, press):
        if self.communications.CAN2.is_connected():
            if VehicleState.b_velocidad_request:
                self.logger.debug(f'Set press {press}')
                value = interp(max(min(press, 1), 0), [0, 1], self.device_range)
                self.communications.CAN2.add_to_queue(self.set_tension_value(value))
            else:
                self.raise_maximum()
        else:
            self.logger.debug('CAN2 is not connected throttle didnt send')

    def raise_maximum(self):
        self.communications.CAN2.add_to_queue(self.set_tension_value(self.device_range[0]))

    def shutdown(self):
        self.set_disable()

    def set_enable(self):
        self.is_enable = True
        return [
            make_can_frame(node=self.cobid, index=0x0001, data=self.device_range[0]),
            make_can_frame(node=self.cobid, index=0x0002, data=0x01)
        ]

    def set_disable(self):
        self.is_enable = False
        return [make_can_frame(node=self.cobid, index=0x0002, data=0x00)]

    def set_tension_value(self, tension: float):
        return [
            make_can_frame(node=self.cobid, index=0x0001, data=tension)
        ]
