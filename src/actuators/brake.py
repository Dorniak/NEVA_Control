import threading
from time import sleep
from numpy import interp
from rclpy.node import Node

from rclpy.logging import get_logger

from src.devices.base import VehicleState
from src.can_utils.common import make_can_frame
from src.devices.communications import Communications


class Brake:

    def __init__(self, cobid=2, node: Node = None, communications: Communications = None, dev_range=None, log_level=10):
        if dev_range is None:
            dev_range = [0, -110000]
        self.name = 'Brake'
        self.node = node
        self.communications = communications
        self.cobid = cobid
        self.device_range = dev_range
        self.enable = False
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.shutdown = False
        self.timer = self.node.create_timer(0.5, self.get_status)

    def get_status(self):
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x6041, write=False),
        ])

    def set_pedal_pressure(self, press):
        if VehicleState.b_velocidad_request:
            value = interp(max(min(press, 1), 0), [0, 1], self.device_range)
            self.logger.debug(f'{self.name}: Set pedal pressure {value}')
            self.set_motor_abs(value)

    def raise_maximum(self):
        self.set_motor_abs(self.device_range[0])

    def drop_maximum(self):
        self.set_motor_abs(self.device_range[1])

    def set_motor_abs(self, value):
        self.logger.debug(f'{self.name}: Abs {value}')
        # cuentas = int((402000 * value) / 360)
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x607A, data=value),
            make_can_frame(node=self.cobid, index=0x6040, data=0x000F),
            make_can_frame(node=self.cobid, index=0x6040, data=0x003F)
        ])

    def set_motor_rel(self, value):
        self.logger.debug(f'{self.name}: Rel {value}')
        # cuentas = int((402000 * value) / 360)
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x607A, data=value),
            make_can_frame(node=self.cobid, index=0x6040, data=0x000F),
            make_can_frame(node=self.cobid, index=0x6040, data=0x007F)
        ])

    def set_enable(self):
        VehicleState.b_freno = True
        self.logger.debug(f'{self.name}: Enable')
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x6040, data=0x0080),
            make_can_frame(node=self.cobid, index=0x6060, data=0x01),
            make_can_frame(node=self.cobid, index=0x6040, data=0x0006),
            make_can_frame(node=self.cobid, index=0x6040, data=0x000F)
        ])

    def set_disable(self):
        self.logger.debug(f'{self.name}: False')
        VehicleState.b_freno = False
        pass
