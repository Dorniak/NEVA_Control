import threading

from rclpy.node import Node
from rclpy.logging import get_logger

from src.can_utils.common import make_can_frame
from src.devices.base import VehicleState
from src.devices.communications import Communications
from src.devices.pid import PIDF
import numpy as np
from time import sleep


class Steering:

    def __init__(self, cobid, node: Node, communications: Communications, log_level=10):
        self.dev_range = [-400, 400]
        self.shutdown_flag = False
        self.Steering_Max_Torque = 32000
        self.Steering_Min_Torque = -32000
        self.name = 'Steering'
        self.communications = communications
        self.cobid = cobid
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.node = node
        self.pid = PIDF(kp=0.1, ti=0.2, td=0.1, anti_wind_up=0.4, pro_wind_up=0)
        self.value = 0
        self.init_device()
        self.timer = self.node.create_timer(0.1, self.sender)
        self.timer = self.node.create_timer(0.5, self.get_status)
        self.timer_pid = threading.Thread(target=self.pid_timer, daemon=False)
        self.timer_pid.start()

    def get_status(self):
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x6041, write=False),
        ])

    def pid_timer(self):
        while not self.shutdown_flag:
            taN = np.interp(VehicleState.direccion_real, self.dev_range, [-1., 1.])
            caN = np.interp(VehicleState.direccion, self.dev_range, [-1., 1.])
            if VehicleState.b_direccion:
                self.value = -self.pid.calcValue(target_value=taN, current_value=caN)
            else:
                self.pid.reset_values()
            sleep(0.1)

    def sender(self):
        # self.logger.debug(f'sender: {VehicleState.b_direccion_request}')
        if VehicleState.b_direccion_request:
            if not VehicleState.b_direccion:
                self.logger.debug('Try to enable')
                self.set_enable()

            torque = np.interp(self.value, [-1, 1], [self.Steering_Min_Torque, self.Steering_Max_Torque])
            # self.logger.debug(f'Direccion {VehicleState.direccion}  Real {VehicleState.direccion_real}')
            error = VehicleState.direccion - VehicleState.direccion_real
            # self.logger.debug(f'Send: {error}  abs value: {VehicleState.direccion}')
            self.logger.debug(f'Torque {torque}')
            self.communications.CAN2.add_to_queue([
                make_can_frame(node=self.cobid, index=0x6071, sub_index=0, data=int(torque))])
        else:
            if VehicleState.b_direccion:
                self.set_disable()

    def set_enable(self):
        self.logger.debug('Sending enable sequence')
        if self.communications.CAN2.is_connected():
            self.communications.CAN2.add_to_queue([
                make_can_frame(node=self.cobid, index=0x6040, data=0x80),
                make_can_frame(node=self.cobid, index=0x6060, data=10),  # mode profile torque mode
                make_can_frame(node=self.cobid, index=0x6040, data=0x6),
                make_can_frame(node=self.cobid, index=0x6040, data=0xF),
                make_can_frame(node=self.cobid, index=0x60FE, sub_index=1, data=0x10000)
            ])
            VehicleState.b_direccion = True

    def init_device(self):
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x6040, data=0x0080),
            make_can_frame(node=self.cobid, index=0x6081, data=0x1388),  # rpm 5000
            make_can_frame(node=self.cobid, index=0x6060, data=10),  # mode profile torque mode
        ])

    def set_disable(self):
        if self.communications.CAN2.is_connected():
            self.communications.CAN2.add_to_queue([
                make_can_frame(node=self.cobid, index=0x60FE, sub_index=1, data=0x0),
                make_can_frame(node=self.cobid, index=0x6040, data=0X07)
            ])
            VehicleState.b_direccion = False

    def shutdown(self):
        self.logger.warn('Shutdown')
        self.shutdown_flag = True
        self.timer.cancel()
