import threading

from rclpy.node import Node
from rclpy.logging import get_logger

from control.can_utils.common import make_can_frame
from control.devices.base import VehicleState
from control.devices.communications import Communications
from control.devices.pid import PIDF
import numpy as np
from time import sleep


class Steering:

    def __init__(self, cobid, node: Node, communications: Communications, log_level=10):
        self.shutdown_flag = False
        self.Steering_Max_Tension = 5.
        self.Steering_Min_Tension = 0.
        self.name = 'Steering'
        self.communications = communications
        self.cobid = cobid
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.node = node
        self.pid = PIDF(kp=0.3, ti=0.2, td=0.1, anti_wind_up=0.4, pro_wind_up=0)
        self.value = 0
        self.timer = self.node.create_timer(0.1, self.sender)
        self.timer_pid = threading.Thread(target=self.pid_timer, daemon=False)
        self.timer_pid.start()

    def pid_timer(self):
        while not self.shutdown_flag:
            taN = np.interp(VehicleState.direccion_real, [-630., 630.], [-1., 1.])
            caN = np.interp(VehicleState.direccion, [-630., 630.], [-1., 1.])
            self.value = -self.pid.calcValue(target_value=taN, current_value=caN)
            sleep(0.1)

    def sender(self):
        if VehicleState.b_direccion_request:
            if not VehicleState.b_direccion:
                self.logger.debug('Try to enable')
                self.set_enable()
                self.set_magnet_enable()

            tension = np.interp(self.value, [-1, 1], [self.Steering_Min_Tension, self.Steering_Max_Tension])
            self.logger.debug(f'Direccion {VehicleState.direccion}  Real {VehicleState.direccion_real}')
            error = VehicleState.direccion - VehicleState.direccion_real
            self.logger.debug(f'Send: {error}  abs value: {VehicleState.direccion}')
            self.communications.CAN2.add_to_queue([
                make_can_frame(node=self.cobid, index=0x6071, sub_index=0, data=tension)])
        else:
            if VehicleState.b_direccion:
                self.set_magnet_disable()

    def set_enable(self):
        self.logger.debug('Sending enable sequence')
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x6060, data=10),
            make_can_frame(node=self.cobid, index=0x6040, data=0),
            make_can_frame(node=self.cobid, index=0x6040, data=6),
            make_can_frame(node=self.cobid, index=0x6040, data=15)
        ])

    # TODO: Falta el enable/disable
    def set_magnet_enable(self):
        self.logger.debug('Magnet enable')
        if self.communications.AIO.is_connected():
            VehicleState.b_direccion = True
            self.communications.AIO.put_queue([[3.0, 3], [3.0, 3], [3.0, 3]])

    def set_magnet_disable(self):
        self.logger.debug('Disable')
        if self.communications.AIO.is_connected():
            VehicleState.b_direccion = False
            self.communications.AIO.put_queue([[0, 3], [0, 3], [0, 3]])

    def shutdown(self):
        self.logger.warn('Shutdown')
        self.shutdown_flag = True
        self.set_magnet_disable()
        self.timer.cancel()
