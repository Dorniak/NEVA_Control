from rclpy.node import Node
from rclpy.logging import get_logger

from control.can_utils.common import make_can_frame
from control.devices.base import VehicleState
from control.devices.communications import Communications
import numpy as np


class Steering:

    def __init__(self, cobid, node: Node, communications: Communications, log_level=10):
        self.name = 'Steering'
        self.communications = communications
        self.cobid = cobid
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.node = node
        self.pid = PIDF(kp=0.3, ti=0.2, td=0.1, anti_wind_up=0.4, pro_wind_up=0)
        self.value = 0
        self.timer = self.node.create_timer(0.1, self.sender)
        self.timer_pid = self.node.create_timer(1 / 50., self.pid_timer)

    def pid_timer(self):
        taN = np.interp(VehicleState.direccion_real, [-630., 630.], [-1., 1.])
        caN = np.interp(VehicleState.direccion, [-630., 630.], [-1., 1.])
        self.value = -self.pid.calcValue(target_value=taN, current_value=caN)

    def sender(self):
        if VehicleState.b_direccion_request:
            if not VehicleState.b_direccion:
                self.logger.debug('Try to enable')
                self.set_enable()
                self.set_magnet_enable()

            tension = np.interp(self.value, [-1, 1], [Steering_Min_Tension, Steering_Max_Tension])
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
        self.set_magnet_disable()
        self.timer.cancel()
        self.timer_pid.cancel()


class PIDF(object):
    kp = None
    td = None
    ti = None
    prev_i = 0
    prev_d = 0
    prev_value = 0
    prev_target = 0
    _antiWindUp = None
    error = 0
    n = 8.0
    h = 0.1 / 5  # TODO DEPENDE DE LA FRECUENCIA

    def __init__(self, kp, td, ti, anti_wind_up, pro_wind_up=0):
        self.kp = kp
        self.td = td
        self.ti = ti
        self._antiWindUp = anti_wind_up
        self._pro_wind_up = pro_wind_up

    def calcValue(self, target_value, current_value, anti_wind_up_change=False, use=(True, True, True)):
        self.error = target_value - current_value

        if use[0] and self.kp != 0:
            p = self.kp * self.error
        else:
            p = 0

        if np.sign(self.prev_target) != np.sign(target_value) and anti_wind_up_change:
            i = 0
            self.prev_i = 0

        if use[1] and self.ti != 0:
            i = self.prev_i + (((self.kp * self.h) / self.ti) * self.error)
            i = self.antiwindup(i=i)
            self.prev_i = i
        else:
            i = 0

        if use[2] and self.td != 0:
            d = ((self.td / (self.td + (self.n * self.h)) * self.prev_d) - (
                    ((self.kp * self.td * self.n) / (self.td + (self.n * self.h))) * (current_value - self.prev_value)))
            self.prev_d = d
        else:
            d = 0

        self.prev_value = current_value

        if p + i + d < -1:
            return -1
        if p + i + d > 1:
            return 1

        self.prev_target = target_value

        return p + i + d

    def calcValue(self, target_value, current_value, kp, td, ti, anti_wind_up_change=False, use=(True, True, True)):
        self.error = target_value - current_value

        if use[0] and self.kp != 0:
            p = kp * self.error
        else:
            p = 0

        if np.sign(self.prev_target) != np.sign(target_value) and anti_wind_up_change:
            i = 0
            self.prev_i = 0

        if use[1] and self.ti != 0:
            i = self.prev_i + (((kp * self.h) / ti) * self.error)
            i = self.antiwindup(i=i)
            self.prev_i = i
        else:
            i = 0

        if use[2] and self.td != 0:
            d = ((td / (td + (self.n * self.h)) * self.prev_d) - (
                    ((kp * td * self.n) / (td + (self.n * self.h))) * (current_value - self.prev_value)))
            self.prev_d = d
        else:
            d = 0

        self.prev_value = current_value

        if p + i + d < -1:
            return -1
        if p + i + d > 1:
            return 1

        self.prev_target = target_value

        return p + i + d

    def antiwindup(self, i):
        if i > self._antiWindUp:
            i = self._antiWindUp
        elif i < -self._antiWindUp:
            i = -self._antiWindUp

        if 0 < i < self._pro_wind_up:
            i = self._pro_wind_up
        elif 0 > i > -self._pro_wind_up:
            i = -self._pro_wind_up

        return i

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.ti = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.td = derivative_gain
