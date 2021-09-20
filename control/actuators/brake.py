import threading
from time import sleep
from numpy import interp

from rclpy.logging import get_logger

from control.devices.base import VehicleState
from control.can_utils.common import make_can_frame
from control.devices.communications import Communications


class Brake:

    def __init__(self, cobid=2, communications: Communications = None, dev_range=None, log_level=10):
        if dev_range is None:
            dev_range = [0, 150]
        self.name = 'Brake'
        self.communications = communications
        self.cobid = cobid
        self.device_range = dev_range
        self.enable = False
        self.calibrated = True
        self.angle_calibrate = 100
        self.AngleSecurityRangeCalibrate = 20
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.calibration_th = None
        self.shutdown = False

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
        cuentas = int((402000 * value) / 360)
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x607A, data=cuentas),
            make_can_frame(node=self.cobid, index=0x6040, data=0x000F),
            make_can_frame(node=self.cobid, index=0x6040, data=0x003F)
        ])

    def set_motor_rel(self, value):
        cuentas = int((402000 * value) / 360)
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x607A, data=cuentas),
            make_can_frame(node=self.cobid, index=0x6040, data=0x000F),
            make_can_frame(node=self.cobid, index=0x6040, data=0x007F)
        ])

    def set_enable(self):
        self.logger.debug(f'{self.name}: Enable')
        self.communications.CAN2.add_to_queue([
            make_can_frame(node=self.cobid, index=0x6040, data=0x0000),
            make_can_frame(node=self.cobid, index=0x6040, data=0x0000),
            make_can_frame(node=self.cobid, index=0x6040, data=0x0006),
            make_can_frame(node=self.cobid, index=0x6040, data=0x000F),
            make_can_frame(node=self.cobid, index=0x6060, data=0x01)
        ])

    # def calibration_thread(self):
    #     VehicleState.b_freno = True
    #     while not self.communications.CAN2.is_connected():
    #         self.logger.warn('Wait calibration up to CAN2 connected')
    #         sleep(0.2)
    #     sleep(1)
    #     self.logger.info(f'{self.name}: Calibrating')
    #     self.calibrated = False
    #     self.set_enable()
    #     self.send_motor_enable()
    #     sleep(1)
    #     while not self.calibrated and not self.shutdown:
    #         self.send_motor_enable()
    #         if VehicleState.b_freno:
    #             self.set_motor_abs(self.angle_calibrate)
    #             self.angle_calibrate = self.angle_calibrate + 2
    #             self.logger.info(f'Calibrating {self.angle_calibrate}')
    #         else:
    #             self.logger.info(f'Else')
    #             self.set_enable()
    #             self.logger.info(f'Calibrating enable 1')
    #             sleep(2)
    #             self.set_enable()
    #             self.logger.info(f'Calibrating enable 2')
    #             sleep(2)
    #             self.set_motor_abs(self.angle_calibrate - self.AngleSecurityRangeCalibrate)
    #             self.logger.info(f'Calibrating set abs {self.angle_calibrate - self.AngleSecurityRangeCalibrate}')
    #             sleep(1)
    #             cuentas = (172032 * self.device_range[1]) / 360
    #             self.communications.CAN2.add_to_queue([
    #                 make_can_frame(node=self.cobid, index=0x4762, sub_index=1, data=cuentas)])
    #             self.calibrated = True
    #             self.logger.info(f'Calibrated true')
    #             self.raise_maximum()
    #         sleep(0.1)
    #     sleep(3)
    #     self.logger.debug(f'{self.name}: Calibrated')
    #
    # def calibrate(self):
    #     self.calibration_th = threading.Thread(target=self.calibration_thread, daemon=True,
    #                                            name=f'Calibration {self.name}')
    #     self.calibration_th.start()
