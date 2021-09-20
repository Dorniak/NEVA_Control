from traceback import format_exc

from control.devices.base import VehicleState
from control.actuators.brake import Brake
from control.actuators.steering import Steering
from control.actuators.throttle import Throttle
from control.devices.communications import Communications
from control.devices.CAN import CAN
from control.devices.IO import AIO, DIO
from control.devices.pid import PID

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy

from std_msgs.msg import Float64, Bool, String, UInt8
from geometry_msgs.msg import Vector3
from mdef_a2sat.msg import RespConduccion, PetConduccion, ElemAlarmas
from numpy import interp
import time


# noinspection PyBroadException
class Control(Node):
    def __init__(self, name: str = 'unknown_control'):
        super().__init__(node_name=name, namespace='control', start_parameter_services=True,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True)

        # Logging configuration
        self.logger = self.get_logger()
        self._log_level: Parameter = self.get_parameter_or('log_level', Parameter(name='log_level', value=10))
        self.logger.set_level(self._log_level.value)


        VehicleState()
        VehicleState.id_platforma = self.get_parameter_or('id_platform',
                                                          Parameter(name='id_platform', value='Unknown vehicle')).value

        self.IantA = 0.
        self.DantA = 0.
        self.IantF = 0.
        self.DantF = 0.
        self.Vant = 0.

        params = self.get_parameters_by_prefix('pid')

        self.pid_brake = PID(kp=params['b_kp'].value, ti=params['b_ti'].value, td=params['b_td'].value,
                             anti_wind_up=0.1)

        self.pid_throttle = PID(kp=params['t_kp'].value, ti=params['t_ti'].value, td=params['t_td'].value,
                                anti_wind_up=0.2)

        self.logger.info('Start control')

        # Init comunications
        self.comunications = Communications()
        self.logger.info('Connection CAN 1')
        params = self.get_parameters_by_prefix('CAN_1')
        self.comunications.CAN1 = CAN(name='CAN 1', node=self, ip=params['ip'].value, port=params['port'].value,
                                      extend=params['extend'].value, log_level=params['log_level'].value)

        self.logger.info('Connection CAN 2')
        params = self.get_parameters_by_prefix('CAN_2')
        self.comunications.CAN2 = CAN(name='CAN 2', node=self, ip=params['ip'].value, port=params['port'].value,
                                      extend=params['extend'].value, log_level=params['log_level'].value,
                                      write_timer_period=100)

        self.logger.info('Connection AIO')
        params = self.get_parameters_by_prefix('AIO')
        self.comunications.AIO = AIO(name='AIO', node=self, ip=params['ip'].value, port=params['port'].value,
                                     log_level=params['log_level'].value)

        self.logger.info('Connection DIO')
        params = self.get_parameters_by_prefix('DIO')
        self.comunications.DIO = DIO(name='DIO', ip=params['ip'].value, port=params['port'].value,
                                     log_level=params['log_level'].value)

        # Subscriber vehicle request
        self.create_subscription(msg_type=Float64, topic='/Vamtac/direccion',
                                 callback=self.sub_direccion, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Float64, topic='/Vamtac/velocidad',
                                 callback=self.sub_velocidad, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Bool, topic='/Vamtac/b_velocidad',
                                 callback=self.sub_b_velocidad, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Bool, topic='/Vamtac/b_direccion',
                                 callback=self.sub_b_direccion, qos_profile=HistoryPolicy.KEEP_LAST)

        # Configuring devices
        params = self.get_parameters_by_prefix('steering')
        if params['on'].value:
            self.logger.info('Configuring steering device')
            self.steering = Steering(cobid=params['cobid'].value, node=self, communications=self.comunications,
                                     log_level=params['log_level'].value)
        else:
            self.logger.warn('Steering device is not configured')
            self.steering = None

        params = self.get_parameters_by_prefix('throttle')
        if params['on'].value:
            self.logger.info('Configuring throttle device')
            self.throttle = Throttle(communications=self.comunications, log_level=params['log_level'].value)
        else:
            self.logger.warn('Throttle device is not configured')
            self.throttle = None

        params = self.get_parameters_by_prefix('brake')
        if params['on'].value:
            self.logger.info('Configuring brake device')
            self.brake = Brake(cobid=params['cobid'].value, communications=self.comunications,
                               log_level=params['log_level'].value)
            # self.brake.calibrate()
        else:
            self.logger.warn('Brake device is not configured')
            self.brake = None

        # # Publishers
        # self.pub_RespConduccion = self.create_publisher(msg_type=RespConduccion, topic='/TeleOperacion/Resp',
        #                                                 qos_profile=HistoryPolicy.KEEP_LAST)

        # Timers
        # self.logger.info('Publisher Resp Conduccion working')
        # self.timer_RespConduccion = self.create_timer(1 / 10, self.publish_RespConduccion)

        # self.logger.info('Keep alive timer working')
        # self.timer_motor = self.create_timer(2, self.motors_keep_alive)

        if self.brake is not None and self.throttle is not None:
            self.logger.info('Speed control working')
            self.timer_speed_control = self.create_timer(1 / 20, self.control_speed)
        else:
            self.logger.warn(
                f'Speed control disabled Throttle: {self.throttle is not None} Brake: {self.brake is not None}')

    def sub_direccion(self, data):
        VehicleState.direccion = max(min(data.data, 600), -600)

    def sub_velocidad(self, data):
        VehicleState.velocidad = data.data

    def sub_marchas(self, data):
        VehicleState.marchas = data.data

    def sub_b_velocidad(self, data):
        if self.brake is not None:
            if data.data:
                VehicleState.b_freno = True
            else:
                VehicleState.b_freno = False
                self.brake.raise_maximum()
        if self.throttle is not None:
            if data.data:
                if not self.throttle.is_enable:
                    self.throttle.set_enable()
            else:
                if self.throttle.is_enable:
                    self.throttle.set_disable()
                if self.brake is not None:
                    self.brake.set_pedal_pressure(0)

        VehicleState.b_velocidad_request = data.data

    def sub_b_direccion(self, data):
        VehicleState.b_direccion_request = data.data

    # Subscriber real

    # def sub_pet_conduccion(self, data):
    #     VehicleState.direccion = - max(min(data.direccion, 600), -600)
    #     VehicleState.velocidad = data.velocidad
    #     VehicleState.marchas = data.marchas
    #
    #     if VehicleState.parada_emergencia_request != data.desact_parada_emergencia and self.brake is not None:
    #         if data.desact_parada_emergencia:
    #             self.brake.emergency_brake()
    #         else:
    #             self.brake.end_emergency_brake()
    #
    #     VehicleState.parada_emergencia_request = data.desact_parada_emergencia
    #
    #     VehicleState.override_request = data.override
    #     VehicleState.obstaculo_conectado = data.override
    #
    #     if self.brake is not None and self.brake.calibrated:
    #         if data.b_velocidad:
    #             VehicleState.b_freno = True
    #         else:
    #             VehicleState.b_freno = False
    #             self.brake.raise_maximum()
    #     if self.throttle is not None:
    #         if data.b_velocidad:
    #             if not self.throttle.is_enable:
    #                 self.throttle.set_enable()
    #         else:
    #             if self.throttle.is_enable:
    #                 self.throttle.set_disable()
    #             if not VehicleState.parada_emergencia and self.brake.calibrated:
    #                 if self.brake is not None:
    #                     self.brake.set_pedal_pressure(0)
    #
    #     VehicleState.b_velocidad_request = data.b_velocidad
    #
    #     VehicleState.b_direccion_request = data.b_direccion
    #
    #     VehicleState.b_marchas = data.b_marchas
    #     VehicleState.b_marchas_request = data.b_marchas

    # def motors_keep_alive(self):
    #     try:
    #         if self.comunications.CAN2 is not None and self.comunications.CAN2.is_connected():
    #             if self.steering is not None:
    #                 self.steering.send_motor_enable()
    #                 if VehicleState.b_direccion and VehicleState.ContEnableSteering >= -4:
    #                     pass
    #                 else:
    #                     if not (VehicleState.b_direccion and VehicleState.ContEnableSteering >= -1):
    #                         self.pub_Alarma.publish(
    #                             ElemAlarmas(
    #                                 id_alarma=40,
    #                                 estado=True,
    #                                 severidad=3,
    #                                 t_alarma=int(time.time()))
    #                         )
    #                         self.steering.set_enable()
    #                         self.steering.send_motor_enable()
    #             if self.brake is not None and self.brake.calibrated:
    #                 self.brake.send_motor_enable()
    #                 if VehicleState.b_freno and VehicleState.ContEnableBrake >= -4:
    #                     pass
    #                 else:
    #                     if not (VehicleState.b_freno and VehicleState.ContEnableBrake >= -1):
    #                         self.pub_Alarma.publish(
    #                             ElemAlarmas(
    #                                 id_alarma=42,
    #                                 estado=True,
    #                                 severidad=3,
    #                                 t_alarma=int(time.time()))
    #                         )
    #                         self.brake.set_enable()
    #                         self.brake.send_motor_enable()
    #
    #     except Exception as e:
    #         self.logger.error(f'Exception in keep alive {e}')

    # def _position_sensor_callback(self, value: Float64):
    #     VehicleState.steering = value.data

    def control_speed(self):
        if self.brake is not None and self.brake.calibrated:
            try:
                if VehicleState.b_velocidad_request:
                    if not self.throttle.is_enable:
                        self.throttle.set_enable()
                    if not VehicleState.parada_emergencia:
                        VehicleState.parada_emergencia = False
                        if VehicleState.velocidad <= 0.1 and VehicleState.velocidad_real <= 2:  # Parking Mode
                            self.brake.drop_maximum()
                            self.throttle.raise_maximum()
                            self.logger.debug("Parking mode")
                        else:
                            params = self.get_parameters_by_prefix('pid')
                            self.pid_brake.Kp = params['b_kp'].value
                            self.pid_brake.Ti = params['b_ti'].value
                            self.pid_brake.Td = params['b_td'].value

                            self.pid_throttle.Kp = params['t_kp'].value
                            self.pid_throttle.Ti = params['t_ti'].value
                            self.pid_throttle.Td = params['t_td'].value

                            target_speed = interp(VehicleState.velocidad, [0., 30.], [0., 1.])
                            current_speed = interp(VehicleState.velocidad_real, [0., 30.], [0., 1.])

                            throttle = max(min(self.pid_throttle.calcValue(target_speed, current_speed), 1), 0)
                            brake = max(min(-self.pid_brake.calcValue(target_speed, current_speed), 1), 0)
                            self.logger.debug(f"Throttle: {throttle} , Brake: {brake}")
                            if (brake > 0 and (VehicleState.velocidad_real - VehicleState.velocidad)>2) or VehicleState.velocidad ==0:
                                self.throttle.raise_maximum()
                                self.brake.set_pedal_pressure(brake)
                            else:
                                self.throttle.set_pedal_pressure(throttle)
                                self.brake.raise_maximum()
                    else:
                        self.throttle.raise_maximum()
                        VehicleState.parada_emergencia = True
                else:
                    if self.throttle.is_enable:
                        self.throttle.set_disable()
                    self.brake.raise_maximum()
            except Exception as e:
                self.logger.error(f'Exception in speed control {e}')
        else:
            self.logger.debug('Brake not calibrated yet')

    def shutdown(self):
        try:
            self.timer_motor.cancel()
        except Exception:
            pass
        try:
            self.timer_speed_control.cancel()
        except Exception:
            pass
        try:
            self.timer_RespConduccion.cancel()
        except Exception:
            pass
        if self.brake is not None:
            self.brake.raise_maximum()
            self.brake.shutdown = True
        if self.steering is not None:
            self.steering.shutdown()
        if self.gears is not None:
            self.gears.shutdown()
        if self.throttle is not None:
            self.throttle.shutdown()
        self.comunications.shutdown()


def main(args=None):
    rclpy.init(args=args)
    manager = None
    try:
        manager = Control(name='NEVA_Control')
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('Keyboard interrupt')
        manager.shutdown()
        manager.destroy_node()
    except Exception:
        print(format_exc())


if __name__ == '__main__':
    main()
