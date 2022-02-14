from traceback import format_exc

from src.devices.base import VehicleState
from src.actuators.brake import Brake
from src.actuators.steering import Steering
from src.actuators.throttle import Throttle
from src.devices.communications import Communications
from src.devices.CAN import CAN
from src.devices.pid import PID

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy

from std_msgs.msg import Float64, Bool, String, UInt8
from neva_msg.msg import Status
from numpy import interp


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

        # Subscriber vehicle request
        self.create_subscription(msg_type=Float64, topic='/NEVA/direccion',
                                 callback=self.sub_direccion, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Float64, topic='/NEVA/velocidad',
                                 callback=self.sub_velocidad, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Bool, topic='/NEVA/b_velocidad',
                                 callback=self.sub_b_velocidad, qos_profile=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_type=Bool, topic='/NEVA/b_direccion',
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
            self.brake = Brake(cobid=params['cobid'].value, node=self, communications=self.comunications,
                               log_level=params['log_level'].value)
            # self.brake.calibrate()
        else:
            self.logger.warn('Brake device is not configured')
            self.brake = None

        # # Publishers
        self.pub_Status = self.create_publisher(msg_type=Status, topic='/NEVA/status',
                                                qos_profile=HistoryPolicy.KEEP_LAST)

        # Timers
        # self.logger.info('Publisher Resp Conduccion working')
        self.timer_Status = self.create_timer(1 / 10, self.timer_status)

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
                if not VehicleState.b_freno:
                    self.brake.set_enable()
            else:
                self.brake.raise_maximum()
                self.brake.set_disable()
        if self.throttle is not None:
            if data.data:
                if not self.throttle.is_enable:
                    self.throttle.set_enable()
            else:
                if self.throttle.is_enable:
                    self.throttle.set_disable()

        VehicleState.b_velocidad_request = data.data

    def sub_b_direccion(self, data):
        VehicleState.b_direccion_request = data.data

    # Subscriber real

    def timer_status(self):
        self.pub_Status.publish(
            Status(
                id_plataforma=VehicleState.id_platforma,

                velocidad_real=VehicleState.velocidad_real,
                volante_real=VehicleState.direccion_real,
                freno_real=VehicleState.freno_real,
                marcha_real=VehicleState.marcha_real,

                velocidad_target=VehicleState.velocidad,
                volante_target=VehicleState.direccion,

                b_velocidad=VehicleState.b_velocidad_request,
                b_volante=VehicleState.b_direccion_request,

                parada_emergencia_request=VehicleState.parada_emergencia_request,
                parada_emergencia=VehicleState.parada_emergencia,

                CAN_connected=self.comunications.CAN2.is_connected(),
                brake_enabled=VehicleState.status_brake,
                steering_enabled=VehicleState.status_steering,
            )
        )

    def control_speed(self):
        try:
            if VehicleState.b_velocidad_request:
                if not VehicleState.parada_emergencia:
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
                        if (brake > 0 and (
                                VehicleState.velocidad_real - VehicleState.velocidad) > 2) or VehicleState.velocidad == 0:
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
                    self.logger.error('Set disable speed control')
                    self.throttle.set_disable()
                self.brake.raise_maximum()
        except Exception as e:
            self.logger.error(f'Exception in speed control {e}')

    def shutdown(self):
        self.logger.warn('Shutdown')
        try:
            self.timer_speed_control.cancel()
        except Exception:
            pass
        if self.brake is not None:
            self.brake.raise_maximum()
            self.brake.shutdown = True
        if self.steering is not None:
            self.steering.shutdown()
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
    except Exception as e:
        print(f'Exception main {e}')
        # print(format_exc())


if __name__ == '__main__':
    main()
