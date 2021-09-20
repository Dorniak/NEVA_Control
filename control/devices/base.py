from multiprocessing.queues import Queue
from traceback import format_exc

from geometry_msgs.msg import Point
from rclpy.logging import get_logger
from std_msgs.msg import Bool, Float64


class BaseDevice:
    """
    Base class for CAN devices
    """

    def __init__(self, name: str, cobid: int, device_range: list, queue: Queue, cmd_range: list):
        """

        :param name: Device name
        :param cobid: CAN COBID for the device
        :param device_range: Physical ranges for the devices (min, max)
        :param queue: Queue from CAN communication to send packages
        :param cmd_range: Input ranges from devices (min, max)
        """

        self.name = name
        if not 1 <= cobid <= 127:
            raise ValueError(f"{name}: COBID must be in range [1-127]")

        if type(cobid) is not int:
            raise ValueError(f"{name}: COBID must be of type int [1-127]")

        if type(device_range) is not list or len(device_range) != 2:
            raise ValueError(
                f"{name}: Device range must be tuple with format (min, max) | {device_range}({type(device_range)})")

        if type(cmd_range) is not list or len(cmd_range) != 2:
            raise ValueError(f"{name}: Device range must be tuple with format (min, max) | {cmd_range}")

        # if device_range[0] >= device_range[1]:
        #     raise ValueError(f"{name}: Device range order must be: (min, max) | {device_range}")

        if cmd_range[0] >= cmd_range[1]:
            raise ValueError(f"{name}: Command range order must be: (min, max) | {cmd_range}")

        self.cobid: int = cobid
        self.device_range: tuple = tuple(device_range)
        self._logger = get_logger(f'{self.name}')
        self.command_topic: str = f'{self.name}/command'
        self.enable_topic: str = f'{self.name}/enable'
        self.queue: Queue = queue
        self.cmd_range: tuple = tuple(cmd_range)
        self.publisher = None

    def _enable(self):
        raise NotImplementedError(f"{self.name}: This function must be implemented in class")

    def _disable(self):
        raise NotImplementedError(f"{self.name}: This function must be implemented in class")

    def callback(self, value):
        try:
            if type(value) is Point:
                if not self.cmd_range[0] <= value.y <= self.cmd_range[1]:
                    self._logger.warning(
                        f"{self.name} command value must be float on range {self.cmd_range} | value={value.y}")
            else:
                if not self.cmd_range[0] <= value.data <= self.cmd_range[1]:
                    self._logger.warning(
                        f"{self.name} command value must be float on range {self.cmd_range} | value={value.data}")
        except Exception:
            self._logger.error(format_exc())

    def enable_callback(self, value: Bool):
        if value.data:
            self._logger.info(f"[ON] {self.name} enable request")
            self._enable()
        else:
            self._logger.info(f"[OFF] {self.name} disable request")
            self._disable()

    def log_level(self, level):
        self._logger.set_level(level)


class VehicleState(object):
    """
    Class to store the vehicle state
    """
    id_platforma: str

    velocidad_real: float
    direccion_real: float

    # Estado del sistema
    b_acelerador: bool
    b_freno: bool
    b_direccion: bool

    # Request
    direccion: float
    velocidad: float
    marchas: str

    b_velocidad_request: bool
    b_direccion_request: bool
    b_marchas_request: bool

    ContEnableSteering: int
    ContEnableBrake: int

    # No se usan
    ActualAngleBrakeMotorAbs: float
    CurrentLimitMaxPos: float
    CurrentLimitMax: float
    StatusEncoderMotor: float

    changing: bool

    def __new__(cls):
        VehicleState.id_platforma = ''
        VehicleState.velocidad_real = 0.
        VehicleState.direccion_real = 0.

        # Estado del sistema
        VehicleState.b_acelerador = False
        VehicleState.b_freno = False
        VehicleState.b_direccion = False

        # Request
        VehicleState.direccion = 0.
        VehicleState.velocidad = 0.
        VehicleState.marchas = 'N'
        VehicleState.b_velocidad_request = False
        VehicleState.b_direccion_request = False
        VehicleState.b_marchas_request = False

        VehicleState.ContEnableSteering = 0
        VehicleState.ContEnableBrake = 0

        # No se usan
        VehicleState.ActualAngleBrakeMotorAbs = 0
        VehicleState.CurrentLimitMaxPos = 0
        VehicleState.CurrentLimitMax = 0
        VehicleState.StatusEncoderMotor = 0

        VehicleState.changing = False


    @classmethod
    def print(cls):
        print("Not implemented")


if __name__ == '__main__':
    VehicleState()
    VehicleState.print()
