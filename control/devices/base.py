from multiprocessing.queues import Queue
from traceback import format_exc

from geometry_msgs.msg import Point
from rclpy.logging import get_logger
from std_msgs.msg import Bool, Float64


class VehicleState(object):
    """
    Class to store the vehicle state
    """
    id_platforma: str

    velocidad_real: float
    direccion_real: float
    freno_real: float
    marcha_real: str

    # Estado del sistema
    b_acelerador: bool
    b_freno: bool
    b_direccion: bool

    # Request
    direccion: float
    velocidad: float

    b_velocidad_request: bool
    b_direccion_request: bool
    b_marchas_request: bool
    parada_emergencia: bool

    def __new__(cls):
        VehicleState.id_platforma = ''
        VehicleState.velocidad_real = 0.
        VehicleState.direccion_real = 0.
        VehicleState.freno_real = 0.
        VehicleState.marcha_real = 'N'

        # Estado del sistema
        VehicleState.b_acelerador = False
        VehicleState.b_freno = False
        VehicleState.b_direccion = False

        # Request
        VehicleState.direccion = 0.
        VehicleState.velocidad = 0.
        VehicleState.b_velocidad_request = False
        VehicleState.b_direccion_request = False
        VehicleState.parada_emergencia = False


if __name__ == '__main__':
    VehicleState()
