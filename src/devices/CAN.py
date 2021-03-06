import queue
import socket
import time

from rclpy.node import Node
from rclpy.logging import get_logger

from src.devices.base import VehicleState
from src.devices.connection import Connection
from struct import unpack


def deco_176(data, logger, node):
    value = unpack('>H', data[4:6])[0] * 0.1
    logger.debug(f'Velocidad real recibida {value}')
    VehicleState.velocidad_real = value


def deco_002(data, logger, node):
    data = unpack('>h', data[5:7])[0]
    value = (data * 0.1)
    logger.debug(f'Dirección real recibida {value}')
    VehicleState.direccion_real = value


def deco_1CB(data, logger, node):
    value = unpack('>H', data[6:8])[0] * 0.001527
    VehicleState.freno_real = value
    logger.debug(f'Posicion freno recibida {value}')


gear_decoder = {
    1: 'P',
    2: 'R',
    3: 'N',
    4: 'D',
    7: 'B',
}


def deco_421(data, logger, node):
    try:
        value = data[4] >> 3
        VehicleState.marcha_real = gear_decoder.get(value, 'N')
        logger.debug(f'Marcha recibida {VehicleState.marcha_real}')
    except Exception as e:
        print(f'Exception decoding gear {e}')


def deco_581(data, logger, node):
    try:
        index = unpack('<H', data[5:7])
        if index == 0x6041:
            status = unpack('<H', data[8:10])
            if status == 0x27 or status == 0x07:
                VehicleState.status_brake = True
            else:
                VehicleState.status_brake = False
        logger.debug(f'Status brake {VehicleState.status_brake}')
    except Exception as e:
        print(f'Exception status brake {e}')


def deco_582(data, logger, node):
    try:
        index = unpack('<H', data[5:7])
        if index == 0x6041:
            status = unpack('<H', data[8:10])
            if status == 0x27 or status == 0x07:
                VehicleState.status_steering = True
            else:
                VehicleState.status_steering = False
        logger.debug(f'Status brake {VehicleState.status_steering}')
    except Exception as e:
        print(f'Exception status steering {e}')


dict_decoder = {  # 0x581: deco_581,
    0x002: deco_002,
    0x1CB: deco_1CB,
    0x176: deco_176,
    0x421: deco_421,
    0x581: deco_581,
    0x582: deco_582,
}


class CAN:
    def __init__(self, name: str = 'Unknown CAN', node: Node = None, ip='192.168.0.6', port: int = 10001,
                 write_timer_period: int = 10,
                 connection_mode: str = 'tcp', timeout: int = 5, extend=False, log_level=10):
        self.name = name
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.logger.info(f'Init {name} with {ip}:{port} extended={extend}')
        self.queue = queue.Queue()
        self.node = node
        # self.ip = ip
        # self.port = port
        self.socket: socket = None
        self.connection_mode = connection_mode  # tcp/udp
        self.timeout = timeout
        self.connection = Connection(name=f'Connection {ip}:{port}', mode=connection_mode, ip=ip, port=port,
                                     deco_function=self.decode_can, log_level=log_level)
        self.timer_write = self.node.create_timer(1 / write_timer_period, self.write)
        self.extend = extend
        self.shutdown_flag = False

    def add_to_queue(self, frames: list) -> None:
        """
        Add a list of CAN frames to be sent
        :param frames: List of CAN frames (13 bytes)
        :return:
        """
        if self.is_connected():
            if len(self.queue.queue) > 100:
                for _ in frames:
                    self.queue.get()
            [self.queue.put(frame) for frame in frames]
            self.logger.debug(f'Q len: {len(self.queue.queue)}')

    def write(self):
        # Write
        if self.is_connected():
            try:
                msg = self.queue.get_nowait()
                assert len(msg) == 13
            except AssertionError:
                self.logger.error('Error in message length')
            except queue.Empty:
                pass
            except Exception as e:
                self.logger.error(f'Not managed exception: {e}')
            else:
                self.connection.send(msg)

    def decode_can(self, data):
        if data is not None:
            # Decode
            try:
                if self.extend:
                    # In case of CAN 1
                    COBID = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3]
                else:
                    COBID = (data[2] << 8) + data[3]
                    # In case of CAN 2
            except Exception as e:
                self.logger.info(f'Exception read: {data} {e}')
            else:
                try:
                    if COBID in dict_decoder.keys():
                        if not (COBID == 0x316):
                            dict_decoder[COBID](data, self.logger, self.node)
                    else:
                        self.logger.debug(f'Received package not managed in: {hex(COBID)}')
                except Exception as e:
                    self.logger.error(f'Error in decoding COBID: {hex(COBID)} {e}')

    def is_connected(self):
        return self.connection.connected

    def shutdown(self):
        self.logger.warn('Shutdown')
        self.shutdown_flag = True
        while len(self.queue.queue) > 0:
            self.logger.info(f'waiting to close q len:{len(self.queue.queue)}')
            self.write()
            time.sleep(0.1)
        self.connection.shutdown()
