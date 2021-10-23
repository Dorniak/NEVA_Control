from rclpy.node import Node
from rclpy.logging import get_logger

import queue
import threading
from math import modf
import time
from control.devices.connection import Connection


class AIO:
    def __init__(self, name: str = 'Unknown AIO', node: Node = None, ip='192.168.0.7', port: int = 4600, log_level=10):
        self.name = name
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.logger.info(f'Init {name} in {ip}:{port}')
        self.queue = queue.Queue()
        self.node = node
        #self.ip = ip
        #self.port = port
        self.shutdown_flag = False
        self.connection = Connection(name='IO tcp', mode='tcp', ip=ip, port=port, deco_function=self.decode,
                                     log_level=log_level)
        self.timer = threading.Thread(target=self.sender, daemon=True, name='Sender AIO')
        self.timer.start()

    def decode(self, data):
        while not self.shutdown_flag:
            try:
                self.logger.debug(f'Read: {data.decode()}')
            except Exception as e:
                self.logger.error(f'Read error: {e}')

    def put_queue(self, data: list):
        if not self.shutdown_flag:
            if len(self.queue.queue) > 100:
                for _ in range(len(data)):
                    self.queue.get()
                self.logger.error(f'{len(data)}')
            [self.queue.put(d) for d in data]
            self.logger.debug(f'Q len: {len(self.queue.queue)}')

    def create_msg(self, AO2, canal):
        pdec, pent = modf(AO2)

        signo = "+" if AO2 >= 0 else "-"
        valor = signo + str(int(pent)).zfill(2) + "." + str(int(pdec * 1000)).zfill(3)
        msg = "#01" + str(canal) + valor + "\r\n"
        return bytes(msg, "ASCII")

    def sender(self):
        while not self.shutdown_flag or len(self.queue.queue) > 0:
            time.sleep(1/250)
            if self.is_connected():
                try:
                    data = self.queue.get_nowait()
                    self.logger.debug(f'{data}')
                    msg = self.create_msg(data[0], data[1])
                    self.logger.debug(f'{msg}')
                except queue.Empty:
                    pass
                except Exception as e:
                    self.logger.error(f'create_msg exception: {e}')
                else:
                    try:
                        if len(data) >= 3:
                            time.sleep(data[2])
                        self.connection.send(msg)
                        if len(data) >= 4:
                            time.sleep(data[3])
                    except Exception as e:
                        self.logger.error(f'Sender exception: {e}')
                        self.connection.reconnect()

    def is_connected(self):
        return self.connection.connected

    def shutdown(self):
        self.logger.warn('Shutdown')
        self.shutdown_flag = True
        while len(self.queue.queue)>0:
            self.logger.info(f'waiting to close q len:{len(self.queue.queue)}')
            self.sender()
            time.sleep(0.1)
        self.connection.shutdown()


class DIO:
    def __init__(self, name: str = 'Unknown DIO', ip='192.168.0.7', port: int = 4601, log_level=10):
        self.name = name
        self.logger = get_logger(self.name)
        self.logger.set_level(log_level)
        self.logger.info(f'Init {name} in {ip}:{port}')
        # self.ip = ip
        # self.port = port
        self.shutdown_flag = False
        self.time = time.time()
        self.connection = Connection(name='DIO tcp', mode='tcp', ip=ip, port=port, deco_function=self.decode,
                                     log_level=log_level)
    
    def decode(self, data):
        pass

    def SetDO0(self, b: bool):
        try:
            DIO0 = "07"
            DIO0 = DIO0 + "0\r\n" if b else DIO0 + "1\r\n"
            self.logger.debug(f'Set DO0 {DIO0}')
            self.connection.send(bytes(DIO0, "ASCII"))
            self.logger.debug(f'Sent 0')
        except Exception:
            self.connection.reconnect()

    def SetDO1(self, b: bool):
        try:
            DIO1 = "08"
            DIO1 = DIO1 + "0\r\n" if b else DIO1 + "1\r\n"
            self.logger.debug(f'Set DO1 {DIO1}')
            self.connection.send(bytes(DIO1, "ASCII"))
            self.logger.debug(f'Sent 1')
        except Exception:
            self.connection.reconnect()

    def SetDIO(self, level: int):
        if level == 0:
            self.logger.debug('Set level 0')
            self.SetDO0(True)
            self.SetDO1(False)
        elif level == 1:
            self.logger.debug('Set level 1')
            self.SetDO0(False)
            self.SetDO1(False)
        elif level == 2:
            self.logger.debug('Set level 2')
            self.SetDO0(False)
            self.SetDO1(True)

    def is_connected(self):
        return self.connection.connected

    def shutdown(self):
        self.logger.warn('Shutdown')
        self.shutdown_flag = True
        self.connection.shutdown()