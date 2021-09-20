from control.devices.CAN import CAN
from control.devices.IO import AIO, DIO


class Communications:
    def __init__(self):
        self.CAN1: CAN = None
        self.CAN2: CAN = None
        self.AIO: AIO = None
        self.DIO: DIO = None

    def shutdown(self):
        if self.CAN1 is not None:
            self.CAN1.shutdown()
        if self.CAN2 is not None:
            self.CAN2.shutdown()
        if self.AIO is not None:
            self.AIO.shutdown()
        if self.DIO is not None:
            self.DIO.shutdown()
