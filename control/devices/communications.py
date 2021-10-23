from control.devices.CAN import CAN


class Communications:
    def __init__(self):
        self.CAN1: CAN = None
        self.CAN2: CAN = None

    def shutdown(self):
        if self.CAN1 is not None:
            self.CAN1.shutdown()
        if self.CAN2 is not None:
            self.CAN2.shutdown()
