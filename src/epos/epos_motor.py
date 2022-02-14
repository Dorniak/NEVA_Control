from src.can_utils.common import make_can_frame
from enum import IntEnum
import struct
from typing import Tuple
from time import time

# QC_FACTOR = 402000 / 360
#from src.devices.base import VehicleState

QC_FACTOR = 1  # 625000 / 360
DIGITAL_OUTPUT_3 = False
DIGITAL_OUTPUT_4 = False


class EPOSCommand(IntEnum):
    SHUTDOWN = 0b00000110
    SWITCH_ON = 0b00000111
    SWITCH_ON_AND_ENABLE = 0b00001111
    DISABLE_VOLTAGE = 0b00000000
    QUICK_STOP = 0b00000010
    DISABLE_OPERATION = 0b00000111
    ENABLE_OPERATION = 0b00001111
    FAULT_RESET = 0b10000000


status_epos = {
    0: 'Not ready to switch on',
    64: 'Switch on disabled',
    33: 'Ready to switch on',
    35: 'Switched on',
    39: 'Operation enabled',
    7: 'Quick stop active',
    15: 'Fault reaction active',
    8: 'Fault'
}

epos_motor_id = {
    1: 'Brake',
    3: 'Steering'
}


class EPOSMotor:
    def __init__(self, cobid, communications, auto_fault_reset=True):
        self.cobid = cobid
        self.status = 0
        self.position = 0
        self.communications = communications
        self.auto_fault_reset = auto_fault_reset
        self.time_reset = 0

    def set_status(self, status):
        self.status = status
        #try:
        #    print(f'{epos_motor_id[self.cobid]} status: {status_epos[self.status & 0x006F]}')
        #except Exception:
        #    print(f'Error decodificando el status: id {self.cobid} status: {bin(self.status)} {bin(self.status & 0x006F)} {self.status & 0x006F}')
        if self.auto_fault_reset:
            if self.status & 0x08:
                if self.communications.CAN2.is_connected():
                    if time() - self.time_reset > 1:
                        print(f'Fault reset {self.cobid}')
                        self.communications.CAN2.add_to_queue(fault_reset(self.cobid))
                        self.time_reset = time()
            #if self.status & 0x04:
            #if VehicleState.b_direccion and self.communications.CAN2.is_connected():
            #self.communications.CAN2.add_to_queue(fault_reset(self.cobid))


def decoder_maxon(msg) -> Tuple[bytes, int, int, int, bytes]:
    msg = msg[2:-1]  # Se elimina el pad delantero (2) y trasero (1) de la trama
    COBID = struct.unpack('>H', msg[0:2])[0]
    b0 = msg[2:3]
    index = struct.unpack('<H', msg[3:5])[0]
    sub_index = struct.unpack('B', msg[5:6])[0]
    data = msg[6:]
    return msg, COBID, index, sub_index, data


def enable(node: int):
    return [
        make_can_frame(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON_AND_ENABLE)]
    # return [
    #     make_can_frame(node=node, index=0x6040, data=EPOSCommand.SHUTDOWN),
    #     make_can_frame(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON),
    #     make_can_frame(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON_AND_ENABLE),
    #     make_can_frame(node=node, index=0x6060, data=0x01)]


def read_status(node: int):
    return [make_can_frame(node=node, index=0x6041, write=False)]


def read_position(node: int):
    return [make_can_frame(node=node, index=0x6064, write=False)]


def disable(node: int):
    return [
        make_can_frame(node=node, index=0x6040, data=EPOSCommand.DISABLE_OPERATION),
    ]


def fault_reset(node: int):
    return [make_can_frame(node=node, index=0x6040, data=EPOSCommand.FAULT_RESET),
            make_can_frame(node, 0x6040, 0, EPOSCommand.SHUTDOWN),
            make_can_frame(node, 0x6040, 0, EPOSCommand.SWITCH_ON_AND_ENABLE), ]


def set_angle_value(node: int, angle: int, absolute=False, epos4=False):
    qc_to_rotate = int(QC_FACTOR * angle)
    set_angle = []
    if absolute:
        set_angle += [make_can_frame(node=node, index=0x607A, data=qc_to_rotate)]
        if epos4:
            set_angle += [make_can_frame(node=node, index=0x6040, data=0x002F)]
        set_angle += [make_can_frame(node=node, index=0x6040, data=0x003F)]
    else:
        set_angle += [make_can_frame(node=node, index=0x607A, data=qc_to_rotate)]
        if epos4:
            set_angle += [make_can_frame(node=node, index=0x6040, data=0x006F)]
        set_angle += [make_can_frame(node=node, index=0x6040, data=0x007F)]
    return set_angle


def enable_digital_4(node: int):
    global DIGITAL_OUTPUT_4, DIGITAL_OUTPUT_3
    DIGITAL_OUTPUT_4 = True

    if DIGITAL_OUTPUT_3:
        return [make_can_frame(node, 0x2078, 1, 0x3000)]
    else:
        return [make_can_frame(node, 0x2078, 1, 0x1000)]


def disable_digital_4(node: int):
    global DIGITAL_OUTPUT_4, DIGITAL_OUTPUT_3
    DIGITAL_OUTPUT_4 = False

    if DIGITAL_OUTPUT_3:
        return [make_can_frame(node, 0x2078, 1, 0x2000)]
    else:
        return [make_can_frame(node, 0x2078, 1, 0x0000)]


def enable_digital_3(node: int):
    global DIGITAL_OUTPUT_4, DIGITAL_OUTPUT_3
    DIGITAL_OUTPUT_3 = True

    if DIGITAL_OUTPUT_4:
        return [make_can_frame(node, 0x2078, 1, 0x3000)]
    else:
        return [make_can_frame(node, 0x2078, 1, 0x2000)]


def disable_digital_3(node: int):
    global DIGITAL_OUTPUT_4, DIGITAL_OUTPUT_3
    DIGITAL_OUTPUT_3 = False

    if DIGITAL_OUTPUT_4:
        return [make_can_frame(node, 0x2078, 1, 0x1000)]
    else:
        return [make_can_frame(node, 0x2078, 1, 0x0000)]


def init_device(node: int, rpm=0x1388):
    return [
        make_can_frame(node, 0x6040, 0, 0x0080),
        # make_can_frame(node, 0x6060, 0, 0x08),  # operation mode=Cyclic Synchronous Position Mode
        make_can_frame(node, 0x6060, 0, 0x01),  # operation mode=profile position
        make_can_frame(node, 0x6081, 0, rpm),  # rpm speed 1-25000 = 10_000 rpm
        make_can_frame(node, 0x6040, 0, EPOSCommand.SHUTDOWN),  # ????
        make_can_frame(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON_AND_ENABLE),
        # make_can_frame(node, 0x6040, 0, EPOSCommand.SWITCH_ON_AND_ENABLE),
        # make_can_frame(node, 0x2078, 2, 0x3000)  # DO configuration
    ]
