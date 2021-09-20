from control.can_utils.common import make_can_frame
from enum import IntEnum

QC_FACTOR = 402000 / 360
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


def enable(node: int):
    return [
        make_can_frame(node=node, index=0x6040, data=EPOSCommand.SHUTDOWN),
        make_can_frame(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON_AND_ENABLE),
        make_can_frame(node=node, index=0x6060, data=0x01)]


def disable(node: int):
    return [

    ]


def fault_reset(node: int):
    return [make_can_frame(node=node, index=0x6040, data=0x0080),
            make_can_frame(node, 0x6040, 0, EPOSCommand.SHUTDOWN),
            make_can_frame(node, 0x6040, 0, EPOSCommand.SWITCH_ON_AND_ENABLE), ]


def set_angle_value(node: int, angle: int, absolute=False):
    qc_to_rotate = int(QC_FACTOR * angle)
    if absolute:
        return [
            make_can_frame(node=node, index=0x607A, data=qc_to_rotate),
            make_can_frame(node=node, index=0x6040, data=0x000F),
            make_can_frame(node=node, index=0x6040, data=0x003F)]
    else:
        return [
            make_can_frame(node=node, index=0x607A, data=qc_to_rotate),
            # make_can_frame(node=node, index=0x6040, data=0x000F),
            # make_can_frame(node=node, index=0x6040, data=0x007F)]
            make_can_frame(node=node, index=0x6040, data=0x007F)]


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


def init_device(node: int):
    return [
        make_can_frame(node, 0x6040, 0, 0x0080),
        make_can_frame(node, 0x6060, 0, 0x01),  # operation mode=profile position
        make_can_frame(node, 0x6081, 0, 0x1388),  # rpm speed 1-25000 = 10_000 rpm
        make_can_frame(node, 0x6040, 0, EPOSCommand.SHUTDOWN),  # ????
        make_can_frame(node, 0x6040, 0, EPOSCommand.SWITCH_ON_AND_ENABLE),
        make_can_frame(node, 0x2078, 2, 0x3000)  # DO configuration
    ]
