#!/usr/bin/env python3
# coding=utf-8
##########  DEVICE EXCEPTIONS  ########## PAG 58-59 ros-can/doc/EPOS-Firmware-Specification-En.pdf


class GenericError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x1000""",


class OverCurrentError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x2310""",


class OverVoltageError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x3210""",


class UnderVoltage(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x3220""",


class OverTemperature(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x4210""",


class SupplyVoltageTooLow(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x5113""",


class InternalSoftwareError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x6100""",


class SoftwareParameterError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x6320""",


class SensorPositionError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x7320""",


class CanOverrunErrorObjectsLost(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x8110""",


class CanOverrunError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x8111""",


class CanPassiveModeError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x8120""",


class CanLifeGuardError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x8130""",


class CanTransmitCobIdCollision(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x8150""",


class CanBusOff(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x81FD""",


class CanRxQueueOverrun(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x81FE""",


class CanTxQueueOverrun(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x81FF""",


class CanPdoLengthError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x8210""",


class FollowingError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0x8611""",


class HallSensorError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF01""",


class IndexProcessingError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF02""",


class EncoderResolutionError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF03""",


class HallsensorNotFoundError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF04""",


class NegativeLimitError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF06""",


class PositiveLimitError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF07""",


class HallAngleDetectionError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF08""",


class SoftwarePositionLimitError(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF09""",


class PositionSensorBreach(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF0A""",


class SystemOverloaded(Exception):
    def __init__(self):
        self.args = ()
        self.args += f"""0xFF0B""",


DEVICE_ERROR_CODE = {0x1000: GenericError, 0x2310: OverCurrentError, 0x3210: OverVoltageError, 0x3220: UnderVoltage,
                     0x4210: OverTemperature, 0x5113: SupplyVoltageTooLow, 0x6100: InternalSoftwareError,
                     0x6320: SoftwareParameterError, 0x7320: SensorPositionError, 0x8110: CanOverrunErrorObjectsLost,
                     0x8111: CanOverrunError, 0x8120: CanPassiveModeError, 0x8130: CanLifeGuardError,
                     0x8150: CanTransmitCobIdCollision, 0x81FD: CanBusOff, 0x81FE: CanRxQueueOverrun,
                     0x81FF: CanTxQueueOverrun, 0x8210: CanPdoLengthError, 0x8611: FollowingError,
                     0xFF01: HallSensorError, 0xFF02: IndexProcessingError, 0xFF03: EncoderResolutionError,
                     0xFF04: HallsensorNotFoundError, 0xFF06: NegativeLimitError, 0xFF07: PositiveLimitError,
                     0xFF08: HallAngleDetectionError, 0xFF09: SoftwarePositionLimitError, 0xFF0A: PositionSensorBreach,
                     0xFF0B: SystemOverloaded}
