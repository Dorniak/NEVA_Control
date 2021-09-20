import struct
from binascii import hexlify
from enum import IntEnum

from control.epos.exceptions import DEVICE_ERROR_CODE


class CANOpenProtocols(IntEnum):
    NMT = 0x00
    EMCY = 0x80
    TSTMP = 0x100
    TransmitPDO1 = 0x180
    ReceivePDO1 = 0x200
    TransmitPDO2 = 0x280
    ReceivePDO2 = 0x300
    TransmitPDO3 = 0x380
    ReceivePDO3 = 0x400
    TransmitPDO4 = 0x480
    ReceivePDO4 = 0x500
    TransmitSD0 = 0x580
    ReceiveSDO = 0x600
    Heartbeat = 0x700


CAN_STRUCT = struct.Struct("!I8cc")
CAN_ERROR_STRUCT = struct.Struct("<HB5c")


def make_can_frame(node, index, sub_index=0, data=0, write=True):
    data = int(data)
    frame = bytearray(2)
    protocol_payload = bytearray([6])
    node_payload = bytearray([node])

    if write:
        mode_payload = bytearray([0x22])
    else:
        mode_payload = bytearray([0x40])

    index_payload = bytearray([index & 0xFF, (index >> 8) & 0xFF])
    sub_index_payload = bytearray([sub_index])
    data_payload = bytearray([data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF])
    payload = frame + protocol_payload + node_payload + mode_payload + index_payload + sub_index_payload + data_payload
    payload.append(0x08)

    return payload


class CANFrame(object):
    __slots__ = ('source', '_data_raw', 'cobid', 'data', 'payload_size', 'protocol', 'can_id')

    def __init__(self, source: int = None, data_raw: bytes = None):
        self.source = source
        self.data_raw = data_raw
        self.cobid, self.data, self.payload_size = None, None, None

    @property
    def data_raw(self):
        return self._data_raw

    @data_raw.setter
    def data_raw(self, data):
        if data:
            self._data_raw = data
            self.cobid, *self.data, self.payload_size = CAN_STRUCT.unpack(data)
            self.protocol = self.cobid & 0b1111111111100000
            self.can_id = self.cobid & 0b0000000000011111
            self.data = b''.join(self.data)

    def __str__(self):
        return f"{hex(self.cobid)} | {hexlify(self.data)} |"

    def __repr__(self):
        return str(self)

    def load_error(self):
        error_code, error_register, *_ = CAN_ERROR_STRUCT.unpack(self.data)
        return DEVICE_ERROR_CODE[error_code]


if __name__ == '__main__':
    a = make_can_frame(node=3, index=0x2070, sub_index=0, data=0)
    print(len(a))
