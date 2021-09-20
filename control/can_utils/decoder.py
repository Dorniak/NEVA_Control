from struct import unpack

# https://docs.python.org/3.6/library/struct.html
import std_msgs.msg

FMT_DICT = {
    (True, 2): '>h',
    (False, 2): '<H',
    (True, 4): '<i',
    (False, 4): '<I',
    (True, 4): '<l',
    (False, 4): '<L',
    (True, 8): '<q',
    (False, 8): '<Q',
    (True, 8): '<d',
    (True, 4): '<f'
}
from binascii import hexlify


class CANFilter(object):
    __slots__ = (
    'ns', 'cobid', 'signed', 'resolution', 'length', 'fmt', 'publisher', 'value', 'name', 'nbytes', 'ros_type')

    def __init__(self, ns, cobid, signed, name, nbytes, ros_type, resolution=1.):
        self.ns = ns
        self.name = name
        self.cobid = cobid
        self.resolution = resolution
        self.signed = signed
        self.publisher = None
        self.ros_type = getattr(std_msgs.msg, ros_type)
        self.value = self.ros_type()
        self.nbytes = nbytes
        self.length = (self.nbytes[1] - self.nbytes[0]) + 1
        self.fmt = FMT_DICT[(self.signed, self.length)]

    def __repr__(self):
        return f"{self.name}: {hex(self.cobid)} | {self.ns} | {self.resolution}"

    def decode(self, msg: bytes):
        self.value.data = unpack(self.fmt, msg[self.nbytes[0]:self.nbytes[1] + 1])[0] * self.resolution
        self.publish()

    def publish(self):
        if self.publisher:
            self.publisher.publish(self.value)
        else:
            print(f"Decoder {self.name} has no publisher")


if __name__ == '__main__':
    import yaml
    from pprint import pprint

    with open('../../conf/packages.yaml') as f:
        data = yaml.safe_load(f)

    pprint(data)
    for item in data:
        a = CANFilter(cobid=item, **data[item])
        print(a)
