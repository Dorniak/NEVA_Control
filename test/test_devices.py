import string
from queue import Queue
from random import randint, choice
from traceback import format_exc

import pytest
from std_msgs.msg import Float64

from control.devices.base import VehicleState, BaseDevice
from control.actuators.brake import Brake
from control.actuators.steering import Steering


def random_string(string_length=6):
    """Generate a random string of letters and digits """
    valid_chars = string.ascii_letters + string.digits + '-' + '_'
    return ''.join(choice(valid_chars) for _ in range(string_length))


@pytest.mark.parametrize('cobids', [randint(1, 127), randint(1, 127), randint(1, 127)])
@pytest.mark.parametrize('names', [random_string(8), random_string(5), random_string(10)])
@pytest.mark.parametrize('device_ranges', [[-360, 360], [0, 154], [1, 5.5]])
@pytest.mark.parametrize('cmd_ranges', [[-1, 1], [0, 1]])
def test_base_init_correct_values(cobids, names, device_ranges, cmd_ranges):
    q = Queue()
    device = BaseDevice(name=names, cobid=cobids, device_range=device_ranges, queue=q, cmd_range=cmd_ranges)
    assert device.name == names
    assert type(device.name) is str
    assert device.cobid == cobids
    assert type(device.cobid) is int
    assert device.device_range == tuple(device_ranges)
    assert type(device.device_range) is tuple
    assert device.cmd_range == tuple(cmd_ranges)
    assert type(device.cmd_range) is tuple
    assert device.queue_CAN2 is q


@pytest.mark.parametrize('cobids', [randint(127, 500), randint(-127, 0), 0, 5])
@pytest.mark.parametrize('names', [random_string(8)])
@pytest.mark.parametrize('device_ranges', [[360, -360], [0, 154], [1, 5.5]])
@pytest.mark.parametrize('cmd_ranges', [(1, -1), randint(1, 8), randint(0, 5)])
def test_base_init_incorrect_values(cobids, names, device_ranges, cmd_ranges):
    q = Queue()
    with pytest.raises(expected_exception=ValueError):
        _ = BaseDevice(name=names, cobid=cobids, device_range=device_ranges, queue=q, cmd_range=cmd_ranges)


# noinspection PyBroadException
def test_brake_subclass():
    brake = Brake(cobid=2, device_range=[0, 1], queue=Queue(), cmd_range=[-1, 1])
    assert brake.name == 'brake'
    try:
        brake._enable()
        assert VehicleState.brake_enabled is True
        brake._disable()
        assert VehicleState.brake_enabled is False
    except Exception:
        pytest.fail(format_exc())


@pytest.mark.parametrize('v', [0.1, 0.5, 1.])
def test_brake_callback_correct(v):
    brake = Brake(cobid=2, device_range=[0, 180], queue=Queue(), cmd_range=[0, 1])
    brake._disable()
    brake.callback(Float64(data=v))
    assert VehicleState.brake != v
    brake._enable()
    brake.callback(Float64(data=v))
    assert VehicleState.brake == v


@pytest.mark.parametrize('v', [-3., 8., 7., 1.])
def test_brake_callback_incorrect(v):
    brake = Brake(cobid=2, device_range=[0, 180], queue=Queue(), cmd_range=[0, 1])
    brake.callback(Float64(data=v))
    assert 0 <= VehicleState.brake <= 1


@pytest.mark.parametrize('interp', [(0., 0.), (0.5, 90.), (1., 180.)])
def test_brake_callback_interp(interp: tuple):
    brake = Brake(cobid=2, device_range=[0, 180], queue=Queue(), cmd_range=[0, 1])
    brake._enable()
    brake.callback(Float64(data=interp[0]))
    assert VehicleState.brake_real == interp[1]


# noinspection PyBroadException
def test_steering_subclass():
    steering = Steering(cobid=2, device_range=[-360, 360], queue=Queue(), cmd_range=[-1, 1])
    assert steering.name == 'steering'
    try:
        steering._enable()
        assert VehicleState.steering_enabled is True
        steering._disable()
        assert VehicleState.steering_enabled is False
    except Exception:
        pytest.fail(format_exc())


@pytest.mark.parametrize('v', [0.1, 0.5, 1.])
def test_steering_callback_correct(v):
    steering = Steering(cobid=2, device_range=[-360, 360], queue=Queue(), cmd_range=[-1, 1])
    steering._disable()
    steering.callback(Float64(data=v))
    assert VehicleState.steering != v
