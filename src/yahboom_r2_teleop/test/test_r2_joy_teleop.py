"""Safety behavior of the joystick node: it must publish zero when disabled
or when joystick input goes stale, because the R2 driver latches /cmd_vel."""

import time

import pytest
import rclpy
from sensor_msgs.msg import Joy

from yahboom_r2_teleop.r2_joy_teleop import R2JoyTeleop

ENABLE_BUTTON = 11
THROTTLE_AXIS = 1
STEERING_AXIS = 2


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def node():
    rclpy.init()
    teleop = R2JoyTeleop()
    teleop._enabled = True
    teleop._enable_debounce = 0.0
    teleop._publisher = Capture()
    yield teleop
    teleop.destroy_node()
    rclpy.shutdown()


def joy(throttle=0.0, enable=False, steering=0.0):
    message = Joy()
    message.axes = [0.0] * 4
    message.axes[THROTTLE_AXIS] = throttle
    message.axes[STEERING_AXIS] = steering
    message.buttons = [0] * 12
    message.buttons[ENABLE_BUTTON] = 1 if enable else 0
    return message


def last_speed(node):
    return node._publisher.messages[-1].linear.x


def test_drives_then_stops_when_joystick_input_goes_stale(node):
    node._on_joy(joy(throttle=1.0))
    node._publish_command()
    assert last_speed(node) > 0.0

    node._last_joy_time = time.monotonic() - 2.0 * node._timeout
    node._publish_command()
    assert last_speed(node) == 0.0


def test_enable_button_edge_disables_and_holds_zero(node):
    node._on_joy(joy(throttle=1.0))
    node._publish_command()
    assert last_speed(node) > 0.0

    node._on_joy(joy(throttle=1.0, enable=True))  # press: disable
    assert not node._enabled
    assert last_speed(node) == 0.0

    # Holding the button is not a new edge, and throttle is ignored.
    node._on_joy(joy(throttle=1.0, enable=True))
    node._on_joy(joy(throttle=1.0))
    node._publish_command()
    assert not node._enabled
    assert last_speed(node) == 0.0


def test_full_steering_stick_sends_firmware_degrees(node):
    # The R2 firmware reads linear.y as degrees / 1000 with +-0.045 = +-45 deg.
    # Full stick must reach the 0.6 rad (34 deg) joint limit, not saturate.
    node._on_joy(joy(throttle=1.0, steering=1.0))
    for _ in range(3):
        node._last_publish_time -= 1.0  # let the 1 rad/s slew limit settle
        node._publish_command()
    assert node._publisher.messages[-1].linear.y == pytest.approx(0.034)
