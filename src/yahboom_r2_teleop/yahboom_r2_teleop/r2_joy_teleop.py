#!/usr/bin/env python3
"""Translate the Yahboom R2 gamepad layout into the chassis Twist protocol."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def wheel_angle_to_firmware_steering(wheel_angle_rad):
    """R2 firmware: front wheel angle in whole degrees / 1000 on linear.y
    (Rosmaster_Lib set_car_motion, R2 v_y in [-0.045, 0.045])."""
    return round(math.degrees(wheel_angle_rad)) / 1000.0


class R2JoyTeleop(Node):
    """Safe ROS adaptation of Yahboom's supplied yahboom_joy_R2_v2 example."""

    def __init__(self):
        super().__init__("r2_joy_teleop")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("buzzer_topic", "/Buzzer")
        self.declare_parameter("buzzer_feedback", True)
        self.declare_parameter("throttle_axis", 1)
        self.declare_parameter("steering_axis", 2)
        self.declare_parameter("enable_button", 11)
        self.declare_parameter("enable_debounce_sec", 1.0)
        # Match alc_yahboom_r2_exploration.launch.py bridge tuning.
        self.declare_parameter("min_linear_speed", 0.08)
        self.declare_parameter("max_linear_speed", 0.11)
        # Front wheel angle limit (R2 URDF steer joint), rad.
        self.declare_parameter("max_steering_angle_rad", 0.6)
        self.declare_parameter("max_linear_accel", 0.12)
        self.declare_parameter("max_steering_rate", 1.0)  # rad/s
        self.declare_parameter("deadzone", 0.20)
        self.declare_parameter("command_timeout_sec", 0.50)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("start_enabled", False)

        self._throttle_axis = self.get_parameter("throttle_axis").value
        self._steering_axis = self.get_parameter("steering_axis").value
        self._enable_button = self.get_parameter("enable_button").value
        self._enable_debounce = self.get_parameter("enable_debounce_sec").value
        self._min_speed = self.get_parameter("min_linear_speed").value
        self._max_speed = self.get_parameter("max_linear_speed").value
        self._max_steering = self.get_parameter("max_steering_angle_rad").value
        self._max_linear_accel = self.get_parameter("max_linear_accel").value
        self._max_steering_rate = self.get_parameter("max_steering_rate").value
        self._deadzone = self.get_parameter("deadzone").value
        self._timeout = self.get_parameter("command_timeout_sec").value
        self._enabled = self.get_parameter("start_enabled").value
        self._buzzer_feedback = self.get_parameter("buzzer_feedback").value
        self._last_enable = False
        self._last_enable_time = -float("inf")
        self._last_joy_time = time.monotonic()
        self._last_publish_time = time.monotonic()
        self._target = Twist()
        self._output = Twist()
        self._buzzer_steps = []
        self._buzzer_deadline = time.monotonic() + 1.0
        self._startup_buzzer_pending = True

        self._publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self._buzzer_publisher = self.create_publisher(
            Bool, self.get_parameter("buzzer_topic").value, 10
        )
        self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self._on_joy, 10
        )
        period = 1.0 / self.get_parameter("publish_rate_hz").value
        self.create_timer(period, self._publish_command)
        self.create_timer(0.02, self._update_buzzer)
        self.get_logger().info(
            "R2 joystick control is %s; press button %d to toggle drive"
            % ("enabled" if self._enabled else "disabled", self._enable_button)
        )

    def _axis(self, message, index):
        if index < 0 or index >= len(message.axes):
            return 0.0
        value = float(message.axes[index])
        return 0.0 if abs(value) < self._deadzone else value

    def _on_joy(self, message):
        self._last_joy_time = time.monotonic()
        enable = (
            0 <= self._enable_button < len(message.buttons)
            and message.buttons[self._enable_button] == 1
        )
        now = time.monotonic()
        if (
            enable
            and not self._last_enable
            and now - self._last_enable_time >= self._enable_debounce
        ):
            self._last_enable_time = now
            self._enabled = not self._enabled
            self._target = Twist()
            self._output = Twist()
            self._publisher.publish(self._output)
            self.get_logger().info(
                "R2 joystick drive %s" % ("enabled" if self._enabled else "disabled")
            )
            self._startup_buzzer_pending = False
            if self._enabled:
                self._play_buzzer([(True, 0.55), (False, 0.05)])
            else:
                self._play_buzzer(
                    [
                        (True, 0.10),
                        (False, 0.10),
                        (True, 0.10),
                        (False, 0.10),
                        (True, 0.10),
                        (False, 0.05),
                    ]
                )
        self._last_enable = enable

        if not self._enabled:
            self._target = Twist()
            return

        # linear.y holds the wheel angle in rad here; _publish_command converts
        # it to the firmware's degrees / 1000.
        throttle = self._axis(message, self._throttle_axis)
        self._target.linear.x = throttle * self._max_speed
        if throttle != 0.0 and abs(self._target.linear.x) < self._min_speed:
            self._target.linear.x = self._min_speed if throttle > 0.0 else -self._min_speed
        self._target.linear.y = (
            self._axis(message, self._steering_axis) * self._max_steering
        )

    @staticmethod
    def _move_toward(current, target, maximum_delta):
        return max(current - maximum_delta, min(target, current + maximum_delta))

    def _play_buzzer(self, steps):
        if not self._buzzer_feedback:
            return
        self._buzzer_steps = list(steps)
        self._buzzer_deadline = time.monotonic()

    def _update_buzzer(self):
        now = time.monotonic()
        if self._startup_buzzer_pending and now >= self._buzzer_deadline:
            self._startup_buzzer_pending = False
            self._play_buzzer(
                [(True, 0.12), (False, 0.12), (True, 0.12), (False, 0.05)]
            )
        if not self._buzzer_steps or now < self._buzzer_deadline:
            return
        enabled, duration = self._buzzer_steps.pop(0)
        message = Bool()
        message.data = enabled
        self._buzzer_publisher.publish(message)
        self._buzzer_deadline = now + duration

    def _publish_command(self):
        now = time.monotonic()
        if not self._enabled or time.monotonic() - self._last_joy_time > self._timeout:
            self._output = Twist()
            self._last_publish_time = now
            self._publisher.publish(Twist())
            return

        dt = max(0.0, now - self._last_publish_time)
        self._last_publish_time = now
        self._output.linear.x = self._move_toward(
            self._output.linear.x,
            self._target.linear.x,
            abs(self._max_linear_accel) * dt,
        )
        self._output.linear.y = self._move_toward(
            self._output.linear.y,
            self._target.linear.y,
            abs(self._max_steering_rate) * dt,
        )
        if 0.0 < abs(self._output.linear.x) < abs(self._min_speed):
            if self._target.linear.x == 0.0:
                self._output.linear.x = 0.0
            else:
                self._output.linear.x = (
                    self._min_speed if self._target.linear.x > 0.0 else -self._min_speed
                )
        command = Twist()
        command.linear.x = self._output.linear.x
        command.linear.y = wheel_angle_to_firmware_steering(self._output.linear.y)
        self._publisher.publish(command)

    def stop(self):
        if not rclpy.ok():
            return
        for _ in range(3):
            self._publisher.publish(Twist())
        buzzer_off = Bool()
        buzzer_off.data = False
        self._buzzer_publisher.publish(buzzer_off)


def main(args=None):
    rclpy.init(args=args)
    node = R2JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
