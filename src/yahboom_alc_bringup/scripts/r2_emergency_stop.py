#!/usr/bin/env python3
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class R2EmergencyStop(Node):
    def __init__(self):
        super().__init__("r2_emergency_stop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("publish_count", 150)
        self.declare_parameter("driver_wait_sec", 4.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.publish_count = int(self.get_parameter("publish_count").value)
        self.driver_wait_sec = float(self.get_parameter("driver_wait_sec").value)
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.driver_process = None

    def run(self):
        if not self.has_cmd_vel_subscriber():
            self.get_logger().warn(
                "No /cmd_vel subscriber found, starting Ackman_driver_R2 for stop"
            )
            self.driver_process = subprocess.Popen(
                ["ros2", "run", "yahboomcar_bringup", "Ackman_driver_R2"]
            )
            deadline = time.monotonic() + self.driver_wait_sec
            while time.monotonic() < deadline and not self.has_cmd_vel_subscriber():
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn("Publishing emergency zero cmd_vel")
        period = 1.0 / self.publish_rate_hz
        stop = Twist()
        for _ in range(self.publish_count):
            self.publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.publisher.publish(stop)

    def has_cmd_vel_subscriber(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        return bool(self.get_subscriptions_info_by_topic(self.cmd_vel_topic))


def main():
    rclpy.init()
    node = R2EmergencyStop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
