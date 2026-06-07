#!/usr/bin/env python3
import signal

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class R2SafetyStop(Node):
    def __init__(self):
        super().__init__("r2_safety_stop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("stop_duration_sec", 4.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.stop_duration_sec = float(
            self.get_parameter("stop_duration_sec").value
        )
        self.stop_requested = False
        self.stop_publish_count = 0
        self.stop_publish_target = max(
            1, int(self.publish_rate_hz * self.stop_duration_sec)
        )

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)
        self.get_logger().info(f"R2 safety stop armed on {self.cmd_vel_topic}")

    def request_stop(self):
        if not self.stop_requested:
            self.get_logger().warn("Shutdown detected, publishing zero cmd_vel")
        self.stop_requested = True

    def on_timer(self):
        if not self.stop_requested:
            return

        self.publisher.publish(Twist())
        self.stop_publish_count += 1
        if self.stop_publish_count >= self.stop_publish_target:
            raise SystemExit


def main():
    rclpy.init()
    node = R2SafetyStop()

    def handle_shutdown(signum, frame):
        del signum, frame
        node.request_stop()

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.request_stop()
        for _ in range(node.stop_publish_target):
            node.publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
