#!/usr/bin/env python3
import signal
import time

import rclpy
from yahboomcar_bringup.Ackman_driver_R2 import yahboomcar_driver


def stop_driver(driver):
    if driver is None:
        return
    try:
        for _ in range(30):
            driver.car.set_car_motion(0.0, 0.0, 0.0)
            time.sleep(0.02)
    except Exception as exc:
        try:
            driver.get_logger().error(f"Failed to send hardware stop: {exc}")
        except Exception:
            pass


def main():
    rclpy.init()
    driver = yahboomcar_driver("driver_node")
    stop_requested = {"value": False}

    def request_stop(signum=None, frame=None):
        del signum, frame
        if not stop_requested["value"]:
            driver.get_logger().warn("Stopping R2 driver with hardware zero command")
        stop_requested["value"] = True
        stop_driver(driver)
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        request_stop()
    finally:
        stop_driver(driver)
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
