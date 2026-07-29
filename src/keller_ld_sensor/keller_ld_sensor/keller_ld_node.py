#!/usr/bin/env python3
"""ROS 2 node publishing Keller LD pressure (bar) and temperature (degC)."""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from keller_ld_sensor.kellerLD import KellerLD


class KellerLDNode(Node):
    def __init__(self):
        super().__init__('keller_ld_sensor')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('reference_pressure_bar', -1.0)  # <0 -> keep auto default

        bus_num = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        ref_bar = self.get_parameter('reference_pressure_bar').get_parameter_value().double_value

        self.pressure_pub = self.create_publisher(Float64, 'pressure_bar', 10)
        # Legacy topic in mbar for teleop / depth-control nodes expecting it.
        self.pressure_mbar_pub = self.create_publisher(Float64, 'pressure', 10)
        self.temperature_pub = self.create_publisher(Float64, 'temperature', 10)

        self.sensor = KellerLD(bus=bus_num)
        self.get_logger().info("Keller LD stabilizing... (1 s)")
        time.sleep(1.0)

        self.sensor_ok = self.sensor.init()
        if not self.sensor_ok:
            self.get_logger().error("Keller LD init failed")
            return

        if ref_bar >= 0.0:
            self.sensor.set_reference_pressure(ref_bar)

        self.get_logger().info(
            f"Keller LD initialized: mode={self.sensor.pMode}, "
            f"pMin={self.sensor.pMin:.4f} bar, pMax={self.sensor.pMax:.4f} bar, "
            f"offset={self.sensor.pModeOffset:.5f} bar")

        period = 1.0 / max(rate_hz, 0.1)
        self.consecutive_failures = 0
        self.timer = self.create_timer(period, self.publish_sensor_data)

    def _read_sensor(self):
        for _ in range(3):
            try:
                if not self.sensor.read():
                    time.sleep(0.02)
                    continue
                p = self.sensor.pressure()
                t = self.sensor.temperature()
            except OSError:
                time.sleep(0.02)
                continue

            if p is None or t is None:
                continue
            if -40.0 < t < 85.0:
                return p, t
            time.sleep(0.01)
        return None

    def publish_sensor_data(self):
        if not self.sensor_ok:
            return
        result = self._read_sensor()
        if result is None:
            self.consecutive_failures += 1
            if self.consecutive_failures >= 20:
                self.get_logger().warn(
                    f"Sensor read failed {self.consecutive_failures}x",
                    throttle_duration_sec=5.0)
            return

        self.consecutive_failures = 0
        pressure_bar, temperature = result
        self.pressure_pub.publish(Float64(data=pressure_bar))
        self.pressure_mbar_pub.publish(Float64(data=pressure_bar * 1000.0))
        self.temperature_pub.publish(Float64(data=temperature))


def main(args=None):
    rclpy.init(args=args)
    node = KellerLDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
