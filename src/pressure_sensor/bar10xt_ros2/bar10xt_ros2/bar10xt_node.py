#!/usr/bin/env python3
"""ROS2 (Humble) node for the Keller Bar10XT pressure sensor.

Publishes:
  ~/pressure    sensor_msgs/FluidPressure   [Pa]
  ~/temperature sensor_msgs/Temperature     [deg C]
  ~/depth       std_msgs/Float64            [m]

Depth is derived from gauge pressure:  depth = (P - P_atm) / (rho * g)

Parameters:
  bus                  I2C bus number                      (default 1)
  rate                 publish rate [Hz]                   (default 5.0)
  frame_id             header frame_id                     (default "pressure_sensor")
  water_density        rho [kg/m^3] (fresh 997, sea 1025)  (default 997.0)
  gravity              g [m/s^2]                           (default 9.80665)
  atmospheric_pressure surface/zero-depth pressure [Pa]    (default 101325.0)
  auto_zero            sample at startup to set P_atm       (default False)
  pressure_offset      single-point calibration [bar]      (default 0.0)
  temperature_offset   single-point calibration [deg C]    (default 0.0)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float64

from bar10xt_ros2.keller_driver import KellerBar10XT

BAR_TO_PA = 1.0e5


class Bar10xtNode(Node):
    def __init__(self):
        super().__init__("bar10xt")

        self.bus = self.declare_parameter("bus", 1).value
        self.rate_hz = self.declare_parameter("rate", 5.0).value
        self.frame_id = self.declare_parameter("frame_id", "pressure_sensor").value
        self.rho = self.declare_parameter("water_density", 997.0).value
        self.g = self.declare_parameter("gravity", 9.80665).value
        self.atm_pa = self.declare_parameter("atmospheric_pressure", 101325.0).value
        self.auto_zero = self.declare_parameter("auto_zero", False).value
        self.p_offset_bar = self.declare_parameter("pressure_offset", 0.0).value
        self.t_offset_c = self.declare_parameter("temperature_offset", 0.0).value
        # This unit's pressure MSB is stuck, capping depth at 0.66 m; the driver
        # counts LSB rollovers to get past it. See keller_driver's docstring --
        # this is a workaround for broken hardware, not a feature.
        self.unwrap = self.declare_parameter("unwrap_msb", True).value

        # Bar10XT: 10 bar range, 0.1% FS accuracy -> ~10 mbar = ~1000 Pa std.
        self.p_variance = (0.001 * 10.0 * BAR_TO_PA) ** 2
        self.t_variance = 1.5 ** 2  # on-chip temp sensor ~ +/-1.5 deg C

        self.sensor = KellerBar10XT(bus=self.bus, unwrap=self.unwrap)
        self.sensor.init()
        self.get_logger().info(
            "Bar10XT init: pMin=%.3f pMax=%.3f offset=%.4f bar"
            % (self.sensor.pMin, self.sensor.pMax, self.sensor.offset))
        if self.unwrap:
            self.get_logger().warn(
                "MSB unwrap ACTIVE -- this sensor's pressure MSB is stuck, so depth "
                "is reconstructed from LSB rollovers. Relative changes are sound; "
                "absolute depth drifts by 0.799 m per missed rollover. Start at the "
                "surface, and replace the sensor.")
        self._last_wraps = 0

        if self.auto_zero:
            self._auto_zero()

        self.pub_p = self.create_publisher(FluidPressure, "~/pressure", qos_profile_sensor_data)
        self.pub_t = self.create_publisher(Temperature, "~/temperature", qos_profile_sensor_data)
        self.pub_d = self.create_publisher(Float64, "~/depth", qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / float(self.rate_hz), self._on_timer)

    def _auto_zero(self, n=20):
        """Average a few samples and use that as the surface (zero-depth) pressure."""
        import time
        vals = []
        for _ in range(n):
            p, _t = self.sensor.read()
            vals.append((p + self.p_offset_bar) * BAR_TO_PA)
            time.sleep(0.05)
        self.atm_pa = sum(vals) / len(vals)
        # Re-anchor so the zeroing samples themselves cannot leave a stale
        # rollover offset behind; depth starts from here.
        self.sensor.reset_unwrap()
        self.get_logger().info("auto_zero: atmospheric_pressure set to %.1f Pa" % self.atm_pa)

    def _on_timer(self):
        try:
            p_bar, t_c = self.sensor.read()
        except Exception as e:
            self.get_logger().warn("sensor read failed: %s" % e, throttle_duration_sec=5.0)
            return

        p_bar += self.p_offset_bar
        t_c += self.t_offset_c
        p_pa = p_bar * BAR_TO_PA
        depth_m = (p_pa - self.atm_pa) / (self.rho * self.g)

        if self.unwrap and self.sensor.wrap_suspect > self._last_wraps:
            # Descending fast enough that a rollover may have slipped between
            # samples -- every later depth would then be off by a whole 0.799 m.
            self._last_wraps = self.sensor.wrap_suspect
            self.get_logger().warn(
                "depth change near the rollover limit (%d so far) -- absolute depth "
                "may now be off by a multiple of 0.799 m. Descend slower, and "
                "re-zero at the surface." % self.sensor.wrap_suspect,
                throttle_duration_sec=5.0)

        now = self.get_clock().now().to_msg()

        pmsg = FluidPressure()
        pmsg.header.stamp = now
        pmsg.header.frame_id = self.frame_id
        pmsg.fluid_pressure = p_pa
        pmsg.variance = self.p_variance
        self.pub_p.publish(pmsg)

        tmsg = Temperature()
        tmsg.header.stamp = now
        tmsg.header.frame_id = self.frame_id
        tmsg.temperature = t_c
        tmsg.variance = self.t_variance
        self.pub_t.publish(tmsg)

        self.pub_d.publish(Float64(data=depth_m))


def main(args=None):
    rclpy.init(args=args)
    node = Bar10xtNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sensor.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
