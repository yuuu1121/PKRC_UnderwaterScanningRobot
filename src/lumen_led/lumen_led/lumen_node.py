"""Lumen LED controller via hardware PWM on J37 GPIO07_BUF (pwmchip3/pwm0)."""

import mmap
import os
import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

# Hardware PWM path: J37 GPIO07_BUF → pwmchip3, channel 0
PWM_CHIP_DIR = '/sys/bus/platform/devices/32e0000.pwm/pwm/pwmchip3'
PWM_ID = 0

PWM_PERIOD_NS = 20_000_000        # 50 Hz
PWM_MIN_NS    =  1_100_000        # 1100 µs  — off
PWM_MAX_NS    =  1_900_000        # 1900 µs  — full brightness

# Pinmux: SOC_GPIO19_PG6 → PWM7
# PADCTL base 0x02430000, register offset 0x4080
PINMUX_BASE   = 0x02430000
PINMUX_SIZE   = 0x19100
PG6_OFFSET    = 0x4080


def _setup_pinmux_pwm7():
    """Set SOC_GPIO19_PG6 pinmux to GP function (PWM7), output, driven."""
    try:
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        mm = mmap.mmap(fd, PINMUX_SIZE, mmap.MAP_SHARED,
                       mmap.PROT_READ | mmap.PROT_WRITE,
                       offset=PINMUX_BASE)
        val = struct.unpack('<I', mm[PG6_OFFSET:PG6_OFFSET + 4])[0]
        func = val & 0x3
        tristate = (val >> 4) & 1
        sfio = (val >> 10) & 1
        if func != 0 or tristate != 0 or sfio != 1:
            new_val = val
            new_val &= ~0x3      # func = 0 (GP/PWM7)
            new_val &= ~(1 << 4) # tristate = 0 (driven)
            new_val &= ~(1 << 6) # input_en = 0 (output)
            new_val |= (1 << 10) # sfio = 1 (PWM routed to pin)
            mm[PG6_OFFSET:PG6_OFFSET + 4] = struct.pack('<I', new_val)
        mm.close()
        os.close(fd)
        return True
    except (OSError, PermissionError):
        return False


class HardwarePWM:
    """Thin wrapper around the Linux sysfs PWM interface."""

    def __init__(self, chip_dir: str, pwm_id: int):
        self._pwm_dir = os.path.join(chip_dir, f'pwm{pwm_id}')
        self._export_path = os.path.join(chip_dir, 'export')
        self._unexport_path = os.path.join(chip_dir, 'unexport')
        self._pwm_id = pwm_id
        self._enabled = False

    # ------------------------------------------------------------------
    def _write(self, filename: str, value) -> None:
        path = os.path.join(self._pwm_dir, filename)
        with open(path, 'w') as f:
            f.write(str(value))

    def setup(self) -> None:
        """Export the PWM channel and configure period."""
        if not os.path.exists(self._pwm_dir):
            with open(self._export_path, 'w') as f:
                f.write(str(self._pwm_id))
            # Wait for udev to set group permissions on the new pwm directory
            time.sleep(0.5)
        # Always write period before duty_cycle
        self._write('period', PWM_PERIOD_NS)
        self._write('duty_cycle', PWM_MIN_NS)
        self._write('enable', 1)
        self._enabled = True

    def set_duty_ns(self, duty_ns: int) -> None:
        duty_ns = max(PWM_MIN_NS, min(PWM_MAX_NS, duty_ns))
        self._write('duty_cycle', duty_ns)

    def set_brightness(self, value: float) -> None:
        """value: 0.0 (off) … 1.0 (full brightness)."""
        value = max(0.0, min(1.0, value))
        duty_ns = int(PWM_MIN_NS + value * (PWM_MAX_NS - PWM_MIN_NS))
        self.set_duty_ns(duty_ns)

    def close(self) -> None:
        if self._enabled:
            try:
                self._write('duty_cycle', PWM_MIN_NS)
                self._write('enable', 0)
            except OSError:
                pass
            try:
                with open(self._unexport_path, 'w') as f:
                    f.write(str(self._pwm_id))
            except OSError:
                pass
            self._enabled = False


class LumenNode(Node):
    def __init__(self):
        super().__init__('lumen_led')
        self.declare_parameter('initial_brightness', 0.0)

        self._brightness = self.get_parameter('initial_brightness').value

        if not _setup_pinmux_pwm7():
            self.get_logger().warn(
                'Pinmux setup skipped (need root). '
                'Run run_lumen.sh or pre-configure pinmux.')

        self._pwm = HardwarePWM(PWM_CHIP_DIR, PWM_ID)
        self._pwm.setup()

        # BlueRobotics Lumen arming: hold 1100µs (off) for 2s first
        self.get_logger().info('Lumen arming... (2s)')
        time.sleep(2.0)

        self._pwm.set_brightness(self._brightness)
        self.get_logger().info(
            f'Lumen LED ready: pwmchip3/pwm0 (J37 GPIO07_BUF)  '
            f'brightness={self._brightness:.2f}'
        )

        self.create_subscription(Float32, 'lumen/brightness', self._cb_brightness, 10)
        self.create_subscription(Bool,    'lumen/on_off',     self._cb_on_off,     10)
        self._pub = self.create_publisher(Float32, 'lumen/state', 10)
        self.create_timer(1.0, self._publish_state)

    def _cb_brightness(self, msg: Float32):
        self._set(msg.data)

    def _cb_on_off(self, msg: Bool):
        self._set(1.0 if msg.data else 0.0)

    def _set(self, val: float):
        val = max(0.0, min(1.0, val))
        self._pwm.set_brightness(val)
        self._brightness = val
        self.get_logger().info(f'Brightness → {val:.2f}')

    def _publish_state(self):
        msg = Float32()
        msg.data = self._brightness
        self._pub.publish(msg)

    def destroy_node(self):
        self._pwm.set_brightness(0.0)
        self._pwm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LumenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
