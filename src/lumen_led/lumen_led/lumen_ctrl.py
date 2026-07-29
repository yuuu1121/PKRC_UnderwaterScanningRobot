"""Lumen LED CLI controller.

Usage:
  ros2 run lumen_led lumen_ctrl on
  ros2 run lumen_led lumen_ctrl off
  ros2 run lumen_led lumen_ctrl 0.5   # brightness 0.0 ~ 1.0
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


def main():
    if len(sys.argv) < 2:
        print("Usage: ros2 run lumen_led lumen_ctrl <on|off|0.0~1.0>")
        sys.exit(1)

    arg = sys.argv[1].strip().lower()

    rclpy.init()
    node = Node('lumen_ctrl')

    if arg == 'on':
        pub = node.create_publisher(Bool, 'lumen/on_off', 1)
        msg = Bool()
        msg.data = True
    elif arg == 'off':
        pub = node.create_publisher(Bool, 'lumen/on_off', 1)
        msg = Bool()
        msg.data = False
    else:
        try:
            val = float(arg)
        except ValueError:
            print(f"ERROR: '{arg}' 은 on/off 또는 0.0~1.0 숫자여야 합니다.")
            sys.exit(1)
        pub = node.create_publisher(Float32, 'lumen/brightness', 1)
        msg = Float32()
        msg.data = max(0.0, min(1.0, val))

    # 퍼블리셔가 연결될 때까지 잠깐 대기 후 전송
    import time
    deadline = time.time() + 3.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if pub.get_subscription_count() == 0:
        print("WARNING: lumen_node 가 실행 중이지 않을 수 있습니다.")

    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)

    print(f"lumen: {arg}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
