#!/usr/bin/env python3
"""
PKRC Wall-Follow Logger
- wall_follow_control 노드의 데이터를 CSV 로 로깅

wall_follow_control 이 퍼블리시하는 3개 디버그 배열 + 모드 문자열을 구독한다.
각 배열의 인덱스는 wall_follow_control.control_loop() 말미의 _pub_array 호출과
1:1 대응하는 계약이다 — 노드 쪽 배열 순서를 바꾸면 여기도 같이 바꿔야 한다.

Usage:
    ros2 run pkrc_control wall_following_logger
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Imu


class WallFollowLogger(Node):
    def __init__(self):
        super().__init__('wall_follow_logger')

        self.declare_parameter('log_dir', '/home/hero/hero_ws/logs')
        self.declare_parameter('log_rate', 10.0)   # Hz

        self.log_dir = self.get_parameter('log_dir').value
        self.log_rate = self.get_parameter('log_rate').value

        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.log_dir, f'wall_follow_{timestamp}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'timestamp', 'mode',
            # wall (소나 거리 제어)
            'target_distance', 'sonar_dist', 'distance_error',
            'surge_cmd', 'sonar_confidence', 'sonar_ok', 'sonar_raw',
            # scan (정렬 시퀀스 진행 상황)
            'scan_accum_deg', 'scan_best_dist', 'scan_best_yaw_deg',
            'scan_samples',
            # heading
            'target_yaw_deg', 'current_yaw_deg', 'yaw_error_deg',
            'yaw_rate_deg_s', 'yaw_cmd', 'pitch_deg', 'is_yawing',
            # depth
            'target_depth', 'current_depth', 'depth_error',
            'heave_cmd', 'depth_est_vel', 'pressure_pa',
            # dvl
            'dvl_vx', 'dvl_vy', 'drift_fb_surge',
            # thrusters
            't1', 't2', 't3', 't4', 't5', 't6',
            # imu raw
            'imu_yaw_rate_deg_s',
        ])

        # ── wall_debug (11) ───────────────────────────────────────────
        self.target_distance = 0.0
        self.sonar_dist = 0.0
        self.distance_error = 0.0
        self.surge_cmd = 0.0
        self.sonar_conf = 0.0
        self.sonar_ok = 0.0
        self.scan_accum_deg = 0.0
        self.scan_best_dist = -1.0
        self.scan_best_yaw_deg = 0.0
        self.scan_samples = 0.0
        self.sonar_raw = 0.0

        # ── yaw_debug (12) ────────────────────────────────────────────
        self.target_yaw_deg = 0.0
        self.current_yaw_deg = 0.0
        self.yaw_error_deg = 0.0
        self.yaw_rate_deg_s = 0.0
        self.yaw_cmd = 0.0
        self.pitch_deg = 0.0
        self.is_yawing = 0.0
        self.dvl_vx = 0.0
        self.dvl_vy = 0.0
        self.drift_fb_surge = 0.0

        # ── depth_debug (6) ───────────────────────────────────────────
        self.target_depth = 0.0
        self.current_depth = 0.0
        self.depth_error = 0.0
        self.heave_cmd = 0.0
        self.depth_est_vel = 0.0
        self.pressure_pa = 0.0

        self.thruster_currents = [0.0] * 6
        self.mode = 'IDLE'
        self.imu_yaw_rate_deg_s = 0.0

        # ── Subscribers ───────────────────────────────────────────────
        # wall_follow_control 의 디버그 퍼블리셔는 모두 기본 QoS(RELIABLE, 10)
        self.create_subscription(Float64MultiArray, '/teleop/wall_debug',
                                 self.wall_callback, 10)
        self.create_subscription(Float64MultiArray, '/teleop/yaw_debug',
                                 self.yaw_callback, 10)
        self.create_subscription(Float64MultiArray, '/teleop/depth_debug',
                                 self.depth_callback, 10)
        self.create_subscription(Float64MultiArray,
                                 '/teleop/thruster_currents',
                                 self.thruster_callback, 10)
        self.create_subscription(String, '/teleop/wall_mode',
                                 self.mode_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.log_timer = self.create_timer(1.0 / self.log_rate, self.log_data)

        self.get_logger().info('=== Wall-Follow Logger Started ===')
        self.get_logger().info(f'Logging to: {self.csv_path}')
        self.get_logger().info(f'Log rate: {self.log_rate} Hz')

    # ─── Callbacks ──────────────────────────────────────────────────
    def wall_callback(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) < 11:
            return
        (self.target_distance, self.sonar_dist, self.distance_error,
         self.surge_cmd, self.sonar_conf, self.sonar_ok,
         self.scan_accum_deg, self.scan_best_dist, self.scan_best_yaw_deg,
         self.scan_samples, self.sonar_raw) = d[:11]

    def yaw_callback(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) < 12:
            return
        (self.target_yaw_deg, self.current_yaw_deg, self.yaw_error_deg,
         self.yaw_rate_deg_s, self.yaw_cmd, _heave, self.pitch_deg,
         self.is_yawing, self.dvl_vx, self.dvl_vy,
         self.drift_fb_surge, _reserved) = d[:12]

    def depth_callback(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) < 6:
            return
        (self.target_depth, self.current_depth, self.depth_error,
         self.heave_cmd, self.depth_est_vel, self.pressure_pa) = d[:6]

    def thruster_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.thruster_currents = list(msg.data[:6])

    def mode_callback(self, msg: String):
        self.mode = msg.data

    def imu_callback(self, msg: Imu):
        self.imu_yaw_rate_deg_s = math.degrees(msg.angular_velocity.z)

    # ─── Logging ────────────────────────────────────────────────────
    def log_data(self):
        timestamp = self.get_clock().now().nanoseconds / 1e9

        row = [
            f'{timestamp:.3f}', self.mode,
            f'{self.target_distance:.3f}',
            f'{self.sonar_dist:.3f}',
            f'{self.distance_error:.3f}',
            f'{self.surge_cmd:.4f}',
            f'{self.sonar_conf:.1f}',
            int(self.sonar_ok),
            f'{self.sonar_raw:.3f}',
            f'{self.scan_accum_deg:.1f}',
            f'{self.scan_best_dist:.3f}',
            f'{self.scan_best_yaw_deg:.2f}',
            int(self.scan_samples),
            f'{self.target_yaw_deg:.2f}',
            f'{self.current_yaw_deg:.2f}',
            f'{self.yaw_error_deg:.2f}',
            f'{self.yaw_rate_deg_s:.2f}',
            f'{self.yaw_cmd:.4f}',
            f'{self.pitch_deg:.2f}',
            int(self.is_yawing),
            f'{self.target_depth:.3f}',
            f'{self.current_depth:.3f}',
            f'{self.depth_error:.3f}',
            f'{self.heave_cmd:.4f}',
            f'{self.depth_est_vel:.4f}',
            f'{self.pressure_pa:.1f}',
            f'{self.dvl_vx:.4f}',
            f'{self.dvl_vy:.4f}',
            f'{self.drift_fb_surge:.4f}',
        ] + [f'{c:.3f}' for c in self.thruster_currents] + [
            f'{self.imu_yaw_rate_deg_s:.2f}',
        ]

        self.csv_writer.writerow(row)
        self.csv_file.flush()

        self.get_logger().info(
            f'[{self.mode}] d:{self.sonar_dist:.2f}m '
            f'(err:{self.distance_error:+.2f} conf:{self.sonar_conf:.0f}% '
            f'{"OK" if self.sonar_ok else "LOST"}) '
            f'surge:{self.surge_cmd:+.3f} '
            f'yaw_err:{self.yaw_error_deg:+.1f}° '
            f'depth:{self.current_depth:.2f}m',
            throttle_duration_sec=1.0)

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'Log saved to: {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowLogger()

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
