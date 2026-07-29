#!/usr/bin/env python3
"""
Teleop Logger - IMU, 키보드 입력, 스러스터 명령 로깅
직진성 확인 및 heading hold 디버깅용

Usage:
    ros2 run pkrc_control teleop_logger
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Float64MultiArray, String
import math
import csv
from datetime import datetime


class TeleopLogger(Node):
    def __init__(self):
        super().__init__('teleop_logger')

        self.declare_parameter('log_interval', 0.05)  # 로깅 간격 (초)

        self.log_interval = self.get_parameter('log_interval').value

        # IMU 데이터
        self.current_yaw = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.angular_vel_z = 0.0
        self.imu_initialized = False
        # IMU raw quaternion
        self.quat_w = 0.0
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0

        # 시작 값
        self.start_yaw = None
        self.start_time = None
        self.last_log_time = 0

        # 키보드/Force 데이터
        self.last_key = ''
        self.force = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # surge, sway, heave, roll, pitch, yaw
        self.thruster_currents = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # T1~T6
        # target_yaw, current_yaw, yaw_error, yaw_rate, yaw_command,
        # heave_command, pitch, is_yawing, dvl_vx, dvl_vy, drift_fb_surge, drift_fb_sway
        self.yaw_debug = [0.0] * 12
        self.depth_debug = [0.0, 0.0, 0.0, 0.0, 0.0]  # target_depth, current_depth, depth_error, depth_correction, ...
        # bar10xt 압력 센서 절대압 [Pa] — /bar10xt/pressure 에서 직접 수신
        self.current_pressure = 0.0

        # 데이터 저장
        self.yaw_history = []
        self.timestamps = []

        # CSV 파일
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f'/home/hero/hero_ws/logs/teleop_log_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time', 'key',
            'imu_yaw_deg', 'yaw_from_start_deg', 'roll_deg', 'pitch_deg',
            'angular_vel_z', 'yaw_rate_deg_s',
            'quat_w', 'quat_x', 'quat_y', 'quat_z',
            'target_yaw_deg', 'current_yaw_deg', 'yaw_error_deg', 'yaw_rate_ctrl_deg_s',
            'yaw_command', 'heave_command_out', 'pitch_ctrl_deg', 'is_yawing',
            'dvl_vx', 'dvl_vy', 'drift_fb_surge', 'drift_fb_sway',
            'target_depth_m', 'current_depth_m', 'depth_error_m', 'depth_correction', 'pressure_pa',
            'force_surge', 'force_sway', 'force_heave', 'force_yaw',
            'T1_surge_L', 'T2_surge_R', 'T3_sway_F', 'T4_sway_R', 'T5_heave_L', 'T6_heave_R'
        ])

        # IMU 구독
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Teleop 데이터 구독
        self.force_sub = self.create_subscription(
            Float64MultiArray,
            '/teleop/force',
            self.force_callback,
            10
        )

        self.thruster_sub = self.create_subscription(
            Float64MultiArray,
            '/teleop/thruster_currents',
            self.thruster_callback,
            10
        )

        self.key_sub = self.create_subscription(
            String,
            '/teleop/key',
            self.key_callback,
            10
        )

        self.yaw_debug_sub = self.create_subscription(
            Float64MultiArray,
            '/teleop/yaw_debug',
            self.yaw_debug_callback,
            10
        )

        self.depth_debug_sub = self.create_subscription(
            Float64MultiArray,
            '/teleop/depth_debug',
            self.depth_debug_callback,
            10
        )

        # bar10xt 압력 센서 직접 구독 (FluidPressure [Pa], BEST_EFFORT)
        self.pressure_sub = self.create_subscription(
            FluidPressure,
            '/bar10xt/pressure',
            self.pressure_callback,
            qos_profile_sensor_data
        )

        # 상태 출력 타이머
        self.print_timer = self.create_timer(1.0, self.print_status)

        self.get_logger().info(f'=== Teleop Logger ===')
        self.get_logger().info(f'IMU topic: /imu/data')
        self.get_logger().info(f'Teleop topics: /teleop/force, /teleop/thruster_currents, /teleop/key')
        self.get_logger().info(f'Debug topics: /teleop/yaw_debug, /teleop/depth_debug')
        self.get_logger().info(f'Saving to: {self.csv_filename}')
        self.get_logger().info('Press Ctrl+C to stop and show summary')

    def quaternion_to_euler(self, orientation):
        """쿼터니언에서 roll, pitch, yaw 추출"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (orientation.w * orientation.x + orientation.y * orientation.z)
        cosr_cosp = 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (orientation.w * orientation.y - orientation.z * orientation.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def force_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.force = list(msg.data[:6])

    def thruster_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.thruster_currents = list(msg.data[:6])

    def key_callback(self, msg: String):
        if msg.data:
            self.last_key = msg.data

    def yaw_debug_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 12:
            self.yaw_debug = list(msg.data[:12])

    def depth_debug_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 5:
            self.depth_debug = list(msg.data[:5])

    def pressure_callback(self, msg: FluidPressure):
        # fluid_pressure 는 절대압 [Pa]
        self.current_pressure = msg.fluid_pressure

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.start_time is None:
            self.start_time = current_time

        # 로깅 간격 체크
        if current_time - self.last_log_time < self.log_interval:
            return
        self.last_log_time = current_time

        # 자세 추출
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation)
        self.current_roll = roll
        self.current_pitch = pitch
        self.current_yaw = yaw
        self.angular_vel_z = msg.angular_velocity.z
        # Quaternion raw 저장
        self.quat_w = msg.orientation.w
        self.quat_x = msg.orientation.x
        self.quat_y = msg.orientation.y
        self.quat_z = msg.orientation.z

        if not self.imu_initialized:
            self.start_yaw = yaw
            self.imu_initialized = True
            self.get_logger().info(f'Start yaw: {math.degrees(yaw):.1f}°')

        # 시작점 대비 yaw 변화
        yaw_from_start = yaw - self.start_yaw
        # Normalize to [-pi, pi]
        while yaw_from_start > math.pi:
            yaw_from_start -= 2 * math.pi
        while yaw_from_start < -math.pi:
            yaw_from_start += 2 * math.pi

        elapsed = current_time - self.start_time

        # 데이터 저장
        self.yaw_history.append(math.degrees(yaw))
        self.timestamps.append(elapsed)

        # Yaw rate (deg/s)
        yaw_rate = math.degrees(self.angular_vel_z)

        # CSV 저장
        self.csv_writer.writerow([
            f'{elapsed:.3f}',
            self.last_key,
            f'{math.degrees(yaw):.2f}',
            f'{math.degrees(yaw_from_start):.2f}',
            f'{math.degrees(roll):.2f}',
            f'{math.degrees(pitch):.2f}',
            f'{self.angular_vel_z:.4f}',
            f'{yaw_rate:.2f}',
            f'{self.quat_w:.6f}',
            f'{self.quat_x:.6f}',
            f'{self.quat_y:.6f}',
            f'{self.quat_z:.6f}',
            f'{self.yaw_debug[0]:.2f}',  # target_yaw_deg
            f'{self.yaw_debug[1]:.2f}',  # current_yaw_deg
            f'{self.yaw_debug[2]:.2f}',  # yaw_error_deg
            f'{self.yaw_debug[3]:.2f}',  # yaw_rate_ctrl_deg_s
            f'{self.yaw_debug[4]:.4f}',  # yaw_command ([-1,1] 제어출력)
            f'{self.yaw_debug[5]:.4f}',  # heave_command_out
            f'{self.yaw_debug[6]:.2f}',  # pitch_ctrl_deg
            f'{self.yaw_debug[7]:.0f}',  # is_yawing
            f'{self.yaw_debug[8]:.4f}',  # dvl_vx
            f'{self.yaw_debug[9]:.4f}',  # dvl_vy
            f'{self.yaw_debug[10]:.4f}',  # drift_fb_surge
            f'{self.yaw_debug[11]:.4f}',  # drift_fb_sway
            f'{self.depth_debug[0]:.4f}',  # target_depth_m
            f'{self.depth_debug[1]:.4f}',  # current_depth_m
            f'{self.depth_debug[2]:.4f}',  # depth_error_m
            f'{self.depth_debug[3]:.4f}',  # depth_correction
            f'{self.current_pressure:.2f}',  # pressure_pa (bar10xt 직접 수신)
            f'{self.force[0]:.3f}',
            f'{self.force[1]:.3f}',
            f'{self.force[2]:.3f}',
            f'{self.force[5]:.3f}',
            f'{self.thruster_currents[0]:.3f}',
            f'{self.thruster_currents[1]:.3f}',
            f'{self.thruster_currents[2]:.3f}',
            f'{self.thruster_currents[3]:.3f}',
            f'{self.thruster_currents[4]:.3f}',
            f'{self.thruster_currents[5]:.3f}'
        ])
        self.csv_file.flush()

    def print_status(self):
        """주기적 상태 출력"""
        if not self.imu_initialized:
            self.get_logger().info('Waiting for IMU data...')
            return

        yaw_deg = math.degrees(self.current_yaw)
        yaw_from_start = self.current_yaw - self.start_yaw
        while yaw_from_start > math.pi:
            yaw_from_start -= 2 * math.pi
        while yaw_from_start < -math.pi:
            yaw_from_start += 2 * math.pi

        yaw_rate = math.degrees(self.angular_vel_z)

        # 현재 움직임 표시
        move = ''
        if abs(self.force[0]) > 0.1:
            move = 'FWD' if self.force[0] > 0 else 'BWD'
        elif abs(self.force[1]) > 0.1:
            move = 'RIGHT' if self.force[1] > 0 else 'LEFT'
        elif abs(self.force[5]) > 0.1:
            move = 'YAW_R' if self.force[5] > 0 else 'YAW_L'

        self.get_logger().info(
            f'[{move:5}] Yaw: {yaw_deg:.1f}° | '
            f'ΔYaw: {math.degrees(yaw_from_start):+.1f}° | '
            f'Rate: {yaw_rate:+.1f}°/s | '
            f'Key: {self.last_key}'
        )

    def print_summary(self):
        """종료 시 요약 출력"""
        if len(self.yaw_history) < 2:
            self.get_logger().info('Not enough data collected')
            return

        # Yaw 변화 통계
        yaw_changes = []
        for i in range(1, len(self.yaw_history)):
            change = self.yaw_history[i] - self.yaw_history[i-1]
            # Handle wrap-around
            if change > 180:
                change -= 360
            elif change < -180:
                change += 360
            yaw_changes.append(abs(change))

        total_yaw_change = sum(yaw_changes)
        max_yaw = max(self.yaw_history)
        min_yaw = min(self.yaw_history)
        yaw_range = max_yaw - min_yaw
        if yaw_range > 180:
            yaw_range = 360 - yaw_range

        # 시작-끝 yaw 차이
        start_yaw = self.yaw_history[0]
        end_yaw = self.yaw_history[-1]
        net_yaw_change = end_yaw - start_yaw
        if net_yaw_change > 180:
            net_yaw_change -= 360
        elif net_yaw_change < -180:
            net_yaw_change += 360

        duration = self.timestamps[-1] if self.timestamps else 0

        self.get_logger().info('=' * 50)
        self.get_logger().info('=== SUMMARY ===')
        self.get_logger().info(f'Duration: {duration:.1f} s')
        self.get_logger().info(f'Data points: {len(self.yaw_history)}')
        self.get_logger().info(f'Start yaw: {start_yaw:.1f}°')
        self.get_logger().info(f'End yaw: {end_yaw:.1f}°')
        self.get_logger().info(f'Net yaw change: {net_yaw_change:+.1f}°')
        self.get_logger().info(f'Yaw range: {yaw_range:.1f}° (min: {min_yaw:.1f}°, max: {max_yaw:.1f}°)')
        self.get_logger().info(f'Total yaw movement: {total_yaw_change:.1f}°')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Data saved to: {self.csv_filename}')

    def destroy_node(self):
        self.print_summary()
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
