#!/usr/bin/env python3
"""
Comparison Data Logger for UKFM vs EKF vs 3-Sensor Integration

Subscribes:
  - /ukfm/odom (UKFM estimate)
  - /ekf/odom (EKF estimate)
  - /imu/data (IMU)
  - /pressure (Depth)
  - /dvl/data (DVL velocity)
  - /aruco/pose_6dof (ArUco detection)

Saves to CSV file with timestamp and all three methods' data
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import csv
import os
from datetime import datetime
import numpy as np
from scipy.spatial.transform import Rotation

try:
    from dvl_msgs.msg import DVL
    DVL_AVAILABLE = True
except ImportError:
    DVL_AVAILABLE = False


class ComparisonDataLogger(Node):
    def __init__(self):
        super().__init__('comparison_data_logger')

        # Parameters
        # Default output directory: plot_tools/csv_data
        default_output = os.path.expanduser('~/hero_ws/src/plot_tools/csv_data')
        self.declare_parameter('output_dir', default_output)
        self.declare_parameter('log_rate', 10.0)  # Hz
        self.declare_parameter('imu_inverted', True)
        self.declare_parameter('imu_rotation_axis', 'x')
        self.declare_parameter('dvl_transform_enabled', False)

        self.output_dir = self.get_parameter('output_dir').value
        log_rate = self.get_parameter('log_rate').value

        # IMU transformation
        imu_inverted = self.get_parameter('imu_inverted').value
        imu_rotation_axis = self.get_parameter('imu_rotation_axis').value
        self.dvl_transform_enabled = self.get_parameter('dvl_transform_enabled').value

        # DVL to Robot body frame transformation
        # DVL mounting: LED → Robot -Y (right), Transducers → Robot +X (forward)
        self.R_dvl_to_body = np.array([
            [0, 0, 1],   # robot_vx (surge) = dvl_vz
            [-1, 0, 0],  # robot_vy (sway) = -dvl_vx
            [0, 1, 0],   # robot_vz (heave) = dvl_vy
        ])

        if imu_inverted:
            if imu_rotation_axis == 'x':
                self.R_imu_correction = Rotation.from_euler('x', 180, degrees=True)
            elif imu_rotation_axis == 'y':
                self.R_imu_correction = Rotation.from_euler('y', 180, degrees=True)
            elif imu_rotation_axis == 'z':
                self.R_imu_correction = Rotation.from_euler('z', 180, degrees=True)
            else:
                self.R_imu_correction = Rotation.from_euler('x', 180, degrees=True)
        else:
            self.R_imu_correction = Rotation.from_euler('x', 0, degrees=True)

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # Generate filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'comparison_{timestamp}.csv')

        # Open CSV file
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header
        self.csv_writer.writerow([
            'timestamp',
            # UKFM output
            'ukfm_x', 'ukfm_y', 'ukfm_z',
            'ukfm_roll', 'ukfm_pitch', 'ukfm_yaw',
            'ukfm_vx', 'ukfm_vy', 'ukfm_vz',
            # EKF output
            'ekf_x', 'ekf_y', 'ekf_z',
            'ekf_roll', 'ekf_pitch', 'ekf_yaw',
            'ekf_vx', 'ekf_vy', 'ekf_vz',
            # 3-Sensor Integration (IMU+DVL+Pressure)
            'simple_x', 'simple_y', 'simple_z',
            'simple_vx', 'simple_vy', 'simple_vz',
            # IMU
            'imu_roll', 'imu_pitch', 'imu_yaw',
            'imu_ax', 'imu_ay', 'imu_az',
            'imu_wx', 'imu_wy', 'imu_wz',
            # Depth
            'depth',
            # DVL
            'dvl_vx', 'dvl_vy', 'dvl_vz', 'dvl_altitude', 'dvl_valid',
            # ArUco
            'aruco_x', 'aruco_y', 'aruco_z', 'aruco_detected'
        ])

        # Data storage
        self.ukfm_data = None
        self.ekf_data = None
        self.imu_data = None
        self.depth = 0.0
        self.dvl_data = None
        self.aruco_data = None
        self.count = 0

        # 3-Sensor Integration state (IMU + DVL + Pressure)
        self.simple_position = np.array([0.0, 0.0, 0.0])
        self.simple_velocity = np.array([0.0, 0.0, 0.0])
        self.simple_last_time = None
        self.simple_initialized = False  # Flag to sync with UKFM start position
        self.gravity = np.array([0.0, 0.0, 9.81])
        self.aruco_alpha = 0.15  # Alpha blending factor for ArUco correction

        # QoS for DVL (BEST_EFFORT to match publisher)
        dvl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.ukfm_sub = self.create_subscription(
            Odometry, '/ukfm/odom', self.ukfm_callback, 10)
        self.ekf_sub = self.create_subscription(
            Odometry, '/ekf/odom', self.ekf_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.pressure_sub = self.create_subscription(
            Float64, '/pressure', self.pressure_callback, 10)
        self.aruco_sub = self.create_subscription(
            PoseStamped, '/aruco/pose_6dof', self.aruco_callback, 10)

        if DVL_AVAILABLE:
            self.dvl_sub = self.create_subscription(
                DVL, '/dvl/data', self.dvl_callback, dvl_qos)

        # Timer for logging
        self.timer = self.create_timer(1.0 / log_rate, self.log_data)

        self.get_logger().info(f'Comparison Data Logger started')
        self.get_logger().info(f'  - Output: {self.csv_path}')
        self.get_logger().info(f'  - Rate: {log_rate} Hz')
        self.get_logger().info(f'  - Logging: UKFM, EKF, 3-Sensor Integration')

    def ukfm_callback(self, msg):
        """Store UKFM odometry data"""
        q = msg.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        euler = rot.as_euler('xyz')

        self.ukfm_data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'roll': euler[0],
            'pitch': euler[1],
            'yaw': euler[2],
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'vz': msg.twist.twist.linear.z,
            'stamp': msg.header.stamp
        }

    def ekf_callback(self, msg):
        """Store EKF odometry data"""
        q = msg.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        euler = rot.as_euler('xyz')

        self.ekf_data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'roll': euler[0],
            'pitch': euler[1],
            'yaw': euler[2],
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'vz': msg.twist.twist.linear.z,
            'stamp': msg.header.stamp
        }

    def imu_callback(self, msg):
        """Store IMU data with coordinate transformation"""
        # Raw IMU data
        quat_raw = Rotation.from_quat([msg.orientation.x, msg.orientation.y,
                                         msg.orientation.z, msg.orientation.w])
        accel_raw = np.array([msg.linear_acceleration.x,
                               msg.linear_acceleration.y,
                               msg.linear_acceleration.z])
        omega_raw = np.array([msg.angular_velocity.x,
                               msg.angular_velocity.y,
                               msg.angular_velocity.z])

        # Apply IMU correction
        quat_corrected = self.R_imu_correction * quat_raw
        R_correction = self.R_imu_correction.as_matrix()
        accel_corrected = R_correction @ accel_raw
        omega_corrected = R_correction @ omega_raw

        try:
            euler = quat_corrected.as_euler('xyz')
        except Exception:
            euler = [0, 0, 0]

        self.imu_data = {
            'roll': euler[0],
            'pitch': euler[1],
            'yaw': euler[2],
            'ax': accel_corrected[0],
            'ay': accel_corrected[1],
            'az': accel_corrected[2],
            'wx': omega_corrected[0],
            'wy': omega_corrected[1],
            'wz': omega_corrected[2],
        }

    def pressure_callback(self, msg):
        """Convert pressure (mbar) to depth (meters)"""
        pressure_mbar = msg.data
        atm_mbar = 1013.25
        self.depth = (pressure_mbar - atm_mbar) * 100.0 / (1025.0 * 9.81)

    def dvl_callback(self, msg):
        """Store DVL data with coordinate transformation"""
        # Raw DVL velocity
        dvl_vel_raw = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])

        # Transform to robot body frame
        dvl_vel_body = self.R_dvl_to_body @ dvl_vel_raw

        vx_transformed = dvl_vel_body[0]
        vy_transformed = dvl_vel_body[1]
        vz_transformed = dvl_vel_body[2]

        self.dvl_data = {
            'vx': vx_transformed,
            'vy': vy_transformed,
            'vz': vz_transformed,
            'altitude': msg.altitude,
            'valid': msg.velocity_valid
        }

    def aruco_callback(self, msg):
        """Store ArUco detection"""
        self.aruco_data = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'detected': True
        }

    def log_data(self):
        """Write data to CSV"""
        if self.ukfm_data is None and self.ekf_data is None:
            return

        self.count += 1

        # Get timestamp (use UKFM or EKF timestamp as reference)
        if self.ukfm_data is not None:
            stamp = self.ukfm_data['stamp']
        elif self.ekf_data is not None:
            stamp = self.ekf_data['stamp']
        else:
            return
        timestamp = stamp.sec + stamp.nanosec * 1e-9

        # Initialize simple position to match UKFM start position
        if not self.simple_initialized and self.ukfm_data is not None:
            self.simple_position[0] = self.ukfm_data['x']
            self.simple_position[1] = self.ukfm_data['y']
            self.simple_position[2] = self.ukfm_data['z']
            self.simple_initialized = True
            self.get_logger().info(
                f'3-Sensor initialized to UKFM position: '
                f'({self.simple_position[0]:.4f}, {self.simple_position[1]:.4f}, {self.simple_position[2]:.4f})'
            )

        # 3-Sensor Integration (IMU + DVL + Pressure)
        # IMU: Orientation (for coordinate transformation)
        # DVL: Velocity (body frame -> world frame via IMU)
        # Pressure: Z position
        current_time = self.get_clock().now()
        if self.simple_last_time is not None and self.imu_data is not None:
            dt = (current_time - self.simple_last_time).nanoseconds * 1e-9
            if 0 < dt < 1.0:
                # Constant velocity model: velocity remains constant until updated by DVL
                # No IMU acceleration integration to prevent drift

                # Update velocity with DVL when valid
                if self.dvl_data and self.dvl_data['valid']:
                    # DVL velocity is in body frame, transform to world frame
                    # using full IMU rotation (same as UKFM: R_imu @ v_body)
                    roll = self.imu_data['roll']
                    pitch = self.imu_data['pitch']
                    yaw = self.imu_data['yaw']

                    R_imu = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

                    v_body = np.array([
                        self.dvl_data['vx'],
                        self.dvl_data['vy'],
                        self.dvl_data['vz']
                    ])

                    # Body to world: v_world = R @ v_body
                    v_world = R_imu @ v_body

                    self.simple_velocity[0] = v_world[0]
                    self.simple_velocity[1] = v_world[1]
                    self.simple_velocity[2] = v_world[2]
                # When DVL invalid, velocity stays constant (no IMU acceleration)

                # Integrate position from velocity
                self.simple_position[0] += self.simple_velocity[0] * dt
                self.simple_position[1] += self.simple_velocity[1] * dt

                # Use pressure sensor directly for Z
                self.simple_position[2] = self.depth

                # ArUco position correction with alpha blending (smooth, no jumps)
                if self.aruco_data and self.aruco_data.get('detected'):
                    aruco_x = self.aruco_data['x']
                    aruco_y = self.aruco_data['y']
                    alpha = self.aruco_alpha
                    self.simple_position[0] += alpha * (aruco_x - self.simple_position[0])
                    self.simple_position[1] += alpha * (aruco_y - self.simple_position[1])

        self.simple_last_time = current_time

        # Prepare row
        row = [f'{timestamp:.3f}']

        # UKFM
        if self.ukfm_data:
            row.extend([
                f'{self.ukfm_data["x"]:.4f}',
                f'{self.ukfm_data["y"]:.4f}',
                f'{self.ukfm_data["z"]:.4f}',
                f'{np.degrees(self.ukfm_data["roll"]):.2f}',
                f'{np.degrees(self.ukfm_data["pitch"]):.2f}',
                f'{np.degrees(self.ukfm_data["yaw"]):.2f}',
                f'{self.ukfm_data["vx"]:.4f}',
                f'{self.ukfm_data["vy"]:.4f}',
                f'{self.ukfm_data["vz"]:.4f}',
            ])
        else:
            row.extend([''] * 9)

        # EKF
        if self.ekf_data:
            row.extend([
                f'{self.ekf_data["x"]:.4f}',
                f'{self.ekf_data["y"]:.4f}',
                f'{self.ekf_data["z"]:.4f}',
                f'{np.degrees(self.ekf_data["roll"]):.2f}',
                f'{np.degrees(self.ekf_data["pitch"]):.2f}',
                f'{np.degrees(self.ekf_data["yaw"]):.2f}',
                f'{self.ekf_data["vx"]:.4f}',
                f'{self.ekf_data["vy"]:.4f}',
                f'{self.ekf_data["vz"]:.4f}',
            ])
        else:
            row.extend([''] * 9)

        # 3-Sensor Integration
        row.extend([
            f'{self.simple_position[0]:.4f}',
            f'{self.simple_position[1]:.4f}',
            f'{self.simple_position[2]:.4f}',
            f'{self.simple_velocity[0]:.4f}',
            f'{self.simple_velocity[1]:.4f}',
            f'{self.simple_velocity[2]:.4f}',
        ])

        # IMU
        if self.imu_data:
            row.extend([
                f'{np.degrees(self.imu_data["roll"]):.2f}',
                f'{np.degrees(self.imu_data["pitch"]):.2f}',
                f'{np.degrees(self.imu_data["yaw"]):.2f}',
                f'{self.imu_data["ax"]:.4f}',
                f'{self.imu_data["ay"]:.4f}',
                f'{self.imu_data["az"]:.4f}',
                f'{self.imu_data["wx"]:.4f}',
                f'{self.imu_data["wy"]:.4f}',
                f'{self.imu_data["wz"]:.4f}',
            ])
        else:
            row.extend([''] * 9)

        # Depth
        row.append(f'{self.depth:.4f}')

        # DVL
        if self.dvl_data:
            row.extend([
                f'{self.dvl_data["vx"]:.4f}',
                f'{self.dvl_data["vy"]:.4f}',
                f'{self.dvl_data["vz"]:.4f}',
                f'{self.dvl_data["altitude"]:.4f}',
                '1' if self.dvl_data["valid"] else '0',
            ])
        else:
            row.extend([''] * 5)

        # ArUco
        if self.aruco_data and self.aruco_data.get('detected'):
            row.extend([
                f'{self.aruco_data["x"]:.4f}',
                f'{self.aruco_data["y"]:.4f}',
                f'{self.aruco_data["z"]:.4f}',
                '1',
            ])
            self.aruco_data['detected'] = False
        else:
            row.extend(['', '', '', '0'])

        self.csv_writer.writerow(row)

        # Flush periodically
        if self.count % 10 == 0:
            self.csv_file.flush()

        # Log statistics periodically
        if self.count % 100 == 0:
            ukfm_str = f'({self.ukfm_data["x"]:.2f}, {self.ukfm_data["y"]:.2f})' if self.ukfm_data else 'N/A'
            ekf_str = f'({self.ekf_data["x"]:.2f}, {self.ekf_data["y"]:.2f})' if self.ekf_data else 'N/A'
            self.get_logger().info(
                f'Logged {self.count} samples | UKFM: {ukfm_str} | '
                f'EKF: {ekf_str} | '
                f'3-Sensor: ({self.simple_position[0]:.2f}, {self.simple_position[1]:.2f})'
            )

    def destroy_node(self):
        """Clean up on shutdown"""
        self.csv_file.close()
        self.get_logger().info(f'Saved {self.count} samples to {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ComparisonDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
