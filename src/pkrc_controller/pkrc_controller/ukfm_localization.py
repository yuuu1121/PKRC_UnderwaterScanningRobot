#!/usr/bin/env python3
"""
UKF-M (Unscented Kalman Filter on Manifolds) Localization for PKRC Robot

Based on: MDPI Sensors 2024 - "A Multi-Sensor Fusion Underwater Localization
Method Based on Unscented Kalman Filter on Manifolds" (Wang et al.)

Implementation:
    - SE2(3) Lie group state representation with left-equivariant retraction
    - Direct IMU orientation (no gyro integration) + constant velocity prediction
    - DVL body-frame velocity, depth, and ArUco position updates via UKF
    - ArUco low-pass filter for noise rejection
    - Merwe scaled sigma points with manifold-aware mean computation
    - Marker timeout: stops publishing odom when ArUco is not visible

Note: IMU acceleration integration (Wang et al. Eq. 3) works in simulation but NOT
on the real robot. Real MEMS IMU accelerometer bias (~0.03 m/s²) causes velocity
drift between DVL updates (~0.5s gaps), inflating path length by ~25%.

State on SE2(3) Lie Group:
    X = [R, v, p] where R in SO(3), v in R^3, p in R^3

Sensors:
    - IMU: /imu/data (GV7-INS) - orientation (after 180 deg x-axis correction)
    - DVL: /dvl/data (DVL A50) - body-frame velocity (after R_dvl_to_body)
    - Depth: /bar10xt/pressure (Bar10XT, FluidPressure) - depth from pressure
    - ArUco: /aruco/pose_array or /aruco/pose_6dof - absolute position (low-pass filtered)

Publishes:
    - /ukfm/odom (Odometry)
    - /ukfm/path (Path)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import cholesky, block_diag
import threading

# DVL A50 message
try:
    from dvl_msgs.msg import DVL
    DVL_AVAILABLE = True
except ImportError:
    DVL_AVAILABLE = False
    print("Warning: dvl_msgs not found, DVL fusion disabled")


class SE23State:
    """
    SE2(3) Lie Group State (Paper Eq. 1)

    State matrix:
    X = [R   v   p]
        [0   1   0]
        [0   0   1]

    where R in SO(3), v in R^3, p in R^3
    """
    def __init__(self, R=None, v=None, p=None):
        self.R = R if R is not None else np.eye(3)  # Rotation matrix
        self.v = v if v is not None else np.zeros(3)  # Velocity
        self.p = p if p is not None else np.zeros(3)  # Position

    def copy(self):
        return SE23State(self.R.copy(), self.v.copy(), self.p.copy())

    @staticmethod
    def exp(xi):
        """
        Exponential map: se2(3) -> SE2(3)
        xi = [xi_R, xi_v, xi_p] in R^9
        """
        xi_R = xi[0:3]  # rotation vector
        xi_v = xi[3:6]  # velocity perturbation
        xi_p = xi[6:9]  # position perturbation

        # SO(3) exponential map
        theta = np.linalg.norm(xi_R)
        if theta < 1e-10:
            R = np.eye(3)
            J = np.eye(3)
        else:
            axis = xi_R / theta
            K = skew(axis)
            R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K
            # Left Jacobian (Paper Eq. 13)
            J = np.eye(3) + (1 - np.cos(theta)) / theta * K + (theta - np.sin(theta)) / theta * K @ K

        v = J @ xi_v
        p = J @ xi_p

        return SE23State(R, v, p)

    @staticmethod
    def log(X):
        """
        Logarithmic map: SE2(3) -> se2(3)
        """
        # SO(3) logarithm
        R = X.R
        trace_R = np.trace(R)
        trace_R = np.clip(trace_R, -1.0, 3.0)

        if trace_R >= 3.0 - 1e-10:
            xi_R = np.zeros(3)
            J_inv = np.eye(3)
        else:
            theta = np.arccos((trace_R - 1) / 2)
            if abs(theta) < 1e-10:
                xi_R = np.zeros(3)
                J_inv = np.eye(3)
            else:
                K = (R - R.T) / (2 * np.sin(theta))
                axis = np.array([K[2, 1], K[0, 2], K[1, 0]])
                xi_R = theta * axis
                # Inverse left Jacobian
                K_axis = skew(axis)
                J_inv = np.eye(3) - theta / 2 * K_axis + (1 - theta / (2 * np.tan(theta / 2))) * K_axis @ K_axis

        xi_v = J_inv @ X.v
        xi_p = J_inv @ X.p

        return np.concatenate([xi_R, xi_v, xi_p])


def skew(v):
    """Skew-symmetric matrix from vector"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def vee(S):
    """Vector from skew-symmetric matrix"""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


class UKFMLocalization(Node):
    def __init__(self):
        super().__init__('ukfm_localization')

        # Parameters
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('pressure_topic', '/bar10xt/pressure')
        self.declare_parameter('dvl_topic', '/dvl/data')
        self.declare_parameter('aruco_topic', '/aruco/pose_array')
        self.declare_parameter('use_dvl', True)
        self.declare_parameter('water_density', 1025.0)
        self.declare_parameter('atmospheric_pressure_pa', 101325.0)

        # IMU mounting configuration
        # If IMU is mounted upside-down, set to true to apply 180° rotation correction
        self.declare_parameter('imu_inverted', True)  # IMU mounted upside-down
        self.declare_parameter('imu_rotation_axis', 'x')  # Rotation axis: 'x', 'y', or 'z'
        self.declare_parameter('imu_filter_alpha', 1.0)  # 1.0 = no filter (raw IMU)

        # DVL mounting configuration (sideways, facing wall)
        # ArUco correction (disabled by default - use ArUco for offline eval only)
        self.declare_parameter('use_aruco_correction', True)

        # ArUco filter
        self.declare_parameter('aruco_filter_alpha', 0.3)

        # Marker timeout - if no marker visible for this duration, stop publishing odom
        self.declare_parameter('marker_timeout', 5.0)

        # UKF parameters (Paper Section 3.3)
        self.declare_parameter('ukf_alpha', 0.1)  # Spread of sigma points
        self.declare_parameter('ukf_beta', 2.0)   # Prior knowledge (2 optimal for Gaussian)
        self.declare_parameter('ukf_kappa', 0.0)  # Secondary scaling

        # Process noise (Paper Eq. 3)
        self.declare_parameter('process_noise_rotation', [0.01, 0.01, 0.01])
        self.declare_parameter('process_noise_velocity', [0.1, 0.1, 0.1])
        self.declare_parameter('process_noise_position', [0.1, 0.1, 0.01])

        # Measurement noise
        self.declare_parameter('dvl_noise', [0.01, 0.01, 0.01])
        self.declare_parameter('depth_noise', 0.02)
        self.declare_parameter('aruco_position_noise', [0.01, 0.01, 0.01])

        # Marker map (default: single marker at origin)
        self.declare_parameter('marker_map_ids', [0])
        self.declare_parameter('marker_map_x', [0.0])
        self.declare_parameter('marker_map_y', [0.0])
        self.declare_parameter('marker_map_z', [0.0])

        # Get parameters
        self.frequency = self.get_parameter('frequency').value
        self.dt = 1.0 / self.frequency
        self.use_dvl = self.get_parameter('use_dvl').value and DVL_AVAILABLE
        self.water_density = self.get_parameter('water_density').value
        self.atm_pressure_pa = self.get_parameter('atmospheric_pressure_pa').value
        self.aruco_filter_alpha = self.get_parameter('aruco_filter_alpha').value
        self.marker_timeout = self.get_parameter('marker_timeout').value
        self.use_aruco_correction = self.get_parameter('use_aruco_correction').value
        self.imu_filter_alpha = self.get_parameter('imu_filter_alpha').value

        # IMU to Robot body frame transformation (if IMU is inverted)
        imu_inverted = self.get_parameter('imu_inverted').value
        imu_rotation_axis = self.get_parameter('imu_rotation_axis').value

        if imu_inverted:
            # 180 degree rotation around specified axis
            if imu_rotation_axis == 'x':
                # Rotation around X-axis: [1,0,0; 0,-1,0; 0,0,-1]
                self.R_imu_correction = Rotation.from_euler('x', 180, degrees=True)
            elif imu_rotation_axis == 'y':
                # Rotation around Y-axis: [-1,0,0; 0,1,0; 0,0,-1]
                self.R_imu_correction = Rotation.from_euler('y', 180, degrees=True)
            elif imu_rotation_axis == 'z':
                # Rotation around Z-axis: [-1,0,0; 0,-1,0; 0,0,1]
                self.R_imu_correction = Rotation.from_euler('z', 180, degrees=True)
            else:
                self.get_logger().warn(f'Invalid imu_rotation_axis: {imu_rotation_axis}, using x')
                self.R_imu_correction = Rotation.from_euler('x', 180, degrees=True)
        else:
            self.R_imu_correction = Rotation.from_euler('x', 0, degrees=True)  # Identity

        # DVL to Robot body frame transformation
        # DVL mounting: LED → Robot -Y (right), Transducers → Robot +X (forward)
        # DVL X (LED) → Robot -Y, DVL Y → Robot +Z, DVL Z (transducers) → Robot +X
        self.R_dvl_to_body = np.array([
            [0, 0, 1],   # robot_vx (surge) = dvl_vz
            [-1, 0, 0],  # robot_vy (sway) = -dvl_vx
            [0, 1, 0],   # robot_vz (heave) = dvl_vy
        ])

        # UKF parameters
        self.alpha = self.get_parameter('ukf_alpha').value
        self.beta = self.get_parameter('ukf_beta').value
        self.kappa = self.get_parameter('ukf_kappa').value

        # Build marker map
        map_ids = self.get_parameter('marker_map_ids').value
        map_x = self.get_parameter('marker_map_x').value
        map_y = self.get_parameter('marker_map_y').value
        map_z = self.get_parameter('marker_map_z').value
        self.marker_map = {}
        for i, mid in enumerate(map_ids):
            self.marker_map[int(mid)] = np.array([
                float(map_x[i]) if i < len(map_x) else 0.0,
                float(map_y[i]) if i < len(map_y) else 0.0,
                float(map_z[i]) if i < len(map_z) else 0.0
            ])

        # State dimension (9D: rotation 3, velocity 3, position 3)
        self.n = 9

        # Initialize state on SE2(3)
        self.X = SE23State()

        # Covariance (in tangent space)
        self.P = np.diag([
            0.01, 0.01, 0.01,   # Rotation (IMU is reliable)
            0.01, 0.01, 0.01,   # Velocity
            0.1, 0.1, 0.05      # Position (moderate uncertainty)
        ])

        # Process noise covariance
        Q_rot = np.diag(self.get_parameter('process_noise_rotation').value)
        Q_vel = np.diag(self.get_parameter('process_noise_velocity').value)
        Q_pos = np.diag(self.get_parameter('process_noise_position').value)
        self.Q = block_diag(Q_rot, Q_vel, Q_pos)

        # Measurement noise
        self.R_dvl = np.diag(self.get_parameter('dvl_noise').value)
        self.R_depth = np.array([[self.get_parameter('depth_noise').value]])
        self.R_aruco = np.diag(self.get_parameter('aruco_position_noise').value)

        # Compute UKF weights (Paper Eq. 20)
        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n
        self.gamma = np.sqrt(self.n + self.lambda_)

        self.W_m = np.zeros(2 * self.n + 1)
        self.W_c = np.zeros(2 * self.n + 1)
        self.W_m[0] = self.lambda_ / (self.n + self.lambda_)
        self.W_c[0] = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2 * self.n + 1):
            self.W_m[i] = 1 / (2 * (self.n + self.lambda_))
            self.W_c[i] = 1 / (2 * (self.n + self.lambda_))

        # Thread lock
        self.lock = threading.Lock()

        # Sensor data
        self.imu_omega = np.zeros(3)  # Angular velocity
        self.imu_accel = np.zeros(3)  # Linear acceleration
        self.imu_yaw = 0.0
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.depth = 0.0
        self.dvl_velocity = np.zeros(3)  # In robot body frame (after transformation)
        self.dvl_altitude = 0.0  # Distance to wall
        self.dvl_valid = False
        self.dvl_last_time = None

        # ArUco
        self.aruco_position = None
        self.aruco_new = False  # Flag: only update when new detection arrives
        self.aruco_update_count = 0  # Skip correction limit for first detection
        self.aruco_last_time = None
        self.aruco_filtered_x = None
        self.aruco_filtered_y = None

        # Timing
        self.last_update_time = None
        self.initialized = False
        self.imu_received = False  # Wait for first IMU before predicting
        self.dvl_received = False  # Wait for first DVL before velocity integration

        # Output smoothing (EMA on published position)
        self.output_alpha = 0.4  # lower = smoother, 1.0 = no smoothing
        self.pub_x = None
        self.pub_y = None
        self.pub_z = None

        # Path visualization
        self.path = Path()
        self.path.header.frame_id = 'world'

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/ukfm/odom', 10)
        self.validated_odom_pub = self.create_publisher(Odometry, '/ukfm/odom_validated', 10)
        self.path_pub = self.create_publisher(Path, '/ukfm/path', 10)
        self.wall_dist_pub = self.create_publisher(Float32, '/ukfm/wall_distance', 10)

        # QoS for DVL (BEST_EFFORT to match publisher)
        dvl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.imu_callback, 10)
        # Bar10XT 는 sensor_data QoS(BEST_EFFORT) 로 발행 — RELIABLE 로 구독하면 안 붙는다
        self.pressure_sub = self.create_subscription(
            FluidPressure, self.get_parameter('pressure_topic').value,
            self.pressure_callback, dvl_qos)
        self.aruco_sub = self.create_subscription(
            PoseArray, self.get_parameter('aruco_topic').value, self.aruco_callback, 10)

        # ArUco 6DOF pose subscriber (single marker)
        self.aruco_6dof_sub = self.create_subscription(
            PoseStamped, '/aruco/pose_6dof', self.aruco_6dof_callback, 10)

        if self.use_dvl:
            self.dvl_sub = self.create_subscription(
                DVL, self.get_parameter('dvl_topic').value, self.dvl_callback, dvl_qos)
            self.get_logger().info('DVL fusion enabled (mounted sideways, BEST_EFFORT QoS)')

        # Main update timer
        self.timer = self.create_timer(self.dt, self.ukfm_update)

        self.get_logger().info('UKF-M Localization started for PKRC Robot')
        self.get_logger().info(f'  - Frequency: {self.frequency} Hz')
        self.get_logger().info(f'  - IMU inverted: {imu_inverted} (rotation axis: {imu_rotation_axis})')
        self.get_logger().info(f'  - Marker map: {len(self.marker_map)} markers')

    def imu_callback(self, msg):
        """Process IMU data with coordinate transformation for inverted IMU"""
        with self.lock:
            # Raw IMU data
            omega_raw = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])
            accel_raw = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
            quat_raw = Rotation.from_quat([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])

            # Apply IMU correction (if IMU is inverted)
            quat_corrected = self.R_imu_correction * quat_raw
            R_correction = self.R_imu_correction.as_matrix()

            # Transform angular velocity and acceleration
            self.imu_omega = R_correction @ omega_raw
            self.imu_accel = R_correction @ accel_raw

            # Extract corrected orientation with low-pass filter
            try:
                euler = quat_corrected.as_euler('xyz')
                if self.imu_received:
                    # Low-pass filter on roll/pitch (small angles, no wrapping)
                    a = self.imu_filter_alpha
                    self.imu_roll = a * euler[0] + (1 - a) * self.imu_roll
                    self.imu_pitch = a * euler[1] + (1 - a) * self.imu_pitch
                    # Circular low-pass for yaw (handles angle wrapping)
                    yaw_diff = euler[2] - self.imu_yaw
                    yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
                    self.imu_yaw = self.imu_yaw + a * yaw_diff
                else:
                    # First IMU: initialize directly
                    self.imu_roll = euler[0]
                    self.imu_pitch = euler[1]
                    self.imu_yaw = euler[2]
            except Exception:
                pass

            # Initialize rotation state on first IMU
            if not self.imu_received:
                self.imu_received = True
                R_init = Rotation.from_euler('xyz', [self.imu_roll, self.imu_pitch, self.imu_yaw]).as_matrix()
                self.X.R = R_init
                self.get_logger().info(
                    f'IMU initialized: roll={np.degrees(self.imu_roll):.1f} '
                    f'pitch={np.degrees(self.imu_pitch):.1f} '
                    f'yaw={np.degrees(self.imu_yaw):.1f}'
                )

    def pressure_callback(self, msg):
        """Convert absolute pressure [Pa] to depth (meters).
        fluid_pressure 는 절대압 — keyboard_control_teleop 과 같은 식을 쓴다.
        """
        with self.lock:
            gauge_pa = msg.fluid_pressure - self.atm_pressure_pa
            self.depth = max(0.0, gauge_pa / (self.water_density * 9.81))

    def dvl_callback(self, msg):
        """Process DVL A50 data with coordinate transformation

        DVL node publishes raw sensor data. Transform to robot body frame here.
        DVL mounting: DVL Z -> Robot X (surge), DVL X -> Robot Y (sway), DVL Y -> Robot Z (heave)
        """
        with self.lock:
            if msg.velocity_valid:
                # Raw DVL velocity in DVL sensor frame
                dvl_vel_raw = np.array([
                    msg.velocity.x,
                    msg.velocity.y,
                    msg.velocity.z
                ])

                # Transform to robot body frame
                self.dvl_velocity = self.R_dvl_to_body @ dvl_vel_raw
                self.dvl_valid = True

                # Altitude is distance to wall (through DVL's forward direction)
                self.dvl_altitude = msg.altitude

                # Publish wall distance
                wall_dist_msg = Float32()
                wall_dist_msg.data = float(self.dvl_altitude)
                self.wall_dist_pub.publish(wall_dist_msg)
            else:
                self.dvl_valid = False

            self.dvl_last_time = self.get_clock().now()

            if not self.dvl_received:
                self.dvl_received = True
                self.get_logger().info('DVL first data received, enabling velocity integration')

    def aruco_callback(self, msg):
        """Process ArUco detections from pose array"""
        if len(msg.poses) == 0:
            return

        try:
            parts = msg.header.frame_id.split(',')
            marker_ids = []
            for part in parts:
                if ':' in part:
                    mid = int(part.split(':')[0])
                else:
                    mid = int(part)
                marker_ids.append(mid)
        except ValueError:
            return

        if len(marker_ids) != len(msg.poses):
            return

        with self.lock:
            positions = []
            weights = []

            for i, pose in enumerate(msg.poses):
                marker_id = marker_ids[i]
                if marker_id not in self.marker_map:
                    continue

                marker_pos = self.marker_map[marker_id]
                tvec = np.array([pose.position.x, pose.position.y, pose.position.z])
                robot_x = marker_pos[0] + tvec[0]
                robot_y = marker_pos[1] + tvec[1]
                positions.append([robot_x, robot_y])
                weights.append(1.0 / (np.linalg.norm(tvec) + 0.1))

            if len(positions) == 0:
                return

            positions = np.array(positions)
            weights = np.array(weights)
            weights /= np.sum(weights)

            robot_x_raw = np.sum(weights * positions[:, 0])
            robot_y_raw = np.sum(weights * positions[:, 1])

            # Low-pass filter
            alpha = self.aruco_filter_alpha
            if self.aruco_filtered_x is None:
                self.aruco_filtered_x = robot_x_raw
                self.aruco_filtered_y = robot_y_raw
            else:
                self.aruco_filtered_x = alpha * robot_x_raw + (1 - alpha) * self.aruco_filtered_x
                self.aruco_filtered_y = alpha * robot_y_raw + (1 - alpha) * self.aruco_filtered_y

            self.aruco_position = np.array([self.aruco_filtered_x, self.aruco_filtered_y, self.depth])
            self.aruco_new = True
            self.aruco_last_time = self.get_clock().now()

    def aruco_6dof_callback(self, msg):
        """Process single ArUco 6DOF pose with low-pass filter"""
        with self.lock:
            robot_x_raw = msg.pose.position.x
            robot_y_raw = msg.pose.position.y

            # Low-pass filter
            alpha = self.aruco_filter_alpha
            if self.aruco_filtered_x is None:
                self.aruco_filtered_x = robot_x_raw
                self.aruco_filtered_y = robot_y_raw
            else:
                self.aruco_filtered_x = alpha * robot_x_raw + (1 - alpha) * self.aruco_filtered_x
                self.aruco_filtered_y = alpha * robot_y_raw + (1 - alpha) * self.aruco_filtered_y

            self.aruco_position = np.array([self.aruco_filtered_x, self.aruco_filtered_y, self.depth])
            self.aruco_new = True
            self.aruco_last_time = self.get_clock().now()

    def generate_sigma_points(self, X, P):
        """Generate sigma points using retraction"""
        sigma_points = []

        # Center point
        sigma_points.append(X.copy())

        # Ensure P is positive definite
        P_reg = (P + P.T) / 2
        eigvals = np.linalg.eigvalsh(P_reg)
        if np.min(eigvals) < 1e-8:
            P_reg += np.eye(self.n) * (1e-8 - np.min(eigvals) + 1e-8)

        # Square root of scaled covariance
        try:
            L = cholesky(P_reg, lower=True)
        except np.linalg.LinAlgError:
            self.get_logger().warn('Cholesky failed, using eigendecomposition')
            eigvals, eigvecs = np.linalg.eigh(P_reg)
            eigvals = np.maximum(eigvals, 1e-8)
            L = eigvecs @ np.diag(np.sqrt(eigvals))

        # Sigma points
        for i in range(self.n):
            xi_plus = self.gamma * L[:, i]
            xi_minus = -self.gamma * L[:, i]

            sigma_points.append(self.retraction(X, xi_plus))
            sigma_points.append(self.retraction(X, xi_minus))

        return sigma_points

    def retraction(self, X, xi):
        """Left-equivariant retraction"""
        delta = SE23State.exp(xi)

        R_new = X.R @ delta.R
        v_new = X.v + X.R @ delta.v
        p_new = X.p + X.R @ delta.p

        return SE23State(R_new, v_new, p_new)

    def inv_retraction(self, X, Y):
        """Inverse retraction"""
        R_inv = X.R.T
        R_diff = R_inv @ Y.R
        v_diff = R_inv @ (Y.v - X.v)
        p_diff = R_inv @ (Y.p - X.p)

        diff_state = SE23State(R_diff, v_diff, p_diff)
        return SE23State.log(diff_state)

    def predict(self, dt):
        """UKF-M Prediction step

        Uses constant velocity model (DVL handles velocity via update step).
        IMU is used ONLY for orientation (R_imu), not for acceleration integration.
        Real MEMS IMU accelerometer bias (~0.03 m/s²) causes velocity drift
        between DVL updates, so acceleration integration is not viable.
        """
        with self.lock:
            X = self.X.copy()
            P = self.P.copy()
            imu_roll = self.imu_roll
            imu_pitch = self.imu_pitch
            imu_yaw = self.imu_yaw

        # Generate sigma points
        sigma_points = self.generate_sigma_points(X, P)

        # Propagate sigma points
        propagated = []

        # Use IMU absolute orientation
        R_imu = Rotation.from_euler('xyz', [imu_roll, imu_pitch, imu_yaw]).as_matrix()

        for Xi in sigma_points:
            R_new = R_imu

            if self.dvl_received:
                # Constant velocity model: DVL provides velocity via update step
                v_new = Xi.v.copy()
                p_new = Xi.p + Xi.v * dt
            else:
                # Before DVL: zero velocity, only update orientation and depth
                v_new = np.zeros(3)
                p_new = Xi.p.copy()

            propagated.append(SE23State(R_new, v_new, p_new))

        # Compute mean
        X_mean = self.compute_mean(propagated)

        # Compute covariance
        P_pred = np.zeros((self.n, self.n))
        for i, Xi in enumerate(propagated):
            xi = self.inv_retraction(X_mean, Xi)
            P_pred += self.W_c[i] * np.outer(xi, xi)

        P_pred += self.Q * dt
        P_pred = self.regularize_covariance(P_pred)

        with self.lock:
            self.X = X_mean
            self.P = P_pred

    def compute_mean(self, sigma_points):
        """Compute mean on SE2(3) manifold"""
        X_mean = sigma_points[0].copy()

        for _ in range(3):
            xi_sum = np.zeros(self.n)
            for i, Xi in enumerate(sigma_points):
                xi = self.inv_retraction(X_mean, Xi)
                xi_sum += self.W_m[i] * xi

            if np.linalg.norm(xi_sum) < 1e-10:
                break
            X_mean = self.retraction(X_mean, xi_sum)

        return X_mean

    def update_dvl(self, measurement):
        """UKF-M Update with DVL measurement"""
        with self.lock:
            X = self.X.copy()
            P = self.P.copy()

        sigma_points = self.generate_sigma_points(X, P)

        # h(X) = R^T * v (world velocity to body frame)
        y_sigma = []
        for Xi in sigma_points:
            y_pred = Xi.R.T @ Xi.v
            y_sigma.append(y_pred)

        y_mean = np.zeros(3)
        for i, y in enumerate(y_sigma):
            y_mean += self.W_m[i] * y

        P_yy = np.zeros((3, 3))
        for i, y in enumerate(y_sigma):
            dy = y - y_mean
            P_yy += self.W_c[i] * np.outer(dy, dy)
        P_yy += self.R_dvl

        P_xy = np.zeros((self.n, 3))
        for i, (Xi, y) in enumerate(zip(sigma_points, y_sigma)):
            xi = self.inv_retraction(X, Xi)
            dy = y - y_mean
            P_xy += self.W_c[i] * np.outer(xi, dy)

        K = P_xy @ np.linalg.inv(P_yy)
        innovation = measurement - y_mean
        xi_update = K @ innovation
        X_new = self.retraction(X, xi_update)
        P_new = P - K @ P_yy @ K.T
        P_new = self.regularize_covariance(P_new)

        with self.lock:
            self.X = X_new
            self.P = P_new

    def update_depth(self, measurement):
        """UKF-M Update with depth measurement"""
        with self.lock:
            X = self.X.copy()
            P = self.P.copy()

        sigma_points = self.generate_sigma_points(X, P)

        y_sigma = []
        for Xi in sigma_points:
            y_sigma.append(np.array([Xi.p[2]]))

        y_mean = np.zeros(1)
        for i, y in enumerate(y_sigma):
            y_mean += self.W_m[i] * y

        P_yy = np.zeros((1, 1))
        for i, y in enumerate(y_sigma):
            dy = y - y_mean
            P_yy += self.W_c[i] * np.outer(dy, dy)
        P_yy += self.R_depth

        P_xy = np.zeros((self.n, 1))
        for i, (Xi, y) in enumerate(zip(sigma_points, y_sigma)):
            xi = self.inv_retraction(X, Xi)
            dy = y - y_mean
            P_xy += self.W_c[i] * np.outer(xi, dy)

        K = P_xy @ np.linalg.inv(P_yy)
        innovation = np.array([measurement]) - y_mean
        xi_update = (K @ innovation).flatten()
        X_new = self.retraction(X, xi_update)
        P_new = P - K @ P_yy @ K.T
        P_new = self.regularize_covariance(P_new)

        with self.lock:
            self.X = X_new
            self.P = P_new

    def update_aruco(self, measurement):
        """UKF-M Update with ArUco position measurement"""
        with self.lock:
            X = self.X.copy()
            P = self.P.copy()

        sigma_points = self.generate_sigma_points(X, P)

        y_sigma = []
        for Xi in sigma_points:
            y_sigma.append(Xi.p.copy())

        y_mean = np.zeros(3)
        for i, y in enumerate(y_sigma):
            y_mean += self.W_m[i] * y

        P_yy = np.zeros((3, 3))
        for i, y in enumerate(y_sigma):
            dy = y - y_mean
            P_yy += self.W_c[i] * np.outer(dy, dy)
        P_yy += self.R_aruco

        P_xy = np.zeros((self.n, 3))
        for i, (Xi, y) in enumerate(zip(sigma_points, y_sigma)):
            xi = self.inv_retraction(X, Xi)
            dy = y - y_mean
            P_xy += self.W_c[i] * np.outer(xi, dy)

        K = P_xy @ np.linalg.inv(P_yy)
        innovation = measurement - y_mean
        xi_update = K @ innovation

        # Adaptive position correction limiting
        # Small gap → small step (smooth), large gap → larger step (fast catch-up)
        # When limited: keep covariance unchanged so filter retains uncertainty
        self.aruco_update_count += 1
        if self.aruco_update_count > 1:
            pos_update = xi_update[6:9]
            pos_norm = np.linalg.norm(pos_update)
            # Adaptive: 30% of gap, clamped to [2cm, 5cm]
            max_pos_step = np.clip(pos_norm * 0.30, 0.02, 0.05)
            if pos_norm > max_pos_step:
                scale = max_pos_step / pos_norm
                xi_update[6:9] = pos_update * scale
                # Don't reduce covariance — filter retains uncertainty for next update
                P_new = P
            else:
                P_new = P - K @ P_yy @ K.T
        else:
            P_new = P - K @ P_yy @ K.T

        X_new = self.retraction(X, xi_update)
        P_new = self.regularize_covariance(P_new)

        with self.lock:
            self.X = X_new
            self.P = P_new

    def regularize_covariance(self, P, min_eig=1e-6):
        """Ensure covariance is symmetric and positive definite"""
        P = (P + P.T) / 2

        if np.any(np.isnan(P)) or np.any(np.isinf(P)):
            self.get_logger().warn('NaN/Inf in covariance, resetting')
            return np.eye(self.n) * 0.1

        eigvals = np.linalg.eigvalsh(P)
        if np.min(eigvals) < min_eig:
            P += np.eye(self.n) * (min_eig - np.min(eigvals) + min_eig)

        return P

    def ukfm_update(self):
        """Main UKF-M update loop

        Always runs prediction + DVL + depth updates (dead reckoning).
        ArUco update only when new detection arrives.
        """
        current_time = self.get_clock().now()

        if self.last_update_time is None:
            self.last_update_time = current_time
            return

        # Skip prediction until IMU data arrives (prevents gravity free-fall)
        if not self.imu_received:
            self.last_update_time = current_time
            return

        dt = (current_time - self.last_update_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        self.last_update_time = current_time

        # Prediction step (always runs — DVL provides velocity for dead reckoning)
        self.predict(dt)

        # DVL update
        if self.use_dvl and self.dvl_last_time is not None:
            with self.lock:
                time_since_dvl = (current_time - self.dvl_last_time).nanoseconds / 1e9
                dvl_vel = self.dvl_velocity.copy()
                dvl_valid = self.dvl_valid
            if time_since_dvl < 0.2 and dvl_valid:
                self.update_dvl(dvl_vel)

        # Depth update
        with self.lock:
            depth_meas = self.depth
        if self.dvl_received:
            self.update_depth(depth_meas)
        else:
            with self.lock:
                self.X.p[2] = depth_meas

        # ArUco update (continuously re-apply last position within timeout)
        with self.lock:
            aruco_pos = self.aruco_position.copy() if self.aruco_position is not None else None
            aruco_time = self.aruco_last_time
        if aruco_pos is not None and self.use_aruco_correction and aruco_time is not None:
            time_since_aruco = (current_time - aruco_time).nanoseconds / 1e9
            if time_since_aruco < self.marker_timeout:
                self.update_aruco(aruco_pos)

        # Publish odometry (always, for trajectory logging)
        self.publish_odometry(current_time)

        # Publish validated odom for BlueBoat (only when ArUco is active)
        aruco_active = (aruco_pos is not None and aruco_time is not None
                        and (current_time - aruco_time).nanoseconds / 1e9 < self.marker_timeout)
        if aruco_active:
            self.publish_validated_odom(current_time)

    def publish_odometry(self, current_time):
        """Publish UKF-M estimated odometry"""
        with self.lock:
            X = self.X.copy()
            P = self.P.copy()

        euler = Rotation.from_matrix(X.R).as_euler('xyz')
        quat = Rotation.from_matrix(X.R).as_quat()

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'

        # Position (direct — correction limiting handles smoothness)
        odom.pose.pose.position.x = float(X.p[0])
        odom.pose.pose.position.y = float(X.p[1])
        odom.pose.pose.position.z = float(X.p[2])

        # Orientation
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])

        # Covariance
        pose_cov = np.zeros((6, 6))
        pose_cov[0:3, 0:3] = P[6:9, 6:9]  # Position covariance
        pose_cov[3:6, 3:6] = P[0:3, 0:3]  # Rotation covariance
        odom.pose.covariance = pose_cov.flatten().tolist()

        # Velocity
        odom.twist.twist.linear.x = float(X.v[0])
        odom.twist.twist.linear.y = float(X.v[1])
        odom.twist.twist.linear.z = float(X.v[2])

        self.odom_pub.publish(odom)
        self._last_odom_msg = odom  # Cache for validated publish

        # Path
        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose
        self.path.poses.append(pose_stamped)

        if len(self.path.poses) > 1000:
            self.path.poses = self.path.poses[-1000:]

        self.path.header.stamp = current_time.to_msg()
        self.path_pub.publish(self.path)

    def publish_validated_odom(self, current_time):
        """Publish validated odom for BlueBoat — only called when ArUco is active."""
        if hasattr(self, '_last_odom_msg'):
            self.validated_odom_pub.publish(self._last_odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UKFMLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
