#!/usr/bin/env python3
"""
ArUco 6DOF Pose Estimator - Active Marker Version
- Subscribes: /bluerov2/up/image_color, /bluerov2/up/camera_info
- Publishes: /aruco/pose_6dof (PoseStamped with full 6DOF pose)
- Uses cv2.aruco.estimatePoseSingleMarkers() for 3D pose estimation
- Supports up to 30 markers (ID 0-29)
- Camera settings optimized for LED marker detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
from scipy.spatial.transform import Rotation
from filterpy.kalman import KalmanFilter


class ArucoDetector6DOF(Node):
    def __init__(self):
        super().__init__('aruco_detector_6dof')

        # Declare parameters - Support up to 30 markers (ID 0-29)
        default_marker_ids = list(range(30))  # [0, 1, 2, ..., 29]
        self.declare_parameter('marker_ids', default_marker_ids)
        self.declare_parameter('marker_size', 0.12)  # 12cm
        self.declare_parameter('image_topic', '/stellarHD/image_raw')
        self.declare_parameter('camera_info_topic', '/stellarHD/camera_info')
        self.declare_parameter('scale_factor', 3.0)
        self.declare_parameter('min_depth', 0.2)   # Minimum valid depth (m)
        self.declare_parameter('max_depth', 15.0)  # Maximum valid depth (m)

        # Camera control parameters (for direct camera access mode)
        self.declare_parameter('use_direct_camera', True)  # Use direct camera instead of ROS topic
        self.declare_parameter('camera_device', 4)  # /dev/video4 (stellarHD)
        self.declare_parameter('camera_width', 1920)
        self.declare_parameter('camera_height', 1080)
        self.declare_parameter('camera_fps', 30)
        # Exposure settings for LED marker detection
        self.declare_parameter('auto_exposure', 1)  # 1=manual, 3=auto
        self.declare_parameter('exposure_time', 1)  # Minimum exposure
        self.declare_parameter('brightness', -64)
        self.declare_parameter('contrast', 64)
        self.declare_parameter('gamma', 90)
        self.declare_parameter('gain', 0)

        # Marker map parameters (30 markers: 5x6 grid, 1.5m spacing)
        default_map_ids = list(range(30))  # [0, 1, 2, ..., 29]
        # 5 columns x 6 rows grid layout
        default_map_x = []
        default_map_y = []
        default_map_z = []
        for i in range(30):
            col = i % 5  # 0-4
            row = i // 5  # 0-5
            default_map_x.append(col * 1.5)  # 1.5m spacing
            default_map_y.append(row * 1.5)
            default_map_z.append(0.0)

        self.declare_parameter('marker_map_ids', default_map_ids)
        self.declare_parameter('marker_map_x', default_map_x)
        self.declare_parameter('marker_map_y', default_map_y)
        self.declare_parameter('marker_map_z', default_map_z)

        # LED preprocessing parameters (adjustable at runtime)
        self.declare_parameter('use_clahe', False)          # Enable CLAHE preprocessing
        self.declare_parameter('clahe_clip_limit', 2.0)     # CLAHE clip limit
        self.declare_parameter('clip_bright_pixels', False) # Enable brightness clipping
        self.declare_parameter('clip_threshold', 255)       # Pixels > this get clipped (50-255)

        marker_ids_param = self.get_parameter('marker_ids').value
        # Handle various input formats: int, list, or string like "[0, 1, 2]"
        if isinstance(marker_ids_param, int):
            self.marker_ids = [marker_ids_param]
        elif isinstance(marker_ids_param, str):
            # Parse string like "[0, 1, 2]" or "0,1,2"
            import ast
            try:
                self.marker_ids = ast.literal_eval(marker_ids_param)
                if isinstance(self.marker_ids, int):
                    self.marker_ids = [self.marker_ids]
            except:
                # Try comma-separated format
                self.marker_ids = [int(x.strip()) for x in marker_ids_param.replace('[','').replace(']','').split(',')]
        else:
            self.marker_ids = [int(x) for x in marker_ids_param]  # Ensure Python int
        self.marker_size = self.get_parameter('marker_size').value

        # Build marker map dictionary: ID -> [x, y, z]
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
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value

        # Direct camera mode parameters
        self.use_direct_camera = self.get_parameter('use_direct_camera').value
        self.camera_device = self.get_parameter('camera_device').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.auto_exposure = self.get_parameter('auto_exposure').value
        self.exposure_time = self.get_parameter('exposure_time').value
        self.brightness = self.get_parameter('brightness').value
        self.contrast = self.get_parameter('contrast').value
        self.gamma = self.get_parameter('gamma').value
        self.gain = self.get_parameter('gain').value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.count = 0
        self.detected = 0
        self.cap = None  # Direct camera capture

        # ArUco setup (based on main.cpp settings)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)  # Same as main.cpp
        # Support both old and new OpenCV API
        try:
            self.params = aruco.DetectorParameters()
            self.use_new_api = True
        except AttributeError:
            self.params = aruco.DetectorParameters_create()
            self.use_new_api = False
        # ArUco detection parameters - tuned for LED markers with circular background
        self.params.adaptiveThreshConstant = 7
        self.params.adaptiveThreshWinSizeMax = 53  # Larger window for LED bloom
        self.params.adaptiveThreshWinSizeMin = 3
        self.params.adaptiveThreshWinSizeStep = 10
        self.params.minMarkerPerimeterRate = 0.005  # Very small markers allowed
        self.params.maxMarkerPerimeterRate = 4.0
        # Relaxed corner detection for rotated/distorted markers
        self.params.polygonalApproxAccuracyRate = 0.1  # More tolerance (default 0.03)
        self.params.minCornerDistanceRate = 0.01  # Closer corners allowed (default 0.05)
        self.params.minDistanceToBorder = 1  # Allow markers near edge
        # Perspective removal - important for angled markers
        self.params.perspectiveRemovePixelPerCell = 8  # More pixels per cell
        self.params.perspectiveRemoveIgnoredMarginPerCell = 0.2  # Ignore 20% margin

        # Create detector AFTER setting parameters (new API only)
        if self.use_new_api:
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.params)
        else:
            self.detector = None

        # Pose averaging buffer for noise reduction (from main.cpp)
        self.averaging_samples = 5  # Number of samples to average
        self.pose_buffer = []  # Buffer for recent pose measurements

        # ===== Kalman Filters for noise reduction (IEEE 2018 paper) =====
        # 1. Corner Kalman Filters - one per marker ID, each tracks 4 corners (8 values: x1,y1,x2,y2,x3,y3,x4,y4)
        self.corner_kfs = {}  # Dict: marker_id -> KalmanFilter

        # 2. Pose Kalman Filter - tracks tvec (3) + quaternion (4) = 7 states
        self.pose_kf = self._create_pose_kf()
        self.pose_kf_initialized = False

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose_6dof', 10)
        self.pose_array_pub = self.create_publisher(PoseArray, '/aruco/pose_array', 10)  # All markers
        self.debug_pub = self.create_publisher(Image, '/aruco/debug_image', 10)  # Main debug image

        # Setup based on mode
        if self.use_direct_camera:
            # Direct camera mode - setup camera with optimized settings
            self._setup_direct_camera()
            # Create timer for camera capture
            self.timer = self.create_timer(1.0 / self.camera_fps, self.timer_callback)
            self.get_logger().info(f'ArUco 6DOF Detector started (Direct Camera Mode)')
        else:
            # ROS topic mode
            self.camera_info_sub = self.create_subscription(
                CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, 10)
            self.get_logger().info(f'ArUco 6DOF Detector started (ROS Topic Mode)')
            self.get_logger().info(f'  - Waiting for camera info...')

        self.get_logger().info(f'  - Marker IDs: {self.marker_ids}')
        self.get_logger().info(f'  - Total markers supported: {len(self.marker_ids)}')
        self.get_logger().info(f'  - Marker size: {self.marker_size}m')
        self.get_logger().info(f'  - Marker map: {len(self.marker_map)} markers')
    def _setup_direct_camera(self):
        """Setup direct camera access with optimized settings for LED marker detection"""
        self.get_logger().info(f'Opening camera device: {self.camera_device}')

        # Open camera
        self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {self.camera_device}')
            return

        # Set resolution and FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        # Set manual exposure mode (auto_exposure=1 means manual)
        auto_exp_set = self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self.auto_exposure)
        self.get_logger().info(f'Manual exposure mode (auto_exposure={self.auto_exposure}): {"Success" if auto_exp_set else "Failed"}')

        # Set minimum exposure time
        exp_set = self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_time)
        self.get_logger().info(f'Exposure time ({self.exposure_time}): {"Success" if exp_set else "Failed"}')

        # Set brightness
        bright_set = self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
        self.get_logger().info(f'Brightness ({self.brightness}): {"Success" if bright_set else "Failed"}')

        # Set contrast
        contrast_set = self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
        self.get_logger().info(f'Contrast ({self.contrast}): {"Success" if contrast_set else "Failed"}')

        # Set gamma
        gamma_set = self.cap.set(cv2.CAP_PROP_GAMMA, self.gamma)
        self.get_logger().info(f'Gamma ({self.gamma}): {"Success" if gamma_set else "Failed"}')

        # Set gain
        gain_set = self.cap.set(cv2.CAP_PROP_GAIN, self.gain)
        self.get_logger().info(f'Gain ({self.gain}): {"Success" if gain_set else "Failed"}')

        # Setup default camera matrix (can be overridden by parameter)
        # Using typical HD camera intrinsics
        fx = self.camera_width * 0.8  # Approximate focal length
        fy = fx
        cx = self.camera_width / 2.0
        cy = self.camera_height / 2.0
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros(5)

        # Scale camera matrix
        self.scaled_camera_matrix = self.camera_matrix.copy()
        self.scaled_camera_matrix[0, 0] *= self.scale_factor
        self.scaled_camera_matrix[1, 1] *= self.scale_factor
        self.scaled_camera_matrix[0, 2] *= self.scale_factor
        self.scaled_camera_matrix[1, 2] *= self.scale_factor

        self.image_width = self.camera_width * self.scale_factor
        self.image_height = self.camera_height * self.scale_factor

        self.get_logger().info(f'Camera opened: {self.camera_width}x{self.camera_height}@{self.camera_fps}fps')
        self.get_logger().info(f'Camera matrix: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}')

    def timer_callback(self):
        """Timer callback for direct camera mode"""
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        # Process frame (same as image_callback but without ROS message conversion)
        self._process_frame(frame)

    def _create_corner_kf(self):
        """Create Kalman Filter for 4 corners (8 states: x1,y1,x2,y2,x3,y3,x4,y4)"""
        kf = KalmanFilter(dim_x=8, dim_z=8)
        # State transition (assume constant position model)
        kf.F = np.eye(8)
        # Measurement matrix
        kf.H = np.eye(8)
        # Process noise - HIGHER = trust prediction less, follow measurement more
        kf.Q = np.eye(8) * 10.0  # Increased from 0.1 (100x more)
        # Measurement noise - LOWER = trust measurement more
        kf.R = np.eye(8) * 1.0  # Reduced from 5.0
        # Initial covariance
        kf.P = np.eye(8) * 100.0
        return kf

    def _create_pose_kf(self):
        """Create Kalman Filter for pose: tvec(3) + quat(4) = 7 states
        Tuned for fast response (low lag) while smoothing high-freq noise
        """
        kf = KalmanFilter(dim_x=7, dim_z=7)
        kf.F = np.eye(7)
        kf.H = np.eye(7)
        # Process noise - VERY HIGH = almost pass-through, minimal smoothing
        kf.Q = np.eye(7) * 50.0  # Very high - trust measurement
        # Measurement noise - LOW = trust measurement
        kf.R = np.eye(7) * 0.5  # Low noise assumption
        # Initial covariance
        kf.P = np.eye(7) * 100.0
        return kf

    def _filter_corners(self, marker_id, corners):
        """Apply Kalman filter to corner coordinates"""
        # Flatten corners to 8 values
        pts = corners[0].flatten()  # shape (8,)

        if marker_id not in self.corner_kfs:
            # Initialize new KF for this marker
            kf = self._create_corner_kf()
            kf.x = pts.reshape(8, 1)
            self.corner_kfs[marker_id] = kf

        kf = self.corner_kfs[marker_id]
        kf.predict()
        kf.update(pts.reshape(8, 1))

        # Return filtered corners in original shape
        filtered = kf.x.flatten().reshape(1, 4, 2)
        return filtered.astype(np.float32)

    def _filter_pose(self, tvec, quat):
        """Apply Kalman filter to pose (tvec + quaternion)"""
        # Combine into 7-element vector
        z = np.concatenate([tvec, quat])

        if not self.pose_kf_initialized:
            self.pose_kf.x = z.reshape(7, 1)
            self.pose_kf_initialized = True

        self.pose_kf.predict()
        self.pose_kf.update(z.reshape(7, 1))

        filtered = self.pose_kf.x.flatten()
        filtered_tvec = filtered[0:3]
        filtered_quat = filtered[3:7]
        # Normalize quaternion
        filtered_quat = filtered_quat / np.linalg.norm(filtered_quat)

        return filtered_tvec, filtered_quat

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            # Extract camera matrix
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)

            # Scale camera matrix for upscaled image
            self.scaled_camera_matrix = self.camera_matrix.copy()
            self.scaled_camera_matrix[0, 0] *= self.scale_factor  # fx
            self.scaled_camera_matrix[1, 1] *= self.scale_factor  # fy
            self.scaled_camera_matrix[0, 2] *= self.scale_factor  # cx
            self.scaled_camera_matrix[1, 2] *= self.scale_factor  # cy

            # Store image dimensions for quality score calculation
            self.image_width = msg.width * self.scale_factor
            self.image_height = msg.height * self.scale_factor

            self.get_logger().info(f'Camera info received:')
            self.get_logger().info(f'  fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}')
            self.get_logger().info(f'  cx={self.camera_matrix[0,2]:.1f}, cy={self.camera_matrix[1,2]:.1f}')

    def image_callback(self, msg):
        """ROS image callback"""
        if self.camera_matrix is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self._process_frame(img, msg.header)

    def _process_frame(self, img, header=None):
        """Process a single frame for marker detection"""
        self.count += 1
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        debug_img = img.copy()

        # Scale up for better detection
        scaled = cv2.resize(gray, None, fx=self.scale_factor, fy=self.scale_factor,
                           interpolation=cv2.INTER_CUBIC)

        # ===== LED marker preprocessing: reduce overexposure bloom =====
        # Read parameters at runtime for real-time tuning
        use_clahe = self.get_parameter('use_clahe').value
        clahe_clip_limit = self.get_parameter('clahe_clip_limit').value
        clip_bright = self.get_parameter('clip_bright_pixels').value
        clip_threshold = self.get_parameter('clip_threshold').value

        # 1. CLAHE to enhance local contrast (helps with bright LED bloom)
        if use_clahe:
            clahe = cv2.createCLAHE(clipLimit=clahe_clip_limit, tileGridSize=(8, 8))
            scaled = clahe.apply(scaled)

        # 2. Clip bright pixels to reduce LED bloom effect
        if clip_bright:
            scaled = np.clip(scaled, 0, clip_threshold)
            scaled = cv2.normalize(scaled, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # Detect markers (support both old and new OpenCV API)
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(scaled)
        else:
            corners, ids, _ = aruco.detectMarkers(scaled, self.aruco_dict, parameters=self.params)

        detected = False
        current_detected_id = None
        all_detected_ids = []  # Initialize for blink tolerance logic
        if ids is not None:
            # Debug: log all detected IDs
            all_ids = [int(x) for x in ids.flatten()]
            if self.count % 100 == 0:  # Log every 100 frames
                self.get_logger().info(f'Detected IDs: {all_ids}, Valid IDs: {self.marker_ids}')

            # Find all markers with matching IDs and select the LARGEST one
            candidates = []
            for i, marker_id in enumerate(ids.flatten()):
                marker_id = int(marker_id)  # Convert numpy.int to Python int
                if marker_id not in self.marker_ids:
                    continue

                # Get corners in scaled image
                scaled_corners = corners[i]
                pts = scaled_corners[0]

                # Calculate size
                width = np.linalg.norm(pts[0] - pts[1])
                height = np.linalg.norm(pts[1] - pts[2])
                size = (width + height) / 2

                # Minimum size: 20 pixels in scaled image (relaxed for long-range detection)
                if size < 20:
                    continue

                # Aspect ratio must be close to square (0.5 ~ 2.0, relaxed)
                aspect = width / height if height > 0 else 0
                if aspect < 0.5 or aspect > 2.0:
                    continue

                candidates.append((i, size, scaled_corners, marker_id))

            # Process detected markers - use LARGEST (closest) marker for reliability
            # Sort by size (largest first) and use the biggest one
            if candidates:
                candidates.sort(key=lambda x: x[1], reverse=True)

            half_size = self.marker_size / 2.0
            obj_points = np.array([
                [-half_size,  half_size, 0],
                [ half_size,  half_size, 0],
                [ half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)

            all_detections = []  # Store ALL valid detections for averaging
            all_detected_ids = []

            for idx, size, scaled_corners, marker_id in candidates:
                # Get 2D image points from detected corners
                img_points = scaled_corners[0].astype(np.float32)

                # Solve PnP to get rotation and translation
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points,
                    self.scaled_camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if not success:
                    continue

                rvec = rvec.flatten()
                tvec = tvec.flatten()

                # ===== Pose KF DISABLED - all KF methods failed for LED markers =====
                # Baseline (0.178m) is optimal - KF adds lag without reducing vibration
                rot = Rotation.from_rotvec(rvec)
                filtered_quat = rot.as_quat()  # No filtering, raw quat

                all_detected_ids.append(marker_id)

                # Get marker world position from marker_map
                if int(marker_id) in self.marker_map:
                    marker_world = np.array(self.marker_map[int(marker_id)])

                    # ============================================================
                    # Compute image-based quality score for this marker
                    # Components:
                    # 1. Area score: larger marker = closer = more reliable
                    # 2. Center score: closer to image center = less distortion
                    # 3. Aspect ratio: closer to square = frontal view
                    # ============================================================
                    corners_2d = scaled_corners[0]  # shape (4, 2)

                    # 1. Area score (normalized by image area)
                    area = cv2.contourArea(corners_2d.astype(np.float32))
                    img_area = self.image_width * self.image_height
                    area_score = min(area / (img_area * 0.01), 1.0)  # cap at 1% of image

                    # 2. Center distance score
                    center = corners_2d.mean(axis=0)
                    img_center = np.array([self.image_width / 2, self.image_height / 2])
                    center_dist = np.linalg.norm(center - img_center)
                    max_dist = np.linalg.norm(img_center)  # corner to center
                    center_score = 1.0 - (center_dist / max_dist)

                    # 3. Aspect ratio score (squareness)
                    # Compute side lengths
                    side_lengths = [np.linalg.norm(corners_2d[i] - corners_2d[(i+1)%4]) for i in range(4)]
                    min_side = min(side_lengths)
                    max_side = max(side_lengths)
                    aspect_score = min_side / max_side if max_side > 0 else 0

                    # Combined quality score (weighted)
                    quality = 0.5 * area_score + 0.3 * center_score + 0.2 * aspect_score

                    all_detections.append({
                        'marker_id': marker_id,
                        'rvec': rvec,
                        'tvec': tvec,
                        'quat': filtered_quat,
                        'corners': scaled_corners,
                        'marker_world': marker_world,
                        'quality': quality
                    })

                # Draw on debug image for ALL detected markers
                corners_orig = scaled_corners / self.scale_factor
                pts_int = corners_orig[0].astype(int)
                cv2.polylines(debug_img, [pts_int], True, (0, 255, 0), 3)
                center = corners_orig[0].mean(axis=0)
                cx, cy = int(center[0]), int(center[1])
                cv2.circle(debug_img, (cx, cy), 8, (0, 255, 0), -1)
                cv2.putText(debug_img, f'ID:{marker_id}', (cx+10, cy),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Use largest marker as primary (for frame_id), but average ALL poses
            best_detection = all_detections[0] if all_detections else None

            # If we have valid detections, publish pose
            if all_detections:
                # Use LARGEST marker (best_detection) for pose estimation
                # Multi-marker averaging happens in EKF after world coord transform
                current_marker_id = best_detection['marker_id']

                # Use temporal averaging buffer for the SINGLE best marker
                self.pose_buffer.append({
                    'tvec': best_detection['tvec'].copy(),
                    'quat': best_detection['quat'].copy(),
                    'marker_id': current_marker_id
                })

                if len(self.pose_buffer) > self.averaging_samples:
                    self.pose_buffer.pop(0)

                # Calculate temporally averaged pose
                avg_tvec = np.mean([p['tvec'] for p in self.pose_buffer], axis=0)

                # Average quaternions with consistent signs (handle antipodal issue)
                quats = [p['quat'] for p in self.pose_buffer]
                ref_quat = quats[0]
                aligned_quats = []
                for q in quats:
                    if np.dot(q, ref_quat) < 0:
                        aligned_quats.append(-q)  # Flip to same hemisphere
                    else:
                        aligned_quats.append(q)
                avg_quat = np.mean(aligned_quats, axis=0)
                quat_norm = np.linalg.norm(avg_quat)
                if quat_norm > 1e-6:
                    avg_quat = avg_quat / quat_norm
                else:
                    avg_quat = np.array([0.0, 0.0, 0.0, 1.0])  # identity quaternion

                # Use the largest marker's ID for frame_id
                primary_id = best_detection['marker_id']

                # Create and publish pose message (single best marker - for backward compat)
                pose = PoseStamped()
                if header is not None:
                    pose.header = header
                else:
                    pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = f'marker_{primary_id}'
                pose.pose.position.x = float(avg_tvec[0])
                pose.pose.position.y = float(avg_tvec[1])
                pose.pose.position.z = float(avg_tvec[2])
                pose.pose.orientation.x = float(avg_quat[0])
                pose.pose.orientation.y = float(avg_quat[1])
                pose.pose.orientation.z = float(avg_quat[2])
                pose.pose.orientation.w = float(avg_quat[3])
                self.pose_pub.publish(pose)

                # Publish ALL detected markers as PoseArray (for multi-marker averaging in EKF)
                # Each pose has marker_world coords encoded in frame_id
                pose_array = PoseArray()
                if header is not None:
                    pose_array.header = header
                else:
                    pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = 'camera'
                for det in all_detections:
                    p = Pose()
                    p.position.x = float(det['tvec'][0])
                    p.position.y = float(det['tvec'][1])
                    p.position.z = float(det['tvec'][2])
                    p.orientation.x = float(det['quat'][0])
                    p.orientation.y = float(det['quat'][1])
                    p.orientation.z = float(det['quat'][2])
                    p.orientation.w = float(det['quat'][3])
                    # Encode marker_id in unused field (hack: use header's frame_id per marker)
                    pose_array.poses.append(p)
                # Store marker IDs and quality scores in frame_id
                # Format: "id1:q1,id2:q2,..." e.g., "0:0.75,1:0.62"
                pose_array.header.frame_id = ','.join([f"{d['marker_id']}:{d['quality']:.3f}" for d in all_detections])
                self.pose_array_pub.publish(pose_array)

                self.detected += 1
                detected = True
                current_detected_id = all_detected_ids if len(all_detected_ids) > 1 else primary_id

                # Log marker detection
                self.get_logger().info(
                    f'[Detection #{self.detected}] Marker ID: {current_detected_id}, '
                    f'Total markers: {len(all_detected_ids)}, '
                    f'Position: X={avg_tvec[0]:.3f}, Y={avg_tvec[1]:.3f}, Z={avg_tvec[2]:.3f}m'
                )

                # Draw axes for best marker
                axis_length = 0.05
                imgpts, _ = cv2.projectPoints(
                    np.float32([[0,0,0], [axis_length,0,0], [0,axis_length,0], [0,0,axis_length]]),
                    best_detection['rvec'], best_detection['tvec'], self.scaled_camera_matrix, self.dist_coeffs)
                imgpts = imgpts.reshape(-1, 2) / self.scale_factor

                if not np.any(np.isnan(imgpts)):
                    origin = (int(imgpts[0][0]), int(imgpts[0][1]))
                    cv2.line(debug_img, origin, (int(imgpts[1][0]), int(imgpts[1][1])), (0, 0, 255), 2)
                    cv2.line(debug_img, origin, (int(imgpts[2][0]), int(imgpts[2][1])), (0, 255, 0), 2)
                    cv2.line(debug_img, origin, (int(imgpts[3][0]), int(imgpts[3][1])), (255, 0, 0), 2)

                # Show pose info
                cv2.putText(debug_img, f'X:{avg_tvec[0]:.2f} Y:{avg_tvec[1]:.2f} Z:{avg_tvec[2]:.2f}m',
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                # Get Euler angles from averaged quaternion (with safety check)
                try:
                    avg_rot = Rotation.from_quat(avg_quat)
                    euler = avg_rot.as_euler('xyz', degrees=True)
                    cv2.putText(debug_img, f'Roll:{euler[0]:.1f} Pitch:{euler[1]:.1f} Yaw:{euler[2]:.1f}',
                               (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                except ValueError:
                    cv2.putText(debug_img, 'Euler: N/A',
                               (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Status display
        if detected:
            status = f'DETECTED ID:{current_detected_id} ({len(all_detected_ids)} markers)'
            color = (0, 255, 0)
        else:
            status = f'No detection'
            color = (0, 0, 255)

        cv2.putText(debug_img, status, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Show total markers being tracked
        cv2.putText(debug_img, f'Tracking: {len(self.marker_ids)} markers (ID 0-{max(self.marker_ids)})',
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
        if header is not None:
            debug_msg.header = header
        else:
            debug_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector6DOF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
