#!/usr/bin/env python3
"""
ArUco 6DOF Pose Estimator - Active Marker Version
- Subscribes: /bluerov2/up/image_color, /bluerov2/up/camera_info
- Publishes: /aruco/pose_6dof (PoseStamped with full 6DOF pose)
- Uses cv2.aruco.estimatePoseSingleMarkers() for 3D pose estimation
- Supports up to 30 markers (ID 0-29)
- Camera settings optimized for LED marker detection
"""

import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Header
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
        self.declare_parameter('camera_width', 1600)
        self.declare_parameter('camera_height', 1200)
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

        # Circle (LED bloom) candidate detection - replaces full-image upscale.
        # We threshold the original gray image, find blobs that look roughly
        # circular, then upscale only that small ROI for ArUco detection.
        self.declare_parameter('blob_threshold', 180)       # Brightness cutoff for LED bloom
        self.declare_parameter('blob_min_area', 30)         # Minimum blob area (px in original)
        self.declare_parameter('blob_max_area_frac', 0.10)  # Maximum blob area as fraction of image
        self.declare_parameter('blob_min_circularity', 0.4) # 1.0 = perfect circle
        self.declare_parameter('blob_roi_padding', 20)      # Padding around blob bbox (px in original)
        self.declare_parameter('draw_blob_candidates', True)  # Visualize candidates on debug image

        # Per-stage timing instrumentation - logs mean/max ms every N frames
        self.declare_parameter('enable_timing', True)
        self.declare_parameter('timing_window', 30)  # frames between log dumps

        # Raw image republishing - useful for rosbag recording in direct mode.
        # Published as CompressedImage (JPEG) on '<image_topic>/compressed' to
        # keep bandwidth and per-frame cost low.
        self.declare_parameter('publish_raw_image', True)
        self.declare_parameter('frame_id', 'stellarHD_optical')
        self.declare_parameter('jpeg_quality', 80)  # 1-100, 80 is a good default

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

        # Per-stage timing accumulators (reset every timing_window frames)
        self._timing = defaultdict(list)
        self._timing_counts = defaultdict(int)  # aggregated counts per window
        self._timing_frames = 0
        # Stable column order for the dump line
        self._timing_order = [
            'grab',         # cap.read() in timer_callback
            'cvt',          # BGR -> gray
            'blob',         # _find_circle_candidates
            'roi_resize',   # sum of per-ROI cv2.resize
            'roi_pre',      # sum of per-ROI optional CLAHE/clip
            'roi_aruco',    # sum of per-ROI ArUco detectMarkers
            'pnp_draw',     # PnP + quality scoring + per-marker drawing
            'imgmsg',       # cv2_to_imgmsg of debug image
            'publish',      # actual ROS publish calls
            'raw_pub',      # cv2_to_imgmsg + publish of raw image (for rosbag)
            'frame_total',  # full _process_frame
        ]

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

        # Raw image republisher (direct camera mode only - ROS topic mode already
        # has the source). JPEG-compressed on '<image_topic>/compressed' to keep
        # bandwidth small and avoid the cv_bridge BGR conversion bottleneck.
        # Standard image_transport naming, so rqt_image_view /stellarHD/image_raw
        # with compressed transport will pick it up automatically.
        self.raw_image_pub = None
        self.camera_info_pub = None
        self._cached_camera_info = None
        if self.use_direct_camera:
            compressed_topic = self.image_topic.rstrip('/') + '/compressed'
            self.raw_image_pub = self.create_publisher(
                CompressedImage, compressed_topic, 10)
            self.camera_info_pub = self.create_publisher(
                CameraInfo, self.camera_info_topic, 10)
            self.get_logger().info(
                f'Publishing raw camera frames as CompressedImage on '
                f'{compressed_topic}')
            self.get_logger().info(
                f'Publishing CameraInfo on {self.camera_info_topic}')

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

        # Force MJPG: stellarHD only supports >=30fps at 1600x1200 in MJPG.
        # YUYV at 1600x1200 is hardware-capped to 5 fps. FOURCC must be set
        # BEFORE width/height/fps or the driver will not honor it.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # Set resolution and FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        # Verify the driver actually accepted MJPG (warn loudly if not)
        fcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fcc_str = ''.join([chr((fcc >> 8 * i) & 0xFF) for i in range(4)])
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Pixel format negotiated: {fcc_str}, driver fps={actual_fps}')
        if fcc_str != 'MJPG':
            self.get_logger().warn(
                f'Camera fell back to {fcc_str} — frame rate will be capped '
                f'(YUYV is 5 fps at 1600x1200 on this device)')

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

        # Stellar HD camera intrinsics (MATLAB Camera Calibrator, 69 images, underwater)
        # Overall Mean Error: 0.38 pixels
        self.camera_matrix = np.array([
            [1144.81393846296, 0, 804.961472601195],
            [0, 1146.71365310824, 636.466220142760],
            [0, 0, 1]
        ], dtype=np.float64)
        self.dist_coeffs = np.array([-0.316025746617216, 0.132104985545318, 0, 0, 0])

        # Detection now happens per-ROI (see _find_circle_candidates), so the
        # whole-image upscale is gone. All 2D image points stay in original
        # (camera_width x camera_height) coordinates and we use the unscaled
        # camera_matrix everywhere downstream.
        self.image_width = self.camera_width
        self.image_height = self.camera_height

        self.get_logger().info(f'Camera opened: {self.camera_width}x{self.camera_height}@{self.camera_fps}fps')
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        self.get_logger().info(f'Camera matrix: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}')

        # Pre-build a CameraInfo message we can republish without re-allocating
        self._cached_camera_info = self._build_camera_info()

    def timer_callback(self):
        """Timer callback for direct camera mode"""
        if self.cap is None or not self.cap.isOpened():
            return

        t0 = time.perf_counter()
        ret, frame = self.cap.read()
        self._t_record('grab', (time.perf_counter() - t0) * 1000.0)
        if not ret:
            return

        # Stamp the capture time once and reuse it for every message published
        # for this frame (raw image, camera_info, debug image, poses).
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('frame_id').value
        self._process_frame(frame, header)

    def _t_record(self, name, dt_ms):
        """Record one timing sample (ms) under a stage name."""
        if not self.get_parameter('enable_timing').value:
            return
        self._timing[name].append(dt_ms)

    def _t_count(self, name, value):
        """Aggregate a per-frame count over the timing window."""
        if not self.get_parameter('enable_timing').value:
            return
        self._timing_counts[name] += value

    def _t_dump_if_due(self):
        """Log mean/max per stage every timing_window frames."""
        if not self.get_parameter('enable_timing').value:
            return
        self._timing_frames += 1
        window = int(self.get_parameter('timing_window').value)
        if self._timing_frames < max(1, window):
            return
        # Build a single-line summary in stable column order
        parts = []
        for name in self._timing_order:
            samples = self._timing.get(name)
            if not samples:
                continue
            mean = sum(samples) / len(samples)
            mx = max(samples)
            parts.append(f'{name}={mean:5.1f}/{mx:5.1f}')
        # Throughput: frame_total mean → effective fps
        ft = self._timing.get('frame_total')
        eff_fps = (1000.0 / (sum(ft) / len(ft))) if ft else 0.0
        # Per-window aggregated counts
        n_blobs = self._timing_counts.get('blobs', 0)
        n_rois = self._timing_counts.get('rois', 0)
        n_markers = self._timing_counts.get('markers', 0)
        n_hits = self._timing_counts.get('hits', 0)
        self.get_logger().info(
            f'[timing ms mean/max over {self._timing_frames} frames]  '
            + '  '.join(parts)
            + f'  | eff_fps={eff_fps:.1f}'
            + f'  blobs(sum)={n_blobs} rois(sum)={n_rois} markers(sum)={n_markers}'
            + f'  hit_rate={n_hits}/{self._timing_frames}'
        )
        self._timing.clear()
        self._timing_counts.clear()
        self._timing_frames = 0

    def _build_camera_info(self):
        """Build a CameraInfo from current intrinsics for republishing."""
        msg = CameraInfo()
        msg.width = int(self.camera_width)
        msg.height = int(self.camera_height)
        msg.distortion_model = 'plumb_bob'
        # Pad/truncate distortion to 5 coeffs (k1,k2,p1,p2,k3) for plumb_bob
        d = list(self.dist_coeffs.flatten())
        d = (d + [0.0] * 5)[:5]
        msg.d = [float(x) for x in d]
        msg.k = [float(x) for x in self.camera_matrix.flatten()]
        # Identity rectification (mono camera)
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        # P = [K | 0]
        K = self.camera_matrix
        msg.p = [float(K[0, 0]), float(K[0, 1]), float(K[0, 2]), 0.0,
                 float(K[1, 0]), float(K[1, 1]), float(K[1, 2]), 0.0,
                 float(K[2, 0]), float(K[2, 1]), float(K[2, 2]), 0.0]
        return msg

    def _find_circle_candidates(self, gray):
        """Find bright circular blobs (LED bloom around each marker).

        Runs on the original 1600x1200 gray image, NOT an upscaled copy. This
        is dramatically cheaper than upscaling the whole frame and lets us
        focus expensive ArUco detection on small ROI patches.

        Returns: list of (x, y, w, h, circularity) bounding boxes in original
        image coordinates, sorted by circularity descending.
        """
        thr = int(self.get_parameter('blob_threshold').value)
        min_area = float(self.get_parameter('blob_min_area').value)
        max_area_frac = float(self.get_parameter('blob_max_area_frac').value)
        min_circ = float(self.get_parameter('blob_min_circularity').value)

        # Threshold to find bright (LED-illuminated) regions
        _, thresh = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY)

        # Close small gaps so the LED bloom forms one connected blob
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        img_h, img_w = gray.shape[:2]
        max_area = img_w * img_h * max_area_frac

        candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area or area > max_area:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter < 1.0:
                continue
            # 4*pi*A / P^2: 1.0 = circle, 0.785 = square, < 0.5 = elongated
            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if circularity < min_circ:
                continue
            x, y, w, h = cv2.boundingRect(c)
            candidates.append((int(x), int(y), int(w), int(h), float(circularity)))

        # Most circle-like first
        candidates.sort(key=lambda t: t[4], reverse=True)
        return candidates

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

            # Detection now happens per-ROI in original-image coordinates,
            # so we keep camera_matrix and image dimensions at native scale.
            self.image_width = msg.width
            self.image_height = msg.height

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
        t_frame = time.perf_counter()

        # Use grayscale (combines all BGR channels) for marker detection
        t0 = time.perf_counter()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        debug_img = img.copy()
        self._t_record('cvt', (time.perf_counter() - t0) * 1000.0)

        # ===== Step 1: find LED bloom blobs on the ORIGINAL gray image =====
        # This is much cheaper than upscaling the whole 1600x1200 frame to
        # 4800x3600 and lets us spend the upscale budget only on real ROIs.
        t0 = time.perf_counter()
        circle_candidates = self._find_circle_candidates(gray)
        self._t_record('blob', (time.perf_counter() - t0) * 1000.0)

        # Read preprocessing parameters once per frame (runtime-tunable)
        use_clahe = self.get_parameter('use_clahe').value
        clahe_clip_limit = self.get_parameter('clahe_clip_limit').value
        clip_bright = self.get_parameter('clip_bright_pixels').value
        clip_threshold = self.get_parameter('clip_threshold').value
        roi_padding = int(self.get_parameter('blob_roi_padding').value)
        draw_blobs = bool(self.get_parameter('draw_blob_candidates').value)

        img_h, img_w = gray.shape[:2]

        # ===== Visualize blob candidates so the user can verify detection =====
        if draw_blobs:
            for (bx, by, bw, bh, bcirc) in circle_candidates:
                # Cyan box = candidate ROI sent to ArUco detector
                cv2.rectangle(debug_img, (bx, by), (bx + bw, by + bh),
                              (255, 255, 0), 1)
                cv2.putText(debug_img, f'c={bcirc:.2f}', (bx, max(0, by - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            cv2.putText(debug_img,
                        f'Blob candidates: {len(circle_candidates)}',
                        (10, img_h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # ===== Step 2: per-ROI ArUco detection on locally upscaled patches =====
        # All detected corners are remapped back to original-image coordinates
        # so the rest of the pipeline (PnP, drawing, quality score) works in a
        # single, unscaled coordinate system.
        all_corners = []   # list of arrays shaped (1, 4, 2), in ORIGINAL coords
        all_marker_ids = []  # list of int marker IDs

        # Sum per-ROI sub-stage times across all candidates this frame
        roi_resize_ms = 0.0
        roi_pre_ms = 0.0
        roi_aruco_ms = 0.0
        n_rois_processed = 0

        for (bx, by, bw, bh, _bcirc) in circle_candidates:
            # Padded ROI, clamped to image
            x0 = max(0, bx - roi_padding)
            y0 = max(0, by - roi_padding)
            x1 = min(img_w, bx + bw + roi_padding)
            y1 = min(img_h, by + bh + roi_padding)
            if x1 - x0 < 8 or y1 - y0 < 8:
                continue

            patch = gray[y0:y1, x0:x1]

            # Local upscale - cheap because the patch is small
            t0 = time.perf_counter()
            patch_up = cv2.resize(patch, None,
                                  fx=self.scale_factor, fy=self.scale_factor,
                                  interpolation=cv2.INTER_CUBIC)
            roi_resize_ms += (time.perf_counter() - t0) * 1000.0

            # Apply same optional preprocessing as before, but only on the patch
            t0 = time.perf_counter()
            if use_clahe:
                clahe = cv2.createCLAHE(clipLimit=clahe_clip_limit, tileGridSize=(8, 8))
                patch_up = clahe.apply(patch_up)
            if clip_bright:
                patch_up = np.clip(patch_up, 0, clip_threshold)
                patch_up = cv2.normalize(patch_up, None, 0, 255,
                                         cv2.NORM_MINMAX).astype(np.uint8)
            roi_pre_ms += (time.perf_counter() - t0) * 1000.0

            t0 = time.perf_counter()
            if self.use_new_api:
                p_corners, p_ids, _ = self.detector.detectMarkers(patch_up)
            else:
                p_corners, p_ids, _ = aruco.detectMarkers(
                    patch_up, self.aruco_dict, parameters=self.params)
            roi_aruco_ms += (time.perf_counter() - t0) * 1000.0
            n_rois_processed += 1

            if p_ids is None:
                continue

            # Remap each marker's corners from patch_up space → original coords
            roi_origin = np.array([x0, y0], dtype=np.float32)
            for i in range(len(p_ids)):
                corners_local = p_corners[i][0] / self.scale_factor  # → patch coords
                corners_global = corners_local + roi_origin           # → original coords
                all_corners.append(np.array([corners_global], dtype=np.float32))
                all_marker_ids.append(int(p_ids[i][0]))

        self._t_record('roi_resize', roi_resize_ms)
        self._t_record('roi_pre', roi_pre_ms)
        self._t_record('roi_aruco', roi_aruco_ms)

        # Build (corners, ids) in the same shape the rest of the function expects
        if all_marker_ids:
            ids = np.array(all_marker_ids, dtype=np.int32).reshape(-1, 1)
            corners = all_corners
        else:
            ids = None
            corners = []

        detected = False
        current_detected_id = None
        all_detected_ids = []  # Initialize for blink tolerance logic
        t_pnp = time.perf_counter()
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

                # Corners are in ORIGINAL image coordinates (1600x1200)
                marker_corners = corners[i]
                pts = marker_corners[0]

                # Calculate size in original-image pixels
                width = np.linalg.norm(pts[0] - pts[1])
                height = np.linalg.norm(pts[1] - pts[2])
                size = (width + height) / 2

                # Minimum size: ~7 px in original (was 20 px in 3x-upscaled space)
                if size < 7:
                    continue

                # Aspect ratio must be close to square (0.5 ~ 2.0, relaxed)
                aspect = width / height if height > 0 else 0
                if aspect < 0.5 or aspect > 2.0:
                    continue

                candidates.append((i, size, marker_corners, marker_id))

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

            for idx, size, marker_corners, marker_id in candidates:
                # 2D image points are already in original-image coordinates
                img_points = marker_corners[0].astype(np.float32)

                # Solve PnP using the unscaled camera matrix
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points,
                    self.camera_matrix, self.dist_coeffs,
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
                    corners_2d = marker_corners[0]  # shape (4, 2), original coords

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
                        'corners': marker_corners,
                        'marker_world': marker_world,
                        'quality': quality
                    })

                # Draw on debug image for ALL detected markers
                # marker_corners is already in original image coordinates.
                pts_int = marker_corners[0].astype(int)
                cv2.polylines(debug_img, [pts_int], True, (0, 255, 0), 3)
                center = marker_corners[0].mean(axis=0)
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

                # Draw axes for best marker (works in original image coords)
                axis_length = 0.05
                imgpts, _ = cv2.projectPoints(
                    np.float32([[0,0,0], [axis_length,0,0], [0,axis_length,0], [0,0,axis_length]]),
                    best_detection['rvec'], best_detection['tvec'],
                    self.camera_matrix, self.dist_coeffs)
                imgpts = imgpts.reshape(-1, 2)

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
        self._t_record('pnp_draw', (time.perf_counter() - t_pnp) * 1000.0)

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

        t0 = time.perf_counter()
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
        if header is not None:
            debug_msg.header = header
        else:
            debug_msg.header.stamp = self.get_clock().now().to_msg()
        self._t_record('imgmsg', (time.perf_counter() - t0) * 1000.0)

        t0 = time.perf_counter()
        self.debug_pub.publish(debug_msg)
        self._t_record('publish', (time.perf_counter() - t0) * 1000.0)

        # ===== Republish raw frame as JPEG CompressedImage (for rosbag) =====
        if (self.raw_image_pub is not None
                and self.get_parameter('publish_raw_image').value):
            t0 = time.perf_counter()
            quality = int(self.get_parameter('jpeg_quality').value)
            ok, jpg_buf = cv2.imencode(
                '.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            if ok:
                comp = CompressedImage()
                if header is not None:
                    comp.header = header
                else:
                    comp.header.stamp = self.get_clock().now().to_msg()
                    comp.header.frame_id = self.get_parameter('frame_id').value
                comp.format = 'jpeg'
                comp.data = jpg_buf.tobytes()
                self.raw_image_pub.publish(comp)
                # Republish CameraInfo with the SAME stamp so downstream tools
                # can synchronize raw frames + intrinsics.
                if self.camera_info_pub is not None and self._cached_camera_info is not None:
                    ci = self._cached_camera_info
                    ci.header = comp.header
                    self.camera_info_pub.publish(ci)
            self._t_record('raw_pub', (time.perf_counter() - t0) * 1000.0)

        # Whole-frame total and periodic dump
        self._t_record('frame_total', (time.perf_counter() - t_frame) * 1000.0)
        self._t_count('blobs', len(circle_candidates))
        self._t_count('rois', n_rois_processed)
        self._t_count('markers', len(all_marker_ids))
        if all_marker_ids:
            self._t_count('hits', 1)
        self._t_dump_if_due()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector6DOF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
