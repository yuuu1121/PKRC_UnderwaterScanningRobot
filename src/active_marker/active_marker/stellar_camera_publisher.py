#!/usr/bin/env python3
"""
Stellar HD camera publisher (no detection).

Opens /dev/video4 with the SAME settings as aruco_detector_6dof's
_setup_direct_camera() and publishes:
  - <image_topic>/compressed  (sensor_msgs/CompressedImage, JPEG)
  - <camera_info_topic>       (sensor_msgs/CameraInfo)

Use this for rosbag recording / debugging without running the ArUco
pipeline. Camera intrinsics, exposure, MJPG fourcc, frame_id all match
the detector node so bagged data is interchangeable.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Header
import cv2
import numpy as np


class StellarCameraPublisher(Node):
    def __init__(self):
        super().__init__('stellar_camera_publisher')

        # ===== Parameters (match aruco_detector_6dof.py defaults) =====
        self.declare_parameter('camera_device', 4)        # /dev/video4
        self.declare_parameter('camera_width', 1600)
        self.declare_parameter('camera_height', 1200)
        self.declare_parameter('camera_fps', 30)
        # Manual exposure tuned for LED markers
        self.declare_parameter('auto_exposure', 1)        # 1=manual, 3=auto
        self.declare_parameter('exposure_time', 1)
        self.declare_parameter('brightness', -64)
        self.declare_parameter('contrast', 64)
        self.declare_parameter('gamma', 90)
        self.declare_parameter('gain', 0)
        # Topics
        self.declare_parameter('image_topic', '/stellarHD/image_raw')
        self.declare_parameter('camera_info_topic', '/stellarHD/camera_info')
        self.declare_parameter('frame_id', 'stellarHD_optical')
        # JPEG
        self.declare_parameter('jpeg_quality', 60)

        self.camera_device = int(self.get_parameter('camera_device').value)
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.camera_fps = int(self.get_parameter('camera_fps').value)
        self.auto_exposure = int(self.get_parameter('auto_exposure').value)
        self.exposure_time = int(self.get_parameter('exposure_time').value)
        self.brightness = int(self.get_parameter('brightness').value)
        self.contrast = int(self.get_parameter('contrast').value)
        self.gamma = int(self.get_parameter('gamma').value)
        self.gain = int(self.get_parameter('gain').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # ===== Stellar HD intrinsics (MATLAB Camera Calibrator, 69 images) =====
        # Same values as aruco_detector_6dof.py to keep bagged data compatible.
        self.camera_matrix = np.array([
            [1144.81393846296, 0, 804.961472601195],
            [0, 1146.71365310824, 636.466220142760],
            [0, 0, 1]
        ], dtype=np.float64)
        self.dist_coeffs = np.array(
            [-0.316025746617216, 0.132104985545318, 0, 0, 0],
            dtype=np.float64,
        )

        # ===== Publishers =====
        compressed_topic = self.image_topic.rstrip('/') + '/compressed'
        self.image_pub = self.create_publisher(CompressedImage, compressed_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self._cached_camera_info = self._build_camera_info()

        # ===== Open camera =====
        self._open_camera()

        # ===== Timer =====
        self.timer = self.create_timer(1.0 / self.camera_fps, self.timer_callback)

        # Stats
        self._frame_count = 0
        self._last_log_time = time.perf_counter()
        self._published = 0

        self.get_logger().info(
            f'StellarCameraPublisher started: {self.camera_width}x{self.camera_height}@{self.camera_fps}fps')
        self.get_logger().info(f'  Compressed image topic: {compressed_topic}')
        self.get_logger().info(f'  Camera info topic:      {self.camera_info_topic}')
        self.get_logger().info(f'  Frame id:               {self.frame_id}')

    def _open_camera(self):
        self.get_logger().info(f'Opening camera device: /dev/video{self.camera_device}')
        self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open /dev/video{self.camera_device}')
            return

        # Force MJPG: stellarHD only supports >=30fps at 1600x1200 in MJPG.
        # FOURCC must be set BEFORE width/height/fps or the driver will not honor it.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        # Verify the driver actually accepted MJPG
        fcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fcc_str = ''.join([chr((fcc >> 8 * i) & 0xFF) for i in range(4)])
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'Pixel format negotiated: {fcc_str}, driver fps={actual_fps}')
        if fcc_str != 'MJPG':
            self.get_logger().warn(
                f'Camera fell back to {fcc_str} - frame rate may be capped '
                f'(YUYV is 5 fps at 1600x1200 on this device)')

        # Manual exposure for LED marker isolation
        ok = self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self.auto_exposure)
        self.get_logger().info(f'Manual exposure mode (auto_exposure={self.auto_exposure}): {"OK" if ok else "FAIL"}')
        ok = self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_time)
        self.get_logger().info(f'Exposure time ({self.exposure_time}): {"OK" if ok else "FAIL"}')
        ok = self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
        self.get_logger().info(f'Brightness ({self.brightness}): {"OK" if ok else "FAIL"}')
        ok = self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
        self.get_logger().info(f'Contrast ({self.contrast}): {"OK" if ok else "FAIL"}')
        ok = self.cap.set(cv2.CAP_PROP_GAMMA, self.gamma)
        self.get_logger().info(f'Gamma ({self.gamma}): {"OK" if ok else "FAIL"}')
        ok = self.cap.set(cv2.CAP_PROP_GAIN, self.gain)
        self.get_logger().info(f'Gain ({self.gain}): {"OK" if ok else "FAIL"}')

    def _build_camera_info(self):
        msg = CameraInfo()
        msg.width = int(self.camera_width)
        msg.height = int(self.camera_height)
        msg.distortion_model = 'plumb_bob'
        d = list(self.dist_coeffs.flatten())
        d = (d + [0.0] * 5)[:5]
        msg.d = [float(x) for x in d]
        msg.k = [float(x) for x in self.camera_matrix.flatten()]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        K = self.camera_matrix
        msg.p = [float(K[0, 0]), float(K[0, 1]), float(K[0, 2]), 0.0,
                 float(K[1, 0]), float(K[1, 1]), float(K[1, 2]), 0.0,
                 float(K[2, 0]), float(K[2, 1]), float(K[2, 2]), 0.0]
        return msg

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        # One stamp shared by image + camera_info for downstream sync
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        quality = int(self.get_parameter('jpeg_quality').value)
        ok, jpg_buf = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return

        comp = CompressedImage()
        comp.header = header
        comp.format = 'jpeg'
        comp.data = jpg_buf.tobytes()
        self.image_pub.publish(comp)

        ci = self._cached_camera_info
        ci.header = header
        self.info_pub.publish(ci)

        self._published += 1
        self._frame_count += 1

        # Lightweight stats every ~3 seconds
        now = time.perf_counter()
        if now - self._last_log_time >= 3.0:
            elapsed = now - self._last_log_time
            fps = self._frame_count / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(
                f'publishing at {fps:.1f} fps  (jpeg_quality={quality}, '
                f'last frame={len(comp.data) // 1024} KB, total={self._published})')
            self._last_log_time = now
            self._frame_count = 0

    def destroy_node(self):
        try:
            if self.cap is not None and self.cap.isOpened():
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StellarCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
