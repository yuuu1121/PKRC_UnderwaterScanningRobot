#!/usr/bin/env python3
"""
laser_camera_publisher node
- Captures from exploreHD (/dev/video0) and publishes to /image_raw/compressed
- MJPG passthrough: camera already delivers JPEG, forwarded without decode/re-encode
- Exposure adjustable at runtime:
    ros2 param set /laser_camera_publisher exposure <value>
"""

import subprocess
import threading
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import SetParametersResult

# ─────────────────────────────────────────
# 레이저 exploreHD 는 by-path 로 지정한다. 이유: 하방 카메라도 같은
# exploreHD 인데 시리얼까지 동일해서(둘 다 SN00009) by-id 로는 구분이
# 불가능하고, /dev/videoN 번호는 USB 열거 순서에 따라 밀린다(실제 사고).
# by-path 는 물리 포트 기준이라 케이블을 같은 포트에 꽂는 한 안정적이다.
# ponytail: 허브를 다른 포트로 옮기면 깨진다 — 그때는 이 경로만 갱신할 것.
CAMERA_DEVICE    = ("/dev/v4l/by-path/"
                    "platform-3610000.usb-usb-0:2.1.2:1.0-video-index0")
PUBLISH_TOPIC    = "/image_raw/compressed"
PUBLISH_HZ       = 30
DEFAULT_EXPOSURE = 50
# gamma 는 이 카메라가 72~500 을 지원한다(v4l2-ctl --list-ctrls, 기본 100).
# 레이저 선을 어두운 물속에서 볼 때 노출만으로는 부족해 함께 조절한다.
DEFAULT_GAMMA    = 100
# Native MJPG mode (v4l2-ctl --list-formats-ext).
# Supported sizes: 1920x1080, 1280x720, 800x600, 640x480, 640x360.
FRAME_WIDTH      = 1280
FRAME_HEIGHT     = 720

# GStreamer, not the V4L2 backend: OpenCV's V4L2 path caps this camera at
# 20fps at every resolution (measured), while v4l2-ctl and GStreamer both
# reach 30. jpegparse keeps the JPEG bytes intact so the passthrough
# contract below is unchanged -- appsink hands us the same MJPG buffer.
def gst_pipeline(device):
    return (
        f"v4l2src device={device} io-mode=2 ! "
        f"image/jpeg,width={FRAME_WIDTH},height={FRAME_HEIGHT},framerate={PUBLISH_HZ}/1 ! "
        f"jpegparse ! appsink drop=true max-buffers=1 sync=false"
    )
# ─────────────────────────────────────────


class LaserCameraPublisher(Node):
    def __init__(self):
        super().__init__("laser_camera_publisher")

        # device/topic 을 파라미터로 열어 하방 카메라도 같은 노드를 쓴다
        # (downward_camera.launch.py). 기본값은 기존 레이저 카메라 그대로.
        self.declare_parameter("device", CAMERA_DEVICE)
        self.declare_parameter("topic", PUBLISH_TOPIC)
        self.device = self.get_parameter("device").value
        topic = self.get_parameter("topic").value

        self.declare_parameter("exposure", DEFAULT_EXPOSURE)
        self.declare_parameter("gamma", DEFAULT_GAMMA)
        self.add_on_set_parameters_callback(self._on_param_change)

        # Manual exposure before opening the stream. Format/size/fps all come
        # from the pipeline caps, so v4l2-ctl only owns the controls now.
        self._v4l2_ctl("auto_exposure=1")   # 3: auto, 1: manual
        self._apply_exposure(DEFAULT_EXPOSURE)
        self._apply_gamma(DEFAULT_GAMMA)

        self.cap = cv2.VideoCapture(gst_pipeline(self.device), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().fatal(f"Cannot open {self.device} via GStreamer")
            raise RuntimeError("Camera open failed")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(CompressedImage, topic, qos)

        self._running = True
        self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap_thread.start()

        self.get_logger().info(
            f"Publishing {topic} @ {PUBLISH_HZ}Hz  |  "
            f"{FRAME_WIDTH}x{FRAME_HEIGHT}  |  "
            f"exposure={DEFAULT_EXPOSURE}  "
            f"(change: ros2 param set /laser_camera_publisher exposure <val>)"
        )

    def _capture_loop(self):
        """Publish straight off the capture thread. A separate timer polling a
        one-slot buffer drops frames whenever the two periods drift out of
        phase; appsink already caps the queue at one buffer, so the read rate
        is the publish rate."""
        while self._running:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                # frame is a 1D uint8 array of raw MJPG bytes (jpegparse output)
                self._publish(frame)

    def _v4l2_ctl(self, ctrl: str):
        """Set one V4L2 control. cap.set() does not reach the device through
        the GStreamer backend, so controls go via v4l2-ctl instead."""
        r = subprocess.run(
            ["v4l2-ctl", "-d", self.device, f"--set-ctrl={ctrl}"],
            capture_output=True, text=True)
        if r.returncode != 0:
            self.get_logger().warn(f"v4l2-ctl {ctrl} failed: {r.stderr.strip()}")
            return False
        return True

    def _apply_exposure(self, value: int):
        if self._v4l2_ctl(f"exposure_time_absolute={value}"):
            self.get_logger().info(f"Exposure set to {value}")

    def _apply_gamma(self, value: int):
        if self._v4l2_ctl(f"gamma={value}"):
            self.get_logger().info(f"Gamma set to {value}")

    def _on_param_change(self, params):
        for p in params:
            if p.name == "exposure" and p.type_ == Parameter.Type.INTEGER:
                self._apply_exposure(p.value)
            elif p.name == "gamma" and p.type_ == Parameter.Type.INTEGER:
                self._apply_gamma(p.value)
        return SetParametersResult(successful=True)

    def _publish(self, buf):
        msg = CompressedImage()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format          = "jpeg"
        msg.data            = buf.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        self._running = False
        self._cap_thread.join(timeout=1.0)
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaserCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
