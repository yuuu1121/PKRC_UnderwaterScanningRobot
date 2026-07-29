#!/usr/bin/env python3
"""
exploreHD Camera Intrinsic Calibration (Fisheye) - ROS2 Topic Version
- ROS2 토픽에서 압축된 이미지를 받아서 캘리브레이션
- 체커보드를 카메라에 보여주면서 SPACE로 캡처
- 충분한 이미지 수집 후 'c'를 눌러 캘리브레이션 수행
- 결과는 camera_intrinsics.yaml 파일로 저장
"""

import cv2
import numpy as np
import yaml
import os
import shutil
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

# ─────────────────────────────────────────
# 설정값 (필요 시 수정)
# ─────────────────────────────────────────
IMAGE_TOPIC    = "/image_raw/compressed"
CELL_SIZE_MM   = 70.00      # 체커보드 한 칸 크기 (mm)
BOARD_W        = 4          # 내부 코너 수 (가로) — 칸 5개 → 코너 4개
BOARD_H        = 5          # 내부 코너 수 (세로) — 칸 6개 → 코너 5개
MIN_IMAGES     = 15         # 캘리브레이션에 필요한 최소 이미지 수
SAVE_DIR       = "calib_images"   # 캡처 이미지 저장 디렉토리
OUTPUT_FILE    = "camera_intrinsics.yaml"
# ─────────────────────────────────────────

os.makedirs(SAVE_DIR, exist_ok=True)

# 체커보드 3D 코너 좌표 — fisheye는 (N,1,3) shape 필요
objp = np.zeros((BOARD_H * BOARD_W, 1, 3), np.float64)
objp[:, 0, :2] = np.mgrid[0:BOARD_W, 0:BOARD_H].T.reshape(-1, 2)
objp *= CELL_SIZE_MM


class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')

        self.objpoints = []   # 3D 포인트 목록
        self.imgpoints = []   # 2D 코너 목록
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.captured = 0
        self.last_capture_time = 0
        self.width = 1920
        self.height = 1080
        self.current_frame = None
        self.calibration_done = False

        # 이미지 구독 (BEST_EFFORT QoS)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            IMAGE_TOPIC,
            self.image_callback,
            qos
        )

        self.get_logger().info(f"구독 토픽: {IMAGE_TOPIC}")
        self.get_logger().info(f"체커보드 내부 코너: {BOARD_W}x{BOARD_H}, 셀 크기: {CELL_SIZE_MM}mm")
        self.get_logger().info("")
        self.get_logger().info("조작법:")
        self.get_logger().info("  SPACE  : 체커보드가 감지된 현재 프레임 캡처")
        self.get_logger().info("  c      : 캘리브레이션 수행 (이미지 수집 완료 후)")
        self.get_logger().info("  d      : 마지막으로 캡처한 이미지 삭제")
        self.get_logger().info("  q / ESC: 종료")
        self.get_logger().info("")

    def image_callback(self, msg):
        # 압축된 이미지 디코드
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn("프레임 디코드 실패")
            return

        self.current_frame = frame
        self.width = frame.shape[1]
        self.height = frame.shape[0]

    def process_frame(self):
        if self.current_frame is None:
            return

        frame = self.current_frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 1080p 원본 탐색은 Jetson에서 프레임당 수십 초 → 절반 해상도 + FAST_CHECK로 탐색.
        # 코너 좌표는 2배로 되돌린 뒤 원본 해상도에서 subpixel 정밀화하므로 정확도 손실 없음.
        small = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        found, corners = cv2.findChessboardCorners(
            small, (BOARD_W, BOARD_H),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE +
            cv2.CALIB_CB_FAST_CHECK
        )
        if found:
            corners = corners * 2.0

        display = frame.copy()

        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            cv2.drawChessboardCorners(display, (BOARD_W, BOARD_H), corners2, found)
            status_color = (0, 255, 0)
            status_text  = f"체커보드 감지됨 | 캡처: {self.captured}/{MIN_IMAGES}"
            self.last_corners = corners2
            self.last_found = True
        else:
            status_color = (0, 100, 255)
            status_text  = f"체커보드 미감지 | 캡처: {self.captured}/{MIN_IMAGES}"
            self.last_found = False

        cv2.rectangle(display, (0, 0), (self.width, 40), (0, 0, 0), -1)
        cv2.putText(display, status_text, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)

        if self.captured >= MIN_IMAGES:
            cv2.putText(display, "c 키로 캘리브레이션 수행 가능",
                        (10, self.height - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        cv2.imshow("exploreHD Fisheye Calibration", display)

    def capture_frame(self):
        if not self.last_found:
            self.get_logger().info("체커보드가 감지되지 않아 캡처하지 않았습니다.")
            return

        now = time.time()
        if now - self.last_capture_time < 0.5:
            return
        self.last_capture_time = now

        self.objpoints.append(objp)
        # fisheye는 (N,1,2) shape 필요
        self.imgpoints.append(self.last_corners.reshape(-1, 1, 2))
        self.captured += 1

        fname = os.path.join(SAVE_DIR, f"calib_{self.captured:03d}.png")
        cv2.imwrite(fname, self.current_frame)
        self.get_logger().info(f"[{self.captured}] 캡처 저장: {fname}")

    def delete_last_capture(self):
        if self.captured > 0:
            self.objpoints.pop()
            self.imgpoints.pop()
            fname = os.path.join(SAVE_DIR, f"calib_{self.captured:03d}.png")
            if os.path.exists(fname):
                os.remove(fname)
            self.captured -= 1
            self.get_logger().info(f"마지막 캡처 삭제. 남은 이미지: {self.captured}")

    def calibrate(self):
        if self.captured < MIN_IMAGES:
            self.get_logger().info(f"이미지가 부족합니다. 최소 {MIN_IMAGES}장 필요 (현재 {self.captured}장)")
            return

        self.get_logger().info(f"\n{self.captured}장 이미지로 어안 캘리브레이션 수행 중...")

        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(self.captured)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(self.captured)]

        flags = (
            cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
            cv2.fisheye.CALIB_CHECK_COND |
            cv2.fisheye.CALIB_FIX_SKEW
        )

        try:
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                self.objpoints, self.imgpoints, (self.width, self.height),
                K, D, rvecs, tvecs,
                flags, self.criteria
            )
        except cv2.error as e:
            self.get_logger().error(f"캘리브레이션 실패: {e}")
            self.get_logger().error("이미지를 더 다양한 각도/위치에서 캡처해보세요.")
            return

        d = D.ravel()
        k1, k2, k3, k4 = float(d[0]), float(d[1]), float(d[2]), float(d[3])

        print("\n========== 캘리브레이션 결과 (Fisheye) ==========")
        print(f"RMS 재투영 오차: {rms:.4f} px")
        print(f"\n카메라 행렬 K:\n{K}")
        print(f"\n왜곡 계수 (k1,k2,k3,k4):")
        print(f"  k1={k1:.6f}, k2={k2:.6f}, k3={k3:.6f}, k4={k4:.6f}")
        print("=================================================\n")

        data = {
            "model": "fisheye",
            "image_width":  self.width,
            "image_height": self.height,
            "rms_error":    float(rms),
            "camera_matrix": {
                "rows": 3, "cols": 3,
                "data": K.flatten().tolist()
            },
            "distortion_coefficients": {
                "model": "fisheye (k1,k2,k3,k4)",
                "rows": 1, "cols": 4,
                "data": [k1, k2, k3, k4]
            },
            "fx": float(K[0, 0]),
            "fy": float(K[1, 1]),
            "cx": float(K[0, 2]),
            "cy": float(K[1, 2]),
            "k1": k1, "k2": k2, "k3": k3, "k4": k4,
        }

        with open(OUTPUT_FILE, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f"결과 저장 완료: {OUTPUT_FILE}")

        pkg_config = os.path.expanduser(
            "~/ros2_ws/src/laser_ros/config/camera_intrinsics.yaml")
        if os.path.isdir(os.path.dirname(pkg_config)):
            shutil.copy(OUTPUT_FILE, pkg_config)
            self.get_logger().info(f"패키지 config 복사 완료: {pkg_config}")

        # 보정된 뷰 미리보기
        self.get_logger().info("보정된 이미지를 미리보기합니다. 아무 키나 누르면 다음, q/ESC로 종료.")
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, (self.width, self.height), np.eye(3), balance=0.0
        )
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), new_K, (self.width, self.height), cv2.CV_16SC2
        )
        for fname in sorted(os.listdir(SAVE_DIR))[:5]:
            img = cv2.imread(os.path.join(SAVE_DIR, fname))
            if img is None:
                continue
            undist = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
            compare = np.hstack([
                cv2.resize(img,    (self.width // 2, self.height // 2)),
                cv2.resize(undist, (self.width // 2, self.height // 2))
            ])
            cv2.putText(compare, "Original", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(compare, "Undistorted", (self.width // 2 + 10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Undistort Preview", compare)
            if cv2.waitKey(0) & 0xFF in (27, ord('q')):
                break

        cv2.destroyWindow("Undistort Preview")
        self.calibration_done = True

    def run(self):
        # 창을 미리 만들고 최상위로 — X11 포워딩에서 다른 창(VS Code 등) 뒤에 가려지는 것 방지
        win = "exploreHD Fisheye Calibration"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 960, 540)
        try:
            cv2.setWindowProperty(win, cv2.WND_PROP_TOPMOST, 1)
        except cv2.error:
            pass

        while rclpy.ok() and not self.calibration_done:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.process_frame()

            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord('q')):
                self.get_logger().info("종료합니다.")
                break
            elif key == ord(' '):
                self.capture_frame()
            elif key == ord('d'):
                self.delete_last_capture()
            elif key == ord('c'):
                self.calibrate()

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    try:
        calibrator = CameraCalibrator()
        calibrator.run()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
