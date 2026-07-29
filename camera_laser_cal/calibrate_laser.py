#!/usr/bin/env python3
"""
레이저 평면 캘리브레이션 (ROS2 토픽 입력 버전)
- 녹색 가로 / 빨강 세로 라인 모두 지원

전제:
    camera.py 가 /camera/image_raw/compressed 로 publish 중이어야 함.

사용법:
    python3 calibrate_laser.py --color green
    python3 calibrate_laser.py --color red

- 토픽으로 받은 압축 이미지를 디코드하여 사용
- 카메라 설정(노출/WB 등)은 camera.py 측에서만 관리
- 체커보드를 여러 위치/각도에 놓고 SPACE로 캡처
- SVD로 레이저 평면 방정식 도출 → {color}_laser_plane.yaml 저장
"""

import argparse

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

# ─── 인자 파싱 ───────────────────────────
parser = argparse.ArgumentParser(description="레이저 평면 캘리브레이션 (ROS2 토픽)")
parser.add_argument("--color", choices=["green", "red"], default="green",
                    help="green: 가로 라인(520nm), red: 세로 라인")
parser.add_argument("--angle", type=float, default=None,
                    help="레이저 라인 각도(도). 미지정시 green=0, red=90. "
                         "대각선이면 45 등으로 설정.")
parser.add_argument("--cell-near", type=float, default=40.0,
                    help="근거리 체커보드 셀 크기(mm). 기본 40.")
parser.add_argument("--cell-far", type=float, default=70.0,
                    help="원거리 체커보드 셀 크기(mm). 기본 70. "
                         "세션 중 'b' 키로 근/원 셀 크기 전환.")
args = parser.parse_args()
COLOR = args.color
CELL_NEAR = float(args.cell_near)
CELL_FAR  = float(args.cell_far)
if args.angle is not None:
    LINE_ANGLE = float(args.angle)
else:
    LINE_ANGLE = 0.0 if COLOR == "green" else 90.0

# ─── 설정 ────────────────────────────────
IMAGE_TOPIC      = "/image_raw/compressed"
INTRINSICS_FILE  = "camera_intrinsics.yaml"
OUTPUT_FILE      = f"{COLOR}_laser_plane.yaml"
BOARD_W          = 5
BOARD_H          = 4
MIN_CAPTURES     = 10

# 레이저 탐지 ROI — 체커 코너 바운딩박스에 더하는 여유(margin, px). t 로 가로/세로 스왑.
MARGIN_X_PX      = 120      # 가로 여유 (px)
MARGIN_Y_PX      = 240      # 세로 여유 (px)
# 색판별+포화보정+BG차분 후의 신호 스케일 기준.
# 라인은 ~150~200, 노이즈/블롭은 BG차분으로 ~5~30 으로 분리됨.
# 녹색은 신호가 약한 경우가 많아 임계값을 좀 낮게.
SIGNAL_THR       = 12 if COLOR == "green" else 30
RANSAC_THR       = 5
RANSAC_ITER      = 500
# ─────────────────────────────────────────

with open(INTRINSICS_FILE) as f:
    cal = yaml.safe_load(f)
K = np.array(cal["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
D = np.array(cal["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1, 1)

def make_objp(cell_mm):
    """체커보드 3D 코너 좌표(mm). 셀 크기로 스케일."""
    objp = np.zeros((BOARD_H * BOARD_W, 3), np.float64)
    objp[:, :2] = np.mgrid[0:BOARD_W, 0:BOARD_H].T.reshape(-1, 2)
    objp *= cell_mm
    return objp

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
rng = np.random.default_rng(0)


def roi_margins(swapped):
    """체커 바운딩박스에 더할 (가로, 세로) 여유 px 반환.
    swapped=True 면 가로/세로 여유를 교환한 좌우 바뀐 상자."""
    return (MARGIN_Y_PX, MARGIN_X_PX) if swapped else (MARGIN_X_PX, MARGIN_Y_PX)


# ─── 헬퍼 ────────────────────────────────

def color_signal(frame):
    """laser_realtime.py 와 동일한 색 판별식.

    - 녹: g - 0.5*(r+b) 로 흰색 거부 + 약한 신호도 통과
    - 빨: r - 0.5*(g+b) + hue 게이트
    - 양 색 모두 포화 보정으로 saturate 된 강한 라인 강제 통과
    - BG차분은 여기서 X. extract_laser_line() 에서 회전 후에 적용한다
      (원본 좌표계에서는 라인 각도가 다양해서 고정 방향 BG가 부적절).
    """
    g = frame[:, :, 1].astype(np.float32)
    r = frame[:, :, 2].astype(np.float32)
    b = frame[:, :, 0].astype(np.float32)

    if COLOR == "green":
        gd = np.clip(g - 0.5 * (r + b), 0, None)
        sat_green = (g >= 245) & (g > r + 30) & (g > b + 30)
        return np.maximum(gd, sat_green.astype(np.float32) * 200.0)

    # 빨강
    rd = np.clip(r - 0.5 * (g + b), 0, None)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_ch = hsv[..., 0]
    red_hue = (h_ch <= 18) | (h_ch >= 162)
    sig = rd * red_hue.astype(np.float32)

    sat_red = (r >= 245) & (r > g + 30) & (r > b + 30)
    sig = np.maximum(sig, sat_red.astype(np.float32) * 200.0)
    return sig


def extract_laser_line(frame, corners, margin_x, margin_y, sig_thr, ransac_thr,
                       angle_deg=0.0):
    """ROI 를 angle_deg 만큼 회전한 좌표계에서 '가로 라인'으로 가정하고
    컬럼별 피크를 잡은 뒤, 역회전하여 원본 이미지 좌표로 돌려준다.

    ROI 는 체커 코너 바운딩박스에 가로 margin_x / 세로 margin_y (px) 여유를 준 직사각형.

    angle_deg = 0  → 원본의 가로 라인 (예: green)
    angle_deg = 90 → 원본의 세로 라인 (예: red)
    angle_deg = 45 → 45도 대각 라인
    """
    H, W = frame.shape[:2]
    pts = corners.reshape(-1, 2)
    x1 = max(0, int(pts[:, 0].min()) - margin_x)
    x2 = min(W, int(pts[:, 0].max()) + margin_x)
    y1 = max(0, int(pts[:, 1].min()) - margin_y)
    y2 = min(H, int(pts[:, 1].max()) + margin_y)

    signal = color_signal(frame)
    roi = signal[y1:y2, x1:x2]
    h_roi, w_roi = roi.shape
    if h_roi < 4 or w_roi < 4:
        return [], (x1, y1, x2, y2)

    cx, cy = w_roi * 0.5, h_roi * 0.5
    M = cv2.getRotationMatrix2D((cx, cy), angle_deg, 1.0)

    cos_a = abs(M[0, 0])
    sin_a = abs(M[0, 1])
    new_w = int(np.ceil(h_roi * sin_a + w_roi * cos_a))
    new_h = int(np.ceil(h_roi * cos_a + w_roi * sin_a))
    M[0, 2] += new_w * 0.5 - cx
    M[1, 2] += new_h * 0.5 - cy

    rot = cv2.warpAffine(
        roi, M, (new_w, new_h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    )

    # 회전 후 라인은 가로 방향이므로 세로 BG 차분으로
    # 큰 블롭/면적 노이즈를 제거하고 얇은 라인만 살린다.
    bg = cv2.GaussianBlur(rot, (1, 61), 0)
    rot = np.clip(rot - bg, 0, None)

    rows_rot = np.arange(new_h, dtype=np.float32)
    peaks = []
    for col in range(new_w):
        col_sig = rot[:, col]
        mask = col_sig > sig_thr
        if mask.sum() < 1:
            continue
        w = col_sig[mask]
        peak_row = float(np.dot(rows_rot[mask], w) / w.sum())
        peaks.append((float(col), peak_row, float(w.max())))

    if len(peaks) < 4:
        return [], (x1, y1, x2, y2)

    xs = np.array([p[0] for p in peaks], dtype=np.float32)
    ys = np.array([p[1] for p in peaks], dtype=np.float32)

    best_mask, best_count = None, 0
    for _ in range(RANSAC_ITER):
        idx = rng.choice(len(xs), 2, replace=False)
        if abs(xs[idx[1]] - xs[idx[0]]) < 1:
            continue
        slope     = (ys[idx[1]] - ys[idx[0]]) / (xs[idx[1]] - xs[idx[0]])
        intercept = ys[idx[0]] - slope * xs[idx[0]]
        dists     = np.abs(ys - (slope * xs + intercept))
        inliers   = dists < ransac_thr
        if inliers.sum() > best_count:
            best_count = inliers.sum()
            best_mask  = inliers

    if best_mask is None or best_mask.sum() < 4:
        return [], (x1, y1, x2, y2)

    inlier_rot = np.column_stack([
        xs[best_mask], ys[best_mask]
    ]).astype(np.float64)
    ones = np.ones((inlier_rot.shape[0], 1), dtype=np.float64)
    pts_h = np.hstack([inlier_rot, ones])

    M_inv = cv2.invertAffineTransform(M)
    pts_roi = (M_inv @ pts_h.T).T
    pts_img = pts_roi + np.array([x1, y1], dtype=np.float64)

    laser_pixels = []
    for (u, v) in pts_img:
        ui, vi = int(round(u)), int(round(v))
        if 0 <= ui < W and 0 <= vi < H:
            laser_pixels.append((ui, vi))

    return laser_pixels, (x1, y1, x2, y2)


def pixel_to_ray_rectified(u, v, inv_new_K):
    """보정(렉티파이) 이미지 픽셀 → 카메라 좌표계 단위 광선 (new_K 기준 핀홀)."""
    d = inv_new_K @ np.array([float(u), float(v), 1.0], dtype=np.float64)
    n = np.linalg.norm(d)
    if n < 1e-12:
        return np.array([0.0, 0.0, 1.0])
    return d / n


def normalized_image_points_rectified(corners_n12, inv_new_K):
    """corners (N,1,2) rectified pixels → solvePnP용 정규화 이미지 좌표 (N,1,2)."""
    flat = corners_n12.reshape(-1, 2).astype(np.float64)
    hom = np.vstack([flat.T, np.ones((1, flat.shape[0]), dtype=np.float64)])
    xyz = inv_new_K @ hom
    xy = (xyz[:2] / np.maximum(xyz[2:3], 1e-12)).T.astype(np.float64)
    return xy.reshape(-1, 1, 2)


def ray_plane_intersect(ray, n, d):
    denom = np.dot(n, ray)
    if abs(denom) < 1e-8:
        return None
    t = d / denom
    return t * ray if t > 0 else None


# ─── ROS 서브스크라이버 ───────────────────

class LaserCalibrator(Node):
    def __init__(self):
        super().__init__('laser_calibrator')
        self.current_frame = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(CompressedImage, IMAGE_TOPIC, self.image_cb, qos)
        self.get_logger().info(f"Subscribing : {IMAGE_TOPIC}")

    def image_cb(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None:
            self.current_frame = frame


# ─── 메인 ────────────────────────────────

def main():
    rclpy.init()
    node = LaserCalibrator()

    print(f"색상      : {COLOR} (라인 각도 {LINE_ANGLE:.1f}도)")
    print(f"토픽      : {IMAGE_TOPIC}")
    print(f"출력 파일 : {OUTPUT_FILE}")

    # 첫 프레임 대기
    print("\n첫 프레임 대기 중... (camera.py 가 publish 중인지 확인)")
    timeout = 10.0
    start = node.get_clock().now()
    while node.current_frame is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = (node.get_clock().now() - start).nanoseconds * 1e-9
        if elapsed > timeout:
            print(f"  X 토픽 수신 실패 (>{timeout:.0f}s). camera.py 가 살아있나요?")
            rclpy.shutdown()
            return

    H, W = node.current_frame.shape[:2]
    print(f"첫 프레임 OK: {W} x {H}\n")

    # Fisheye undistort 맵 사전 계산 (intrinsics 는 fisheye 4계수여야 함)
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (W, H), np.eye(3), balance=0.0)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), new_K, (W, H), cv2.CV_16SC2)
    inv_new_K = np.linalg.inv(new_K)

    window_name = f"Laser Detection ({COLOR})"
    debug_window_name = f"DEBUG: {COLOR} signal"

    cv2.namedWindow(window_name)
    cv2.createTrackbar("SIGNAL_THR",  window_name, SIGNAL_THR,        50, lambda x: None)
    cv2.createTrackbar("RANSAC_THR",  window_name, RANSAC_THR,        20, lambda x: None)
    cv2.createTrackbar("DEBUG_SIGNAL",window_name, 0,                  1, lambda x: None)

    laser_3d_pts = []
    captured = 0
    cap_per_cell = {}   # {셀크기mm: 캡처수} — 근/원 보드 기여도 추적

    # 레이저 ROI 여유 토글 (False: 가로120/세로240, True: 좌우 바뀐 가로240/세로120)
    box_swapped = False

    # 활성 셀 크기: 원거리(CELL_FAR) / 근거리(CELL_NEAR) 를 'b' 로 전환.
    # 보드를 바꿔 끼울 때마다 실제 보드에 맞는 셀 크기를 골라야
    # solvePnP 가 깊이를 정확히 잡아 동일 레이저 평면 위에 점이 쌓인다.
    # 원거리 보드부터 시작 → 'b' 로 근거리 전환.
    cell_is_far = True
    active_cell = CELL_FAR
    objp = make_objp(active_cell)

    print("조작법:")
    print("  SPACE : 체커보드+레이저 동시 감지 시 캡처")
    print("  t     : ROI 상자 가로/세로 전환 (좌우 바뀐 상자)")
    print(f"  b     : 셀 크기 전환 (시작=원거리 {CELL_FAR:.0f}mm <-> 근거리 {CELL_NEAR:.0f}mm)")
    print("  c     : 레이저 평면 계산 (근/원 점 모두 합쳐 한 번에 피팅)")
    print("  q/ESC : 종료\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.current_frame is None:
                continue

            frame = node.current_frame.copy()
            undist = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

            # 체커보드 검출용 그레이.
            # 초록 레이저는 G 채널을 강하게 때려서 보드 흑백 대비를 망가뜨림.
            # → green 모드에선 R/B 평균만 사용 (G 채널 드롭).
            # red 모드는 대칭적으로 G/B 평균 사용.
            b_ch, g_ch, r_ch = cv2.split(undist)
            if COLOR == "green":
                gray_undist = cv2.addWeighted(r_ch, 0.5, b_ch, 0.5, 0)
            else:
                gray_undist = cv2.addWeighted(g_ch, 0.5, b_ch, 0.5, 0)

            sig_thr      = max(1, cv2.getTrackbarPos("SIGNAL_THR",  window_name))
            ransac_thr   = max(1, cv2.getTrackbarPos("RANSAC_THR",  window_name))
            debug_signal = cv2.getTrackbarPos("DEBUG_SIGNAL", window_name)

            # ROI 여유 (체커 바운딩박스에 더할 가로/세로 margin, 토글 시 스왑)
            margin_x, margin_y = roi_margins(box_swapped)

            # 체커보드: 왜곡 보정 이미지에서 검출 (피시아이에서 안정적)
            found, corners = cv2.findChessboardCorners(
                gray_undist, (BOARD_W, BOARD_H),
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            display = undist.copy()

            if found:
                corners2 = cv2.cornerSubPix(
                    gray_undist, corners, (11, 11), (-1, -1), criteria)

                laser_pixels, (rx1, ry1, rx2, ry2) = extract_laser_line(
                    undist, corners2, margin_x, margin_y, sig_thr, ransac_thr,
                    angle_deg=LINE_ANGLE,
                )

                cv2.drawChessboardCorners(
                    display, (BOARD_W, BOARD_H), corners2, True)

                urx1, ury1 = int(rx1), int(ry1)
                urx2, ury2 = int(rx2), int(ry2)
                cv2.rectangle(display, (urx1, ury1), (urx2, ury2), (255, 200, 0), 1)

                b_txt, b_col = "체커보드 OK", (0, 255, 0)
            else:
                laser_pixels = []
                b_txt, b_col = "체커보드 미감지", (0, 100, 255)

            if debug_signal:
                sig = color_signal(undist)
                signal_vis = cv2.normalize(sig, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                cv2.imshow(debug_window_name, signal_vis)
            else:
                if cv2.getWindowProperty(debug_window_name, cv2.WND_PROP_VISIBLE) >= 1:
                    cv2.destroyWindow(debug_window_name)

            if laser_pixels:
                for (u, v) in laser_pixels:
                    cv2.circle(display, (int(u), int(v)), 2, (0, 0, 255), -1)

            l_col = (0, 255, 0) if len(laser_pixels) > 10 else (0, 100, 255)
            unit  = "점"
            l_txt = f"레이저 {len(laser_pixels)}{unit}" if laser_pixels else "레이저 미감지"

            cv2.rectangle(display, (0, 0), (W, 100), (0, 0, 0), -1)
            cv2.putText(display, f"[{COLOR.upper()}] {b_txt}", (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, b_col, 2)
            cv2.putText(display, f"{l_txt} | 캡처: {captured}/{MIN_CAPTURES}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, l_col, 2)
            cell_tag = "FAR" if cell_is_far else "NEAR"
            cv2.putText(display,
                        f"ROI {margin_x}x{margin_y}px [t]  cell {active_cell:.0f}mm({cell_tag}) [b]",
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
            if captured >= MIN_CAPTURES:
                cv2.putText(display, "c: 레이저 평면 계산",
                            (10, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow(window_name, display)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord('q')):
                break

            elif key == ord('t'):
                box_swapped = not box_swapped
                mx, my = roi_margins(box_swapped)
                print(f"  ROI 여유 전환 → 가로 {mx}px / 세로 {my}px")

            elif key == ord('b'):
                cell_is_far = not cell_is_far
                active_cell = CELL_FAR if cell_is_far else CELL_NEAR
                objp = make_objp(active_cell)
                tag = "원거리(FAR)" if cell_is_far else "근거리(NEAR)"
                print(f"  셀 크기 전환 → {active_cell:.0f}mm {tag} "
                      f"(보드를 실제로 바꿔 끼웠는지 확인!)")

            elif key == ord(' '):
                if not found:
                    print("  체커보드 미감지")
                    continue
                if len(laser_pixels) < 10:
                    print(f"  레이저 픽셀 부족 ({len(laser_pixels)}{unit})")
                    continue

                norm_xy = normalized_image_points_rectified(corners2, inv_new_K)
                ok, rvec, tvec = cv2.solvePnP(
                    objp, norm_xy,
                    np.eye(3), np.zeros((4, 1))
                )
                if not ok:
                    print("  solvePnP 실패")
                    continue

                R, _ = cv2.Rodrigues(rvec)
                t = tvec.ravel()
                plane_n = R[:, 2]
                plane_d = float(np.dot(plane_n, t))

                added = 0
                for (u, v) in laser_pixels[::2]:
                    ray = pixel_to_ray_rectified(u, v, inv_new_K)
                    pt3d = ray_plane_intersect(ray, plane_n, plane_d)
                    if pt3d is not None:
                        laser_3d_pts.append(pt3d)
                        added += 1

                captured += 1
                cap_per_cell[active_cell] = cap_per_cell.get(active_cell, 0) + 1
                print(f"  [{captured}] +{added}점 (누적 {len(laser_3d_pts)}점) "
                      f"@ {active_cell:.0f}mm")

            elif key == ord('c'):
                if captured < MIN_CAPTURES:
                    print(f"  캡처 부족 ({captured}/{MIN_CAPTURES})")
                    continue

                pts = np.array(laser_3d_pts, dtype=np.float64)
                print(f"\n{len(pts)}개 점으로 {COLOR} 레이저 평면 피팅 중...")

                centroid = pts.mean(axis=0)
                _, _, Vt = np.linalg.svd(pts - centroid)
                normal = Vt[-1]
                d_val  = float(np.dot(normal, centroid))

                residuals = np.abs(pts @ normal - d_val)
                rmse      = float(np.sqrt(np.mean(residuals ** 2)))

                print(f"\n========== {COLOR.upper()} 레이저 평면 결과 ==========")
                print(f"법선 벡터   : [{normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f}]")
                print(f"평면 방정식 : {normal[0]:.4f}x + {normal[1]:.4f}y + {normal[2]:.4f}z = {d_val:.4f}")
                print(f"RMSE        : {rmse:.4f} mm")
                print(f"사용 점수   : {len(pts)}")
                cap_summary = ", ".join(f"{c:.0f}mm x{n}"
                                        for c, n in sorted(cap_per_cell.items()))
                print(f"캡처 구성   : {cap_summary}")
                print("==============================================\n")

                data = {
                    "laser_plane": {
                        "color":      COLOR,
                        "angle_deg":  LINE_ANGLE,
                        "normal":     normal.tolist(),
                        "d":          d_val,
                        "equation":   f"{normal[0]:.6f}x + {normal[1]:.6f}y + {normal[2]:.6f}z = {d_val:.4f}",
                        "rmse_mm":    rmse,
                        "num_points": len(pts),
                        "captures_per_cell_mm": {f"{c:.0f}": n
                                                 for c, n in sorted(cap_per_cell.items())},
                    },
                    "detection_params": {
                        "method":     "ROI + color separation + peak per scanline + RANSAC",
                        "SIGNAL_THR": int(sig_thr),
                        "RANSAC_THR": int(ransac_thr),
                    }
                }
                with open(OUTPUT_FILE, "w") as f:
                    yaml.dump(data, f, default_flow_style=False, sort_keys=False)
                print(f"저장 완료: {OUTPUT_FILE}")
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()