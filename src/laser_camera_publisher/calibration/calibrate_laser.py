#!/usr/bin/env python3
"""
녹색 레이저 평면 캘리브레이션 (Tkinter GUI 버전)

전제:
    카메라(/dev/video0)를 직접 연다 — laser_camera_publisher 가 켜져 있으면
    장치를 못 여니 먼저 꺼야 한다 (pkill -f laser_camera_publisher).

사용법:
    python3 calibrate_laser.py
    python3 calibrate_laser.py --device /dev/video0 --exposure 50 --angle 0

- 단일 Tkinter 창: 근/원거리 셀 전환, 임계값 슬라이더,
  ROI 스왑, 디버그 신호 뷰, 캡처/평면 계산 버튼
- 검출 방식 토글: 기존 녹색판별(legacy) ↔ 신규 밝기대비(contrast)
- 키보드 단축키: SPACE(캡처) b(셀 전환) t(ROI 스왑) m(검출방식) c(평면 계산) q/ESC(종료)
- 체커보드를 여러 위치/각도에 놓고 캡처 → SVD로 레이저 평면 도출
  → green_laser_plane.yaml 저장
"""

import argparse
import subprocess
import threading
import tkinter as tk
from tkinter import messagebox

import cv2
import numpy as np
import yaml

# ─── 인자 파싱 ───────────────────────────
parser = argparse.ArgumentParser(description="녹색 레이저 평면 캘리브레이션 (Tkinter GUI)")
# 기본 장치는 camera_node.py 와 같은 by-path — 하방 카메라(같은 exploreHD,
# 같은 시리얼) 추가로 /dev/videoN 번호가 밀리므로 번호로 지정하지 않는다.
parser.add_argument("--device",
                    default=("/dev/v4l/by-path/"
                             "platform-3610000.usb-usb-0:2.1.2:1.0-video-index0"),
                    help="카메라 장치. 기본: 레이저 exploreHD (by-path).")
parser.add_argument("--exposure", type=int, default=50,
                    help="수동 노출값 (v4l2 exposure_time_absolute). 기본 50.")
parser.add_argument("--angle", type=float, default=0.0,
                    help="레이저 라인 각도(도). 가로=0. 대각선이면 45 등으로 설정.")
parser.add_argument("--cell-near", type=float, default=40.0,
                    help="근거리 체커보드 셀 크기(mm). 기본 40.")
parser.add_argument("--cell-far", type=float, default=70.0,
                    help="원거리 체커보드 셀 크기(mm). 기본 70.")
args = parser.parse_args()
CELL_NEAR  = float(args.cell_near)
CELL_FAR   = float(args.cell_far)
LINE_ANGLE = float(args.angle)

# ─── 설정 ────────────────────────────────
INTRINSICS_FILE  = "camera_intrinsics.yaml"
OUTPUT_FILE      = "green_laser_plane.yaml"
BOARD_W          = 5
BOARD_H          = 4
MIN_CAPTURES     = 10

# 레이저 탐지 ROI — 체커 코너 바운딩박스에 더하는 여유(margin, px). t 로 가로/세로 스왑.
MARGIN_X_PX      = 120      # 가로 여유 (px)
MARGIN_Y_PX      = 240      # 세로 여유 (px)
# 색판별+포화보정+BG차분 후의 신호 스케일 기준.
# 라인은 ~150~200, 노이즈/블롭은 BG차분으로 ~5~30 으로 분리됨.
SIGNAL_THR       = 12
RANSAC_THR       = 5
RANSAC_ITER      = 500

# ── 검출 방식 ────────────────────────────
# "legacy"   : 녹색 판별식 g-0.5(r+b) + 세로 BG차분 + 고정 임계
# "contrast" : (밝기 국소대비) x (초록우세 게이트) + 매치드필터 + 라인방향 적분
#
# 레이저가 과노출로 흰색이 되면(R221 G255 B253 처럼 G,B 동시 포화) legacy 의
# 녹색 판별식은 라인 18 vs 배경 7±7.7 = 1.4시그마 로 배경에 묻힌다.
# contrast 는 같은 구간에서 라인 82 vs 배경 1.8±3.8 = 21시그마.
METHOD_DEFAULT   = "legacy"
SIGNAL_THR_CONTRAST = 4      # contrast 모드 권장 임계 (legacy 와 스케일이 다름)
CONTRAST_MEDIAN_K   = 31     # 국소 배경 추정 커널. 라인 두께보다 충분히 커야 함
GREEN_GATE_OFFSET   = 10.0   # g-r 이 이 값 이상이면 게이트가 열리기 시작
GREEN_GATE_SCALE    = 40.0   # 게이트가 완전히 열리는 g-r 폭
MATCH_WIDTH         = 2.0    # 매치드필터가 가정하는 라인 반폭(px)
INTEGRATE_LEN       = 41     # 라인 방향 적분 길이(px). 클수록 SNR↑, 곡선 추종↓
PEAK_HALFWIN        = 4      # 무게중심을 구할 예측행 주변 반폭(px)
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
    """laser_realtime.py 와 동일한 녹색 판별식.

    - g - 0.5*(r+b) 로 흰색 거부 + 약한 신호도 통과
    - 포화 보정으로 saturate 된 강한 라인 강제 통과
    - BG차분은 여기서 X. extract_laser_line() 에서 회전 후에 적용한다
      (원본 좌표계에서는 라인 각도가 다양해서 고정 방향 BG가 부적절).
    """
    g = frame[:, :, 1].astype(np.float32)
    r = frame[:, :, 2].astype(np.float32)
    b = frame[:, :, 0].astype(np.float32)

    gd = np.clip(g - 0.5 * (r + b), 0, None)
    sat_green = (g >= 245) & (g > r + 30) & (g > b + 30)
    return np.maximum(gd, sat_green.astype(np.float32) * 200.0)


def contrast_signal(frame, med_k=CONTRAST_MEDIAN_K):
    """밝기의 국소대비 x 초록우세 게이트.

    과노출로 라인이 흰색(G,B 동시 포화)이 되면 색으로는 못 찾는다.
    대신 '주변보다 국소적으로 밝은 얇은 구조'로 찾는다.

    - V - medianBlur(V) : 창문/조명 같은 '넓은' 밝은 영역은 중앙값에 흡수되어
      0 이 되고, 폭이 커널보다 얇은 라인만 살아남는다.
    - x (g-r) 게이트    : 색이 조금이라도 남아 있으면 활용해 파란/중성 구조물을
      눌러 준다. 완전 무채색이어도 게이트가 0 이 되진 않는다.

    전체 프레임에 쓰면 medianBlur 가 ~128ms 로 비싸다. 반드시 ROI 크롭에만 쓸 것.
    """
    V = frame.max(axis=2)
    local = cv2.medianBlur(V, med_k).astype(np.float32)
    contrast = np.clip(V.astype(np.float32) - local, 0, None)

    g = frame[:, :, 1].astype(np.float32)
    r = frame[:, :, 2].astype(np.float32)
    gate = np.clip((g - r + GREEN_GATE_OFFSET) / GREEN_GATE_SCALE, 0.0, 1.0)
    return contrast * gate


def signal_of(frame, method):
    """검출 방식에 맞는 신호 맵."""
    return contrast_signal(frame) if method == "contrast" else color_signal(frame)


def line_response(band, width=MATCH_WIDTH, integrate=INTEGRATE_LEN):
    """얇은 가로 라인 강조: 세로 LoG 매치드필터 + 라인방향 적분.

    세로 LoG 는 라인 두께에 맞춘 대역통과라 넓은 밝기 변화를 죽인다.
    가로 적분은 사람 눈이 라인을 따라가며 보는 것과 같은 효과로,
    컬럼마다 독립 판단할 때보다 SNR 이 sqrt(적분길이) 배 올라간다.
    """
    k = int(width * 6) | 1
    y = np.arange(k, dtype=np.float32) - k // 2
    s2 = width ** 2
    ker = ((s2 - y ** 2) / s2 ** 2) * np.exp(-y ** 2 / (2 * s2))
    ker -= ker.mean()
    resp = cv2.filter2D(band, -1, ker.reshape(-1, 1))
    resp = cv2.blur(resp, (integrate, 1))
    return np.clip(resp, 0, None)


def extract_laser_line(frame, corners, margin_x, margin_y, sig_thr, ransac_thr,
                       angle_deg=0.0, method=METHOD_DEFAULT):
    """ROI 를 angle_deg 만큼 회전한 좌표계에서 '가로 라인'으로 가정하고
    컬럼별 피크를 잡은 뒤, 역회전하여 원본 이미지 좌표로 돌려준다.

    ROI 는 체커 코너 바운딩박스에 가로 margin_x / 세로 margin_y (px) 여유를 준 직사각형.

    angle_deg = 0  → 원본의 가로 라인
    angle_deg = 45 → 45도 대각 라인
    """
    H, W = frame.shape[:2]
    pts = corners.reshape(-1, 2)
    x1 = max(0, int(pts[:, 0].min()) - margin_x)
    x2 = min(W, int(pts[:, 0].max()) + margin_x)
    y1 = max(0, int(pts[:, 1].min()) - margin_y)
    y2 = min(H, int(pts[:, 1].max()) + margin_y)

    # 신호는 ROI 크롭에만 계산한다 (contrast 의 medianBlur 가 전체프레임이면 ~128ms).
    roi = signal_of(frame[y1:y2, x1:x2], method)
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

    peaks = []
    if method == "contrast":
        # 적분응답으로 '어느 컬럼에 라인이 있나'를 정하고,
        # 무게중심은 적분 전 신호에서 그 행 주변만 보고 구한다
        # (적분은 가로로 뭉개므로 위치 정밀도에 쓰면 안 된다).
        resp = line_response(rot)
        arg = resp.argmax(axis=0)
        colmax = resp.max(axis=0)
        for col in range(new_w):
            if colmax[col] <= sig_thr:
                continue
            yc = int(arg[col])
            lo = max(0, yc - PEAK_HALFWIN)
            hi = min(new_h, yc + PEAK_HALFWIN + 1)
            seg = rot[lo:hi, col]
            w = np.clip(seg - seg.min(), 0, None)
            if w.sum() <= 0:
                continue
            peak_row = lo + float(np.dot(np.arange(len(w), dtype=np.float32), w) / w.sum())
            peaks.append((float(col), peak_row, float(colmax[col])))
    else:
        # 회전 후 라인은 가로 방향이므로 세로 BG 차분으로
        # 큰 블롭/면적 노이즈를 제거하고 얇은 라인만 살린다.
        bg = cv2.GaussianBlur(rot, (1, 61), 0)
        rot = np.clip(rot - bg, 0, None)

        rows_rot = np.arange(new_h, dtype=np.float32)
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


# ─── 직접 캡처 ────────────────────────────
# ROS 토픽 경유(publish→DDS→imdecode)의 지연을 없애려고 camera_node.py 와
# 같은 GStreamer MJPG 파이프라인으로 장치를 직접 연다. OpenCV V4L2 백엔드는
# 이 카메라에서 20fps 로 막히므로(camera_node.py:33-36 실측) 반드시 GStreamer.

class DirectCamera:
    """camera_node.py 와 동일한 파이프라인으로 /dev/videoX 를 직접 읽는다.

    appsink 가 JPEG 바이트를 주므로 스레드에서 imdecode 까지 해서
    current_frame 에 BGR 최신 프레임 하나만 유지한다 (기존 image_cb 와 동일).
    """

    def __init__(self, device, exposure):
        self.device = device
        self.current_frame = None
        self._running = True

        # camera_node.py:53-57 과 같은 순서 — 포맷/크기/fps 는 파이프라인
        # caps 가, 노출은 v4l2-ctl 이 담당한다 (cap.set 은 GStreamer 백엔드를
        # 통과하지 못한다).
        self._v4l2_ctl("auto_exposure=1")
        self._v4l2_ctl(f"exposure_time_absolute={exposure}")

        pipeline = (
            f"v4l2src device={device} io-mode=2 ! "
            f"image/jpeg,width=1280,height=720,framerate=30/1 ! "
            f"jpegparse ! appsink drop=true max-buffers=1 sync=false"
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"{device} 열기 실패 — laser_camera_publisher 가 켜져 있으면 "
                f"장치를 점유합니다. 먼저 끄세요: pkill -f laser_camera_publisher")

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _v4l2_ctl(self, ctrl):
        r = subprocess.run(["v4l2-ctl", "-d", self.device, f"--set-ctrl={ctrl}"],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f"  v4l2-ctl {ctrl} 실패: {r.stderr.strip()}")

    def _loop(self):
        while self._running:
            ret, buf = self.cap.read()   # 1D uint8 JPEG 바이트 (jpegparse 출력)
            if not ret or buf is None:
                continue
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if frame is not None:
                self.current_frame = frame

    def release(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.cap.release()


# ─── GUI ─────────────────────────────────

class CalibApp:
    UPDATE_MS = 30
    MAX_DISPLAY_W = 1280   # 표시 폭이 이보다 크면 절반 축소

    def __init__(self, root, node):
        self.root = root
        self.node = node
        root.title("Green Laser Calibration")

        # 캘리브레이션 상태
        self.laser_3d_pts = []
        self.captured = 0
        self.cap_per_cell = {}   # {셀크기mm: 캡처수} — 근/원 보드 기여도 추적

        # undistort 맵 (프레임 크기 기준 lazy 계산)
        self.calib_size = None
        self.map1 = self.map2 = None
        self.inv_new_K = None

        # 최근 처리 프레임의 검출 결과 (캡처 시 사용)
        self.last_found = False
        self.last_corners = None
        self.last_laser_pixels = []

        self.photo = None        # PhotoImage 참조 유지 (GC 방지)
        self.closing = False

        # ── 셀 크기 (근/원거리) ──
        cell = tk.Frame(root)
        cell.pack(fill=tk.X, padx=8, pady=2)
        tk.Label(cell, text="셀 크기:").pack(side=tk.LEFT)
        self.cell_var = tk.StringVar(value="far")
        tk.Radiobutton(cell, text=f"원거리 {CELL_FAR:.0f}mm", value="far",
                       variable=self.cell_var, command=self.on_cell_change
                       ).pack(side=tk.LEFT)
        tk.Radiobutton(cell, text=f"근거리 {CELL_NEAR:.0f}mm", value="near",
                       variable=self.cell_var, command=self.on_cell_change
                       ).pack(side=tk.LEFT)
        tk.Label(cell, text="(보드를 실제로 바꿔 끼웠는지 확인!)",
                 fg="gray40").pack(side=tk.LEFT, padx=8)
        self.objp = make_objp(self.active_cell())

        # ── 검출 방식 토글 ──
        mf = tk.Frame(root)
        mf.pack(fill=tk.X, padx=8, pady=2)
        tk.Label(mf, text="검출 방식:").pack(side=tk.LEFT)
        self.method_var = tk.StringVar(value=METHOD_DEFAULT)
        tk.Radiobutton(mf, text="기존: 녹색판별 (legacy)", value="legacy",
                       variable=self.method_var, command=self.on_method_change
                       ).pack(side=tk.LEFT)
        tk.Radiobutton(mf, text="신규: 밝기대비 (contrast)", value="contrast",
                       variable=self.method_var, command=self.on_method_change
                       ).pack(side=tk.LEFT)
        tk.Label(mf, text="[m] 로 전환", fg="gray40").pack(side=tk.LEFT, padx=8)

        # ── 임계값 슬라이더 ──
        sl = tk.Frame(root)
        sl.pack(fill=tk.X, padx=8, pady=2)
        self.sig_var = tk.IntVar(value=SIGNAL_THR)
        self.ransac_var = tk.IntVar(value=RANSAC_THR)
        tk.Label(sl, text="SIGNAL_THR").pack(side=tk.LEFT)
        tk.Scale(sl, from_=1, to=50, orient=tk.HORIZONTAL,
                 variable=self.sig_var, length=160).pack(side=tk.LEFT, padx=(0, 12))
        tk.Label(sl, text="RANSAC_THR").pack(side=tk.LEFT)
        tk.Scale(sl, from_=1, to=20, orient=tk.HORIZONTAL,
                 variable=self.ransac_var, length=120).pack(side=tk.LEFT)

        # ── 토글 ──
        tg = tk.Frame(root)
        tg.pack(fill=tk.X, padx=8, pady=2)
        self.swap_var = tk.BooleanVar(value=False)
        self.debug_var = tk.BooleanVar(value=False)
        tk.Checkbutton(tg, text="ROI 가로/세로 스왑 (t)",
                       variable=self.swap_var).pack(side=tk.LEFT)
        tk.Checkbutton(tg, text="신호 디버그 뷰",
                       variable=self.debug_var).pack(side=tk.LEFT, padx=12)

        # ── 영상 ──
        self.video = tk.Label(root, text="프레임 대기 중…",
                              bg="black", fg="white", width=80, height=24)
        self.video.pack(padx=8, pady=4)

        # ── 버튼 + 상태 ──
        bt = tk.Frame(root)
        bt.pack(fill=tk.X, padx=8, pady=2)
        tk.Button(bt, text="캡처 (SPACE)", command=self.capture).pack(side=tk.LEFT)
        self.compute_btn = tk.Button(bt, text=f"평면 계산 (C)", command=self.compute)
        self.compute_btn.pack(side=tk.LEFT, padx=8)

        self.status_var = tk.StringVar(value="프레임 대기 중…")
        tk.Label(root, textvariable=self.status_var, anchor="w"
                 ).pack(fill=tk.X, padx=8, pady=(2, 8))

        root.bind("<Key>", self.on_key)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(self.UPDATE_MS, self.update)

    # ── 상태 헬퍼 ──

    def active_cell(self):
        return CELL_FAR if self.cell_var.get() == "far" else CELL_NEAR

    def on_method_change(self):
        """방식 전환. 두 방식은 신호 스케일이 달라 임계값도 같이 옮겨 준다."""
        m = self.method_var.get()
        self.sig_var.set(SIGNAL_THR_CONTRAST if m == "contrast" else SIGNAL_THR)
        name = "밝기대비(contrast)" if m == "contrast" else "녹색판별(legacy)"
        print(f"  검출 방식 → {name} (임계값 {self.sig_var.get()} 로 자동 조정)")

    def on_cell_change(self):
        self.objp = make_objp(self.active_cell())
        tag = "원거리(FAR)" if self.cell_var.get() == "far" else "근거리(NEAR)"
        print(f"  셀 크기 전환 → {self.active_cell():.0f}mm {tag} "
              f"(보드를 실제로 바꿔 끼웠는지 확인!)")

    def on_key(self, event):
        if isinstance(event.widget, tk.Entry):
            return                        # 입력 위젯에서는 단축키 무시
        k = event.keysym.lower()
        if k == "space":
            self.capture()
        elif k == "b":
            self.cell_var.set("near" if self.cell_var.get() == "far" else "far")
            self.on_cell_change()
        elif k == "t":
            self.swap_var.set(not self.swap_var.get())
        elif k == "m":
            self.method_var.set(
                "legacy" if self.method_var.get() == "contrast" else "contrast")
            self.on_method_change()
        elif k == "c":
            self.compute()
        elif k in ("q", "escape"):
            self.close()

    # ── 주기 처리 ──

    def build_maps(self, W, H):
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, (W, H), np.eye(3), balance=0.0)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), new_K, (W, H), cv2.CV_16SC2)
        self.inv_new_K = np.linalg.inv(new_K)
        self.calib_size = (W, H)
        print(f"undistort 맵 생성: {W} x {H}")

    def update(self):
        if self.closing:
            return

        frame = self.node.current_frame
        if frame is None:
            self.status_var.set(f"프레임 대기 중… (장치: {self.node.device})")
            self.root.after(self.UPDATE_MS, self.update)
            return

        frame = frame.copy()
        H, W = frame.shape[:2]
        if self.calib_size != (W, H):
            self.build_maps(W, H)

        undist = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)

        # 체커보드 검출용 그레이.
        # 초록 레이저는 G 채널을 강하게 때려서 보드 흑백 대비를 망가뜨림.
        # → R/B 평균만 사용 (G 채널 드롭).
        b_ch, _, r_ch = cv2.split(undist)
        gray_undist = cv2.addWeighted(r_ch, 0.5, b_ch, 0.5, 0)

        sig_thr    = max(1, self.sig_var.get())
        ransac_thr = max(1, self.ransac_var.get())
        margin_x, margin_y = roi_margins(self.swap_var.get())

        found, corners = cv2.findChessboardCorners(
            gray_undist, (BOARD_W, BOARD_H),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        display = undist.copy()
        corners2 = None
        laser_pixels = []

        if found:
            corners2 = cv2.cornerSubPix(
                gray_undist, corners, (11, 11), (-1, -1), criteria)

            laser_pixels, (rx1, ry1, rx2, ry2) = extract_laser_line(
                undist, corners2, margin_x, margin_y, sig_thr, ransac_thr,
                angle_deg=LINE_ANGLE, method=self.method_var.get(),
            )

            cv2.drawChessboardCorners(
                display, (BOARD_W, BOARD_H), corners2, True)
            cv2.rectangle(display, (int(rx1), int(ry1)), (int(rx2), int(ry2)),
                          (255, 200, 0), 1)
            b_txt = "체커보드 OK"
        else:
            b_txt = "체커보드 미감지"

        self.last_found = found
        self.last_corners = corners2
        self.last_laser_pixels = laser_pixels

        for (u, v) in laser_pixels:
            cv2.circle(display, (int(u), int(v)), 2, (0, 0, 255), -1)

        if self.debug_var.get():
            # contrast 의 medianBlur 는 전체프레임에서 ~128ms 라 절반 해상도로 본다.
            small = cv2.resize(undist, None, fx=0.5, fy=0.5,
                               interpolation=cv2.INTER_AREA)
            sig = signal_of(small, self.method_var.get())
            vis = cv2.normalize(sig, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            display = cv2.cvtColor(cv2.resize(vis, (W, H)), cv2.COLOR_GRAY2BGR)

        l_txt = f"레이저 {len(laser_pixels)}점" if laser_pixels else "레이저 미감지"
        cell_tag = "FAR" if self.cell_var.get() == "far" else "NEAR"
        m_tag = "밝기대비" if self.method_var.get() == "contrast" else "녹색판별"
        self.status_var.set(
            f"{b_txt} · {l_txt} · [{m_tag}] · 캡처 {self.captured}/{MIN_CAPTURES} · "
            f"셀 {self.active_cell():.0f}mm({cell_tag}) · ROI {margin_x}x{margin_y}px"
        )

        self.show_image(display)
        self.root.after(self.UPDATE_MS, self.update)

    def show_image(self, img):
        if img.shape[1] > self.MAX_DISPLAY_W:
            img = cv2.resize(img, None, fx=0.5, fy=0.5,
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".ppm", img)
        if not ok:
            return
        self.photo = tk.PhotoImage(data=buf.tobytes())
        self.video.config(image=self.photo, text="", width=img.shape[1],
                          height=img.shape[0])

    # ── 캡처 / 평면 계산 ──

    def capture(self):
        if not self.last_found or self.last_corners is None:
            print("  체커보드 미감지")
            return
        if len(self.last_laser_pixels) < 10:
            print(f"  레이저 픽셀 부족 ({len(self.last_laser_pixels)}점)")
            return

        norm_xy = normalized_image_points_rectified(self.last_corners, self.inv_new_K)
        ok, rvec, tvec = cv2.solvePnP(
            self.objp, norm_xy,
            np.eye(3), np.zeros((4, 1))
        )
        if not ok:
            print("  solvePnP 실패")
            return

        R, _ = cv2.Rodrigues(rvec)
        t = tvec.ravel()
        plane_n = R[:, 2]
        plane_d = float(np.dot(plane_n, t))

        added = 0
        for (u, v) in self.last_laser_pixels[::2]:
            ray = pixel_to_ray_rectified(u, v, self.inv_new_K)
            pt3d = ray_plane_intersect(ray, plane_n, plane_d)
            if pt3d is not None:
                self.laser_3d_pts.append(pt3d)
                added += 1

        cell = self.active_cell()
        self.captured += 1
        self.cap_per_cell[cell] = self.cap_per_cell.get(cell, 0) + 1
        print(f"  [{self.captured}] +{added}점 (누적 {len(self.laser_3d_pts)}점) "
              f"@ {cell:.0f}mm")

    def compute(self):
        if self.captured < MIN_CAPTURES:
            msg = f"캡처 부족 ({self.captured}/{MIN_CAPTURES})"
            print(f"  {msg}")
            messagebox.showwarning("평면 계산", msg)
            return

        pts = np.array(self.laser_3d_pts, dtype=np.float64)
        print(f"\n{len(pts)}개 점으로 green 레이저 평면 피팅 중...")

        centroid = pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(pts - centroid)
        normal = Vt[-1]
        d_val  = float(np.dot(normal, centroid))

        residuals = np.abs(pts @ normal - d_val)
        rmse      = float(np.sqrt(np.mean(residuals ** 2)))

        cap_summary = ", ".join(f"{c:.0f}mm x{n}"
                                for c, n in sorted(self.cap_per_cell.items()))

        print(f"\n========== GREEN 레이저 평면 결과 ==========")
        print(f"법선 벡터   : [{normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f}]")
        print(f"평면 방정식 : {normal[0]:.4f}x + {normal[1]:.4f}y + {normal[2]:.4f}z = {d_val:.4f}")
        print(f"RMSE        : {rmse:.4f} mm")
        print(f"사용 점수   : {len(pts)}")
        print(f"캡처 구성   : {cap_summary}")
        print("==============================================\n")

        data = {
            "laser_plane": {
                "color":      "green",
                "angle_deg":  LINE_ANGLE,
                "normal":     normal.tolist(),
                "d":          d_val,
                "equation":   f"{normal[0]:.6f}x + {normal[1]:.6f}y + {normal[2]:.6f}z = {d_val:.4f}",
                "rmse_mm":    rmse,
                "num_points": len(pts),
                "captures_per_cell_mm": {f"{c:.0f}": n
                                         for c, n in sorted(self.cap_per_cell.items())},
            },
            "detection_params": {
                "mode":       self.method_var.get(),
                "method": (
                    "ROI + local-contrast x green-gate + matched filter + "
                    "line-direction integration + RANSAC"
                    if self.method_var.get() == "contrast" else
                    "ROI + color separation + peak per scanline + RANSAC"
                ),
                "SIGNAL_THR": int(max(1, self.sig_var.get())),
                "RANSAC_THR": int(max(1, self.ransac_var.get())),
            }
        }
        with open(OUTPUT_FILE, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        print(f"저장 완료: {OUTPUT_FILE}")

        messagebox.showinfo(
            "평면 계산 완료",
            f"저장 완료: {OUTPUT_FILE}\n\n"
            f"RMSE: {rmse:.4f} mm\n"
            f"사용 점수: {len(pts)}\n"
            f"캡처 구성: {cap_summary}"
        )

    def close(self):
        self.closing = True
        self.root.destroy()


# ─── 메인 ────────────────────────────────

def main():
    node = DirectCamera(args.device, args.exposure)

    print(f"라인 각도 : {LINE_ANGLE:.1f}도")
    print(f"장치      : {args.device} (노출 {args.exposure})")
    print(f"출력 파일 : {OUTPUT_FILE}")
    print(f"검출 방식 : {METHOD_DEFAULT} (GUI 라디오버튼 또는 m 키로 전환)")
    print("조작법: SPACE(캡처) b(셀 전환) t(ROI 스왑) m(검출방식) c(평면 계산) q/ESC(종료)\n")

    root = tk.Tk()
    CalibApp(root, node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.release()


if __name__ == '__main__':
    main()
