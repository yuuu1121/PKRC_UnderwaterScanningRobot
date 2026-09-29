#!/usr/bin/env python3
"""GUI 영상 위에 레이저 검출 결과를 겹쳐 그린다.

검출식은 calibration/calibrate_laser.py 를 그대로 가져왔다 — 캘리브레이션에서
검증된 식과 화면에서 보는 식이 갈리면 임계값 튜닝이 무의미해진다. 바뀐 것은
ROI 뿐이다. 캘리브레이션은 체커보드 코너로 ROI 를 잡지만(extract_laser_line 의
corners 인자) 운용 중 영상에는 체커보드가 없으므로 전체 프레임을 본다.

'전체 프레임' 이 공짜가 아니라는 게 이 파일의 설계를 정한다. 젯슨 8코어에서
1280x720 실측(스트리밍 스레드 1개, 디코드+검출+인코드 왕복):

    legacy   @640   26.2 ms/frame   12fps 시 코어  31.5%
    contrast @640   37.8 ms/frame   12fps 시 코어  45.3%
    legacy   @1280  47.4 ms/frame   12fps 시 코어  56.9%
    contrast @1280  90.5 ms/frame   12fps 시 코어 108.6%  ← 12fps 를 못 맞춘다

그래서 검출 해상도를 고를 수 있게 했다. contrast 는 medianBlur 가 지배적이라
전체 해상도에서 한 코어를 통째로 먹는다. 검출은 축소본에서 하고 그리기는 원본
해상도에 하므로, 화면에 나가는 영상과 bag 에 남는 영상은 언제나 1280x720 이다.
"""

import cv2
import numpy as np

# ── calibrate_laser.py 와 동일한 상수 (:61-79) ──────────────────────────
# 여기 값이 캘리브레이션과 갈리면 화면에서 맞춘 임계값이 캘리브레이션에서
# 재현되지 않는다. 바꿀 일이 생기면 양쪽을 같이 바꿔야 한다.
SIGNAL_THR_LEGACY   = 12     # legacy 기본 임계 (신호 스케일 ~0-200)
SIGNAL_THR_CONTRAST = 4      # contrast 기본 임계 (스케일이 달라 따로 둔다)
CONTRAST_MEDIAN_K   = 31     # 국소 배경 추정 커널. 라인 두께보다 충분히 커야 함
GREEN_GATE_OFFSET   = 10.0
GREEN_GATE_SCALE    = 40.0
MATCH_WIDTH         = 2.0    # 매치드필터가 가정하는 라인 반폭(px)
INTEGRATE_LEN       = 41     # 라인 방향 적분 길이(px)
PEAK_HALFWIN        = 4      # 무게중심을 구할 예측행 주변 반폭(px)
RANSAC_THR          = 5
RANSAC_ITER         = 200    # 캘리브레이션은 500. 화면용은 절반 이하로 충분하다

# 검출 결과 색 — 레이저 원본이 초록이라 초록 오버레이는 원본에 묻힌다.
COLOR_STRONG = (255, 0, 255)    # 마젠타: 임계의 2배 넘는 확실한 검출
COLOR_WEAK   = (200, 120, 200)  # 흐린 마젠타: 임계는 넘었지만 약한 검출
COLOR_FIT    = (0, 220, 255)    # 노랑: RANSAC 직선 (켰을 때만)

_rng = np.random.default_rng(0)


def color_signal(frame):
    """calibrate_laser.py:106 과 동일한 녹색 판별식.

    g - 0.5*(r+b) 로 흰색을 거부하고, 포화 보정으로 saturate 된 강한 라인을
    강제 통과시킨다.
    """
    g = frame[:, :, 1].astype(np.float32)
    r = frame[:, :, 2].astype(np.float32)
    b = frame[:, :, 0].astype(np.float32)

    gd = np.clip(g - 0.5 * (r + b), 0, None)
    sat_green = (g >= 245) & (g > r + 30) & (g > b + 30)
    return np.maximum(gd, sat_green.astype(np.float32) * 200.0)


def contrast_signal(frame, med_k=CONTRAST_MEDIAN_K):
    """calibrate_laser.py:123 과 동일한 밝기 국소대비 x 초록우세 게이트.

    레이저가 과노출로 흰색(G,B 동시 포화)이 되면 색으로는 못 찾는다. 대신
    '주변보다 국소적으로 밝은 얇은 구조' 로 찾는다. 캘리브레이션 주석의 실측:
    같은 구간에서 legacy 1.4시그마 vs contrast 21시그마.
    """
    V = frame.max(axis=2)
    local = cv2.medianBlur(V, med_k).astype(np.float32)
    contrast = np.clip(V.astype(np.float32) - local, 0, None)

    g = frame[:, :, 1].astype(np.float32)
    r = frame[:, :, 2].astype(np.float32)
    gate = np.clip((g - r + GREEN_GATE_OFFSET) / GREEN_GATE_SCALE, 0.0, 1.0)
    return contrast * gate


def line_response(band, width=MATCH_WIDTH, integrate=INTEGRATE_LEN):
    """calibrate_laser.py:151 과 동일. 세로 LoG 매치드필터 + 라인방향 적분."""
    k = int(width * 6) | 1
    y = np.arange(k, dtype=np.float32) - k // 2
    s2 = width ** 2
    ker = ((s2 - y ** 2) / s2 ** 2) * np.exp(-y ** 2 / (2 * s2))
    ker -= ker.mean()
    resp = cv2.filter2D(band, -1, ker.reshape(-1, 1))
    resp = cv2.blur(resp, (integrate, 1))
    return np.clip(resp, 0, None)


def detect_peaks(frame, method, sig_thr):
    """프레임 전체에서 컬럼별 레이저 피크를 찾는다.

    calibrate_laser.py:extract_laser_line 에서 체커보드 ROI 와 회전을 걷어낸
    형태다. 회전을 뺀 이유: 캘리브레이션은 체커보드를 여러 각도로 눕혀 놓고
    찍으므로 라인 각도가 다양하지만, 운용 중에는 카메라와 레이저의 기구
    배치가 고정이라 화면에서 라인이 항상 가로에 가깝다.

    Returns:
        [(x, y, strength), ...] — frame 좌표계 기준. 검출 실패면 빈 리스트.
    """
    sig = contrast_signal(frame) if method == 'contrast' else color_signal(frame)
    h, w = sig.shape
    if h < 4 or w < 4:
        return []

    peaks = []
    if method == 'contrast':
        # 적분응답으로 '어느 컬럼에 라인이 있나' 를 정하고, 무게중심은 적분 전
        # 신호에서 그 행 주변만 보고 구한다 — 적분은 가로로 뭉개므로 위치
        # 정밀도에 쓰면 안 된다 (calibrate_laser.py:209-211).
        resp = line_response(sig)
        arg = resp.argmax(axis=0)
        colmax = resp.max(axis=0)
        for col in range(w):
            if colmax[col] <= sig_thr:
                continue
            yc = int(arg[col])
            lo = max(0, yc - PEAK_HALFWIN)
            hi = min(h, yc + PEAK_HALFWIN + 1)
            seg = sig[lo:hi, col]
            weight = np.clip(seg - seg.min(), 0, None)
            total = weight.sum()
            if total <= 0:
                continue
            row = lo + float(
                np.dot(np.arange(len(weight), dtype=np.float32), weight) / total)
            peaks.append((float(col), row, float(colmax[col])))
    else:
        # 라인이 가로 방향이므로 세로 BG 차분으로 큰 블롭/면적 노이즈를
        # 제거하고 얇은 라인만 살린다 (calibrate_laser.py:228-231).
        bg = cv2.GaussianBlur(sig, (1, 61), 0)
        sig = np.clip(sig - bg, 0, None)

        rows = np.arange(h, dtype=np.float32)
        for col in range(w):
            col_sig = sig[:, col]
            mask = col_sig > sig_thr
            if not mask.any():
                continue
            weight = col_sig[mask]
            row = float(np.dot(rows[mask], weight) / weight.sum())
            peaks.append((float(col), row, float(weight.max())))

    return peaks


def fit_ransac(peaks, thr=RANSAC_THR):
    """검출점에 직선을 맞춘다. calibrate_laser.py:249-263 과 동일한 RANSAC.

    기본으로 끄고 쓴다. 캘리브레이션은 '평면 위의 직선' 이 보장되지만 운용
    중 검사 대상이 곡면이면 직선 가정이 정상 검출점을 버린다.

    Returns:
        (slope, intercept) 또는 실패 시 None.
    """
    if len(peaks) < 4:
        return None
    xs = np.array([p[0] for p in peaks], dtype=np.float32)
    ys = np.array([p[1] for p in peaks], dtype=np.float32)

    best_mask, best_count = None, 0
    for _ in range(RANSAC_ITER):
        idx = _rng.choice(len(xs), 2, replace=False)
        if abs(xs[idx[1]] - xs[idx[0]]) < 1:
            continue
        slope = (ys[idx[1]] - ys[idx[0]]) / (xs[idx[1]] - xs[idx[0]])
        intercept = ys[idx[0]] - slope * xs[idx[0]]
        inliers = np.abs(ys - (slope * xs + intercept)) < thr
        if inliers.sum() > best_count:
            best_count = int(inliers.sum())
            best_mask = inliers

    if best_mask is None or best_count < 4:
        return None
    # 최적 후보의 인라이어로 최소자승 재적합 — 2점 표본보다 안정적이다.
    coef = np.polyfit(xs[best_mask], ys[best_mask], 1)
    return float(coef[0]), float(coef[1])


def draw_overlay(frame, peaks, scale, sig_thr, fit=None):
    """검출 결과를 frame 위에 제자리에서 그린다.

    점만 찍는다 — 선으로 이으면 검출이 끊긴 구간까지 이어져, 임계값이 잘못
    잡혔을 때 그 사실이 화면에서 사라진다. 끊긴 곳은 끊겨 보여야 한다.

    strength 로 색을 나눠 임계값 튜닝이 눈으로 되게 했다: 임계의 2배를 넘으면
    진한 마젠타, 겨우 넘긴 것은 흐린 마젠타. 흐린 점만 잔뜩이면 임계가 낮다.

    Args:
        frame: 원본 BGR (1280x720). 여기 직접 그린다.
        peaks: detect_peaks 결과. 검출 해상도 기준 좌표.
        scale: 검출→원본 배율.
        sig_thr: 색 구분 기준으로 쓰는 임계값.
        fit: fit_ransac 결과 (slope, intercept) 또는 None. 검출 해상도 기준.
    """
    strong_thr = sig_thr * 2.0
    for (x, y, strength) in peaks:
        color = COLOR_STRONG if strength >= strong_thr else COLOR_WEAK
        cv2.circle(frame, (int(x * scale), int(y * scale)), 2, color, -1)

    if fit is not None:
        h, w = frame.shape[:2]
        slope, intercept = fit
        # 검출 좌표계의 직선을 원본 좌표계로 옮긴다. x,y 에 같은 배율이
        # 걸리므로 기울기는 그대로고 절편만 scale 배가 된다.
        y0 = intercept * scale
        y1 = (slope * (w / scale) + intercept) * scale
        cv2.line(frame, (0, int(y0)), (w, int(y1)), COLOR_FIT, 1)

    label = f'{len(peaks)} pts'
    cv2.putText(frame, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 0), 3)
    cv2.putText(frame, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, COLOR_STRONG, 1)


def render(jpeg_bytes, method, sig_thr=None, det_width=640, ransac=False,
           jpeg_quality=80):
    """JPEG 바이트 → 오버레이를 그린 JPEG 바이트.

    이 함수는 스트리밍 스레드 안에서만 불린다. 여기서 만든 이미지는 HTTP
    응답으로만 나가고 ROS 토픽으로 되돌아가지 않는다 — bag 에 남는 것은
    카메라가 발행한 원본이지 이 결과가 아니다.

    Args:
        jpeg_bytes: 카메라가 발행한 원본 JPEG.
        method: 'legacy' 또는 'contrast'.
        sig_thr: 임계값. None 이면 method 별 기본값.
        det_width: 검출을 수행할 가로 해상도. 원본보다 작으면 축소해서
            검출하고 좌표를 되돌린다. 그리기는 언제나 원본 해상도.
        ransac: 직선 피팅 표시 여부.
        jpeg_quality: 재인코딩 품질.

    Returns:
        오버레이가 그려진 JPEG 바이트. 디코딩 실패 시 입력을 그대로 반환한다
        (화면이 멈추는 것보다 오버레이 없는 영상이 낫다).
    """
    if sig_thr is None:
        sig_thr = (SIGNAL_THR_CONTRAST if method == 'contrast'
                   else SIGNAL_THR_LEGACY)

    frame = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return jpeg_bytes

    h, w = frame.shape[:2]
    if 0 < det_width < w:
        det = cv2.resize(frame, (det_width, max(1, h * det_width // w)))
        scale = w / float(det_width)
    else:
        det = frame
        scale = 1.0

    peaks = detect_peaks(det, method, sig_thr)
    fit = fit_ransac(peaks) if ransac else None
    draw_overlay(frame, peaks, scale, sig_thr, fit)

    ok, buf = cv2.imencode('.jpg', frame,
                           [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    return buf.tobytes() if ok else jpeg_bytes
