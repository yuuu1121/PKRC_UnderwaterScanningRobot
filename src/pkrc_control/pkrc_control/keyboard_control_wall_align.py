#!/usr/bin/env python3
"""
PKRC Wall-Align Teleop — 원본 robust teleop + C키 벽면 정렬 시퀀스
================================================================================
keyboard_control_robust_update_good.py 를 그대로 유지하고, C 키 한 개를 얹었다.
수동 조작(↑/↓/←/→/A/D/W/S)은 원본과 100% 동일하게 동작한다.

C 키 시퀀스
  SCAN     제자리 360° 회전하며 매 소나 샘플의 (yaw, 거리) 기록
  BRAKE    스캔 회전 관성 제거 (없으면 TURN 이 목표를 25° 지나침 — 실측)
  TURN     기록 중 최소거리 방위로 heading hold 회전
  APPROACH surge 거리 PID 로 1.00 m 접근
  HOLD     거리 0.97~1.03 m 를 5초 이상 유지하면 진입.
           그 자리에서 heading 고정 + 거리 PID 로 1 m 능동 유지.

중단 조건 (요청 4)
  시퀀스 진행 중 ↑/↓/←/→/A/D/W/S/R/T/X 중 아무 키나 누르면 그 즉시 IDLE 로
  떨어지고 자동 surge 출력이 0 이 된다. 그 키의 원래 동작은 정상 수행된다.
  (C 를 다시 누르는 것도 취소)

surge 축 소유권
  IDLE  : ↑/↓ 수동 (원본과 동일)
  APPROACH/HOLD : 거리 PID 전용. 이 상태에서 ↑/↓ 를 누르면 위 규칙에 따라
                  시퀀스가 먼저 중단되고, 그 다음 수동 surge 가 걸린다.

yaw 는 quaternion 이 아니라 자이로 적분이다
  이 장비(GV7-INS)는 GNSS·자기계가 없어 EKF 가 Vertical Gyro 모드로 돌고
  yaw 를 추정하지 않는다 — quaternion yaw 는 회전해도 제자리로 끌려간다
  (실측: 손으로 60° 왕복 → yaw 6.8° 만 변함, gyro z 는 ±30°/s 로 정상).
  360° 를 누적해야 하는 SCAN 이 이 결함에 직결되므로 yaw 만 gyro z 적분으로
  대체하고, 기동 시 정지 상태에서 자이로 bias 를 매번 새로 측정한다.

이 코드는 평면 벽 전용이다. 곡면(원통 내벽 등)은 이동하면 법선 방향이 계속
바뀌는데 단일빔으로는 곡률을 추정할 정보가 없다.
"""

import math
import select
import sys
import termios
import tty

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Float32, Float64MultiArray, String
from pkrc_control.live_tuning import setup_live_tuning, _FIXED

try:
    from dvl_msgs.msg import DVL
    _HAS_DVL_MSG = True
except ImportError:
    DVL = None
    _HAS_DVL_MSG = False


# ─── 유틸 ───────────────────────────────────────────────────────────────
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ─── Heading Hold (캐스케이드 PID) ─────────────────────────────────────
class HeadingHoldController:
    """
    외부 P :  angle_error            → target_yaw_rate  (max_rate 로 포화)
    내부 PID: (target_rate - gyro_z) → yaw_command ∈ [-1, +1]
    """

    def __init__(self,
                 angle_kp=5.0,
                 angle_ki=1.0,
                 rate_kp=3.5,
                 rate_ki=0.005,
                 rate_kd=0.85,
                 rate_aw=1.0,
                 max_rate=1.2,
                 max_integral=2.0,
                 deadband_deg=0.6,
                 smoothing=0.5,
                 output_slew=4.0):
        self.angle_kp = angle_kp
        self.angle_ki = angle_ki
        self.rate_kp = rate_kp
        self.rate_ki = rate_ki
        self.rate_kd = rate_kd
        self.rate_aw = rate_aw
        self.max_rate = max_rate
        self.max_integral = max_integral
        self.deadband = math.radians(deadband_deg)
        self.smoothing = smoothing
        self.output_slew = output_slew
        self.max_output = 1.0
        self.max_angle_integral = (0.5 * max_rate / angle_ki
                                   if angle_ki > 0.0 else 0.0)

        self.integral = 0.0
        self.angle_integral = 0.0
        self.prev_rate_err = 0.0
        self.prev_output = 0.0

    def compute(self, yaw_error, yaw_rate, dt):
        if dt <= 0.0:
            return self.prev_output

        err = normalize_angle(yaw_error)

        if abs(err) < self.deadband:
            self.prev_output *= 0.9
            self.angle_integral *= 0.98
            return self.prev_output

        err_adj = err - math.copysign(self.deadband, err)

        p_rate = self.angle_kp * err_adj
        if abs(p_rate) < self.max_rate:
            self.angle_integral += err_adj * dt
        if err_adj * self.angle_integral < 0.0:
            self.angle_integral *= 0.9
        self.angle_integral = clamp(self.angle_integral,
                                    -self.max_angle_integral,
                                    self.max_angle_integral)

        target_rate = clamp(p_rate + self.angle_ki * self.angle_integral,
                            -self.max_rate, self.max_rate)

        rate_err = target_rate - yaw_rate

        p = self.rate_kp * rate_err

        self.integral += rate_err * dt
        if rate_err * self.integral < 0.0:
            self.integral *= 0.9
        self.integral = clamp(self.integral,
                              -self.max_integral, self.max_integral)
        i = self.rate_ki * self.integral

        d_raw = clamp((rate_err - self.prev_rate_err) / dt, -5.0, 5.0)
        d = self.rate_kd * d_raw
        self.prev_rate_err = rate_err

        unsat = p + i + d
        raw = clamp(unsat, -self.max_output, self.max_output)

        self.integral += self.rate_aw * (raw - unsat) * dt
        out = self.smoothing * raw + (1.0 - self.smoothing) * self.prev_output

        max_step = self.output_slew * dt
        out = clamp(out,
                    self.prev_output - max_step,
                    self.prev_output + max_step)

        self.prev_output = out
        return out

    def reset(self):
        self.integral = 0.0
        self.angle_integral = 0.0
        self.prev_rate_err = 0.0
        self.prev_output = 0.0


# ─── Yaw Rate Controller (A/D 수동 회전 + SCAN/BRAKE) ──────────────────
class YawRateController:
    """목표 각속도에 정속 추종하는 순수 rate PID."""

    def __init__(self,
                 kp=2.0,
                 ki=2.5,
                 kd=0.10,
                 max_output=0.5,
                 max_integral=0.6,
                 smoothing=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.max_integral = max_integral
        self.smoothing = smoothing

        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_output = 0.0

    def compute(self, target_rate, yaw_rate, dt):
        if dt <= 0.0:
            return self.prev_output

        err = target_rate - yaw_rate

        p = self.kp * err

        if abs(self.prev_output) < self.max_output * 0.95:
            self.integral += err * dt
        self.integral = clamp(self.integral,
                              -self.max_integral, self.max_integral)
        i = self.ki * self.integral

        d_raw = clamp((err - self.prev_err) / dt, -5.0, 5.0)
        d = self.kd * d_raw
        self.prev_err = err

        raw = clamp(p + i + d, -self.max_output, self.max_output)
        out = self.smoothing * raw + (1.0 - self.smoothing) * self.prev_output
        self.prev_output = out
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_output = 0.0


# ─── Depth Hold (캐스케이드 PID) ───────────────────────────────────────
class CascadedDepthController:
    """외부 P : depth_error → target_vel, 내부 PID: → heave cmd."""

    def __init__(self,
                 pos_kp=2.0,
                 vel_kp=1.2,
                 vel_ki=0.3,
                 vel_kd=0.15,
                 max_velocity=0.4,
                 max_integral=0.5,
                 smoothing=0.4,
                 vel_filter_alpha=0.4):
        self.pos_kp = pos_kp
        self.vel_kp = vel_kp
        self.vel_ki = vel_ki
        self.vel_kd = vel_kd
        self.max_velocity = max_velocity
        self.max_integral = max_integral
        self.smoothing = smoothing
        self.vel_filter_alpha = vel_filter_alpha
        self.max_output = 1.0

        self.integral = 0.0
        self.prev_vel_err = 0.0
        self.prev_output = 0.0
        self.prev_depth = None
        self.estimated_velocity = 0.0

    def update_velocity(self, current_depth, sensor_dt):
        """센서 콜백 (10Hz) 에서 호출 — 센서 rate 로 velocity 추정."""
        if self.prev_depth is not None and sensor_dt > 0.001:
            raw_vel = (current_depth - self.prev_depth) / sensor_dt
            raw_vel = clamp(raw_vel, -2.0, 2.0)
            self.estimated_velocity = (
                self.vel_filter_alpha * raw_vel
                + (1.0 - self.vel_filter_alpha) * self.estimated_velocity)
        self.prev_depth = current_depth

    def compute(self, depth_error, current_depth, dt):
        if dt <= 0.0:
            return self.prev_output

        target_vel = clamp(self.pos_kp * depth_error,
                           -self.max_velocity, self.max_velocity)

        vel_err = target_vel - self.estimated_velocity

        p = self.vel_kp * vel_err
        if abs(self.prev_output) < self.max_output * 0.9:
            self.integral += vel_err * dt
        if vel_err * self.integral < 0.0:
            self.integral *= 0.85
        self.integral = clamp(self.integral,
                              -self.max_integral, self.max_integral)
        i = self.vel_ki * self.integral

        d_raw = clamp((vel_err - self.prev_vel_err) / dt, -2.0, 2.0)
        d = self.vel_kd * d_raw
        self.prev_vel_err = vel_err

        raw = clamp(p + i + d, -self.max_output, self.max_output)
        out = self.smoothing * raw + (1.0 - self.smoothing) * self.prev_output
        self.prev_output = out
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_vel_err = 0.0
        self.prev_output = 0.0
        self.prev_depth = None
        self.estimated_velocity = 0.0


# ─── Wall Distance Controller (소나 → surge) ───────────────────────────
class WallDistanceController:
    """
    벽면 거리 유지 PID. err = current_dist - target_dist
      err > 0 (멀다)   → surge +  (전진, 벽에 접근)
      err < 0 (가깝다) → surge -  (후진)

    depth hold 와 달리 캐스케이드가 아닌 단일 PID인 이유: 소나 거리 미분은
    벽면 요철·빔 산란에 오염되어 속도 추정이 depth(압력) 만큼 신뢰할 수 없다.

    simplified: 단일 루프 PID. 조류 등 정상 외란이 크면 DVL surge 속도를
    내부 루프로 넣는 캐스케이드로 승격 필요.
    """

    def __init__(self,
                 kp=0.8,
                 ki=0.15,
                 kd=0.25,
                 max_output=0.45,
                 max_integral=0.8,
                 deadband=0.03,
                 smoothing=0.35,
                 d_filter_alpha=0.3,
                 output_slew=1.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.max_integral = max_integral
        self.deadband = deadband
        self.smoothing = smoothing
        self.d_filter_alpha = d_filter_alpha
        self.output_slew = output_slew

        self.integral = 0.0
        self.prev_err = None
        self.filtered_d = 0.0
        self.prev_output = 0.0

    def compute(self, dist_error, dt):
        if dt <= 0.0:
            return self.prev_output

        # 데드밴드 — 소나 분해능·벽면 요철 수준의 오차는 무시해 채터링 방지
        if abs(dist_error) < self.deadband:
            self.prev_output *= 0.85
            self.integral *= 0.97
            self.prev_err = dist_error
            return self.prev_output

        err = dist_error - math.copysign(self.deadband, dist_error)

        p = self.kp * err

        self.integral += err * dt
        if err * self.integral < 0.0:
            self.integral *= 0.9
        self.integral = clamp(self.integral,
                              -self.max_integral, self.max_integral)
        i = self.ki * self.integral

        # D — 10Hz 센서를 50Hz 루프에서 미분하면 4/5 루프는 0, 1/5 는 spike
        if self.prev_err is None:
            d_raw = 0.0
        else:
            d_raw = clamp((err - self.prev_err) / dt, -2.0, 2.0)
        self.filtered_d = (self.d_filter_alpha * d_raw
                           + (1.0 - self.d_filter_alpha) * self.filtered_d)
        d = self.kd * self.filtered_d
        self.prev_err = err

        unsat = p + i + d
        raw = clamp(unsat, -self.max_output, self.max_output)

        if self.ki > 0.0:
            self.integral += (raw - unsat) * dt

        out = self.smoothing * raw + (1.0 - self.smoothing) * self.prev_output

        max_step = self.output_slew * dt
        out = clamp(out,
                    self.prev_output - max_step,
                    self.prev_output + max_step)

        self.prev_output = out
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_err = None
        self.filtered_d = 0.0
        self.prev_output = 0.0


# ─── Main Node ─────────────────────────────────────────────────────────
class KeyboardTeleopWallAlign(Node):
    # 정렬 시퀀스 상태
    IDLE = 'IDLE'          # 원본 teleop 동작 (수동 surge 포함)
    SCAN = 'SCAN'          # 제자리 360° 회전하며 최소거리 방위 탐색
    BRAKE = 'BRAKE'        # 스캔 회전 관성 제거 — TURN 진입 전 정지
    TURN = 'TURN'          # 찾은 방위로 heading 회전
    APPROACH = 'APPROACH'  # target_distance 로 접근
    HOLD = 'HOLD'          # 도달 지점에서 자세유지 (거리 1m + heading)

    # 시퀀스를 중단시키는 수동 조작 키 (요청 4).
    # 'c' 는 제외 — C 는 시작/취소 토글이라 아래 키 처리에서 따로 다룬다.
    ABORT_KEYS = frozenset(
        ('UP', 'DOWN', 'LEFT', 'RIGHT', 'a', 'd', 'w', 's', 'r', 't', 'x'))

    def __init__(self):
        # 런타임 노드 이름을 파일·실행파일 이름과 일치시킨다 — 둘이 다르면
        # ros2 param set / 서비스 경로를 찾을 때 어느 쪽인지 매번 헷갈린다.
        super().__init__('keyboard_control_wall_align')

        # ── Parameters ─────────────────────────────────────────────────
        # Heading hold — 원본(update_good) 값보다 낮춘 것.
        # 이유: 이 차체는 yaw 명령이 실제 회전으로 나타나기까지 0.5초 지연이
        # 있다(로그 cmd↔yaw_rate 상호상관 lag 0.5s 에서 r=+0.979).
        # 원본 게인은 이 지연에서 위상여유가 없어 TURN 이 목표를 25° 지나치고
        # 1.45초 주기로 자기발진한다(출력 포화 18/56, 전류 3.0A 한계 도달).
        # 원본 teleop 은 heading 을 크게 돌릴 일이 없어 이 결함이 안 드러났다.
        self.declare_parameter('hh_angle_kp', 1.0)
        self.declare_parameter('hh_angle_ki', 0)
        self.declare_parameter('hh_rate_kp', 0.8)
        self.declare_parameter('hh_rate_ki', 0.01)
        self.declare_parameter('hh_rate_kd', 0.8)
        self.declare_parameter('hh_rate_aw', 1.0)
        self.declare_parameter('hh_max_rate', 0.20)
        self.declare_parameter('hh_deadband_deg', 0.1)
        self.declare_parameter('hh_output_slew', 4.0)
        # 정지 직후 heading hold 과출력 진동 억제 (실측 확정)
        self.declare_parameter('idle_output_scale', 0.15)
        self.declare_parameter('idle_grace_sec', 0.0)
        # 키 뗌 후 이 시간까지는 그 순간 방위를 target 으로 붙들어 자세유지한다.
        # 이후에는 target 을 현재 heading 으로 끌고 간다 — yaw 가 자이로 적분이라
        # 절대 기준이 없어 시간이 갈수록 측정값이 흘러가고(분당 약 7°, 실측),
        # 없앨 수 없는 그 오차를 계속 쫓으면 로봇이 제자리에서 혼자 돈다.
        self.declare_parameter('heading_hold_sec', 300.0)

        # Yaw rate (A/D)
        self.declare_parameter('yr_target_rate', 0.30)   # rad/s ≈ 17°/s
        self.declare_parameter('yr_kp', 2.0)
        self.declare_parameter('yr_ki', 2.5)
        self.declare_parameter('yr_kd', 0.10)
        self.declare_parameter('yr_max_output', 0.50)

        # ── 벽면 정렬 시퀀스 ───────────────────────────────────────────
        self.declare_parameter('sonar_topic',
                               '/sensor/sonar/ping1d/data', _FIXED)
        self.declare_parameter('sonar_conf_topic',
                               '/sensor/sonar/ping1d/confidence', _FIXED)
        self.declare_parameter('target_distance', 1.0)      # [m] 유지 거리
        self.declare_parameter('wd_kp', 0.8)
        self.declare_parameter('wd_ki', 0.15)
        self.declare_parameter('wd_kd', 0.25)
        self.declare_parameter('wd_max_output', 0.45)       # surge 출력 한계
        self.declare_parameter('wd_deadband', 0.03)         # [m] 3cm
        self.declare_parameter('wd_output_slew', 1.5)
        # 소나 유효성 게이팅
        self.declare_parameter('sonar_min_range', 0.15)     # [m] 이 미만 무효
        # [m] 이 초과는 무효. launch 의 scan_length(소나 자체 측정 상한)와
        # 함께 올려야 의미가 있다 — 둘 중 작은 쪽이 실제 상한이 된다.
        self.declare_parameter('sonar_max_range', 5.0)
        self.declare_parameter('sonar_min_confidence', 50.0)  # [%]
        self.declare_parameter('sonar_stale_sec', 0.6)      # 10Hz → 6샘플 유실
        self.declare_parameter('sonar_lpf_alpha', 0.4)      # 거리 LPF
        self.declare_parameter('sonar_jump_limit', 0.5)     # [m] 샘플간 최대변화
        # 자이로 bias 측정 — yaw 를 자이로 적분으로 만들기 때문에 필요
        self.declare_parameter('gyro_bias_sec', 3.0)        # [s] 측정 시간
        # 정지 판정 임계 [deg/s] — 표본 표준편차가 이보다 크면 재측정.
        # 실측: 정지 0.4, 손으로 회전 62 → 그 사이.
        self.declare_parameter('gyro_bias_max_std_deg', 1.0)
        # bias 크기 상한 [deg/s] — std 만으로는 등속 회전을 못 거른다
        # (흩어짐이 아니라 치우침이라 std 가 작다).
        self.declare_parameter('gyro_bias_max_deg', 10.0)
        # 시퀀스 파라미터
        self.declare_parameter('scan_yaw_rate', 0.20)       # [rad/s] 스캔 속도
        # 스캔 회전 전류 한계 [A] — 스캔 중엔 surge/sway 가 0 이라 yaw 가
        # 수평 4기 전체에 배분되므로 스러스터 1기당 이 전류가 상한이 된다.
        self.declare_parameter('scan_max_current', 1.2)     # [A]
        self.declare_parameter('scan_sweep_rad', 2.0 * math.pi)  # 1회전
        # 스캔 종료 후 감속(BRAKE) — 스캔은 11.5°/s 로 돌던 중에 끝나는데,
        # 그 관성을 안고 TURN 에 들어가면 목표를 25° 지나친다(실측).
        self.declare_parameter('brake_rate_tol_deg', 3.0)   # 이하면 정지로 봄
        self.declare_parameter('brake_settle_sec', 0.3)     # 유지 시간
        self.declare_parameter('brake_timeout_sec', 5.0)    # 못 멈추면 포기
        self.declare_parameter('scan_timeout_sec', 90.0)    # 스캔 실패 시 중단
        self.declare_parameter('align_tol_deg', 2.0)        # TURN 종료 각도
        self.declare_parameter('align_settle_sec', 1.0)     # 허용 내 유지 시간
        # HOLD 진입 조건 (요청 3): 0.97~1.03m 를 5초 이상 유지
        self.declare_parameter('approach_tol', 0.03)        # [m] ±3cm
        self.declare_parameter('approach_settle_sec', 5.0)  # [s] 5초
        self.declare_parameter('approach_timeout_sec', 60.0)
        # APPROACH 진입 직후 소나 회복 유예 [s] — TURN 이 lost 상태로 끝나는
        # 경우가 흔해, 유예가 없으면 정렬에 성공하고도 23ms 만에 abort 된다.
        self.declare_parameter('approach_grace_sec', 3.0)

        # Feedforward — pitch→heave 만 유지 (기하학적 투영 보상)
        self.declare_parameter('ff_pitch_to_heave', 2.5)

        # Depth hold (W/S → target_depth ∓depth_step)
        self.declare_parameter('depth_pos_kp', 2.0)
        self.declare_parameter('depth_vel_kp', 1.2)
        self.declare_parameter('depth_vel_ki', 0.3)
        self.declare_parameter('depth_vel_kd', 0.15)
        self.declare_parameter('depth_step', 0.10)
        self.declare_parameter('water_density', 1000.0, _FIXED)
        self.declare_parameter('gravity', 9.81, _FIXED)
        self.declare_parameter('atmospheric_pressure_pa', 101325.0, _FIXED)

        # 입력 크기
        self.declare_parameter('surge_cmd', 0.5)
        self.declare_parameter('sway_cmd', 0.5)

        # DVL 기반 측방 드리프트 보상 (일자 주행)
        self.declare_parameter('dvl_topic', '/dvl/data', _FIXED)
        self.declare_parameter('dvl_enabled', True)
        self.declare_parameter('dvl_drift_kp', 0.8)
        self.declare_parameter('dvl_drift_ki', 0.6)
        self.declare_parameter('dvl_drift_max', 0.35)
        self.declare_parameter('dvl_drift_max_int', 0.5)
        self.declare_parameter('dvl_vel_lpf_alpha', 0.35)
        self.declare_parameter('dvl_vel_deadband', 0.03)
        self.declare_parameter('dvl_stale_timeout', 0.5)
        self.declare_parameter('dvl_min_cmd', 0.10)

        # 전류 한계 (A)
        self.declare_parameter('max_current_surge', 3.0)
        self.declare_parameter('max_current_sway', 3.0)
        self.declare_parameter('max_current_heave', 5.0)

        # 스러스터별 추력 캘리브레이션 게인 [T1..T6] ∈ [0, 1]
        # 규약: 센 쪽을 깎아서(derate) 약한 쪽에 맞춘다 — 1.0 초과 금지.
        self.declare_parameter('thruster_gain',
                               [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        # 파라미터 로드
        self.hh = HeadingHoldController(
            angle_kp=self.get_parameter('hh_angle_kp').value,
            angle_ki=self.get_parameter('hh_angle_ki').value,
            rate_kp=self.get_parameter('hh_rate_kp').value,
            rate_ki=self.get_parameter('hh_rate_ki').value,
            rate_kd=self.get_parameter('hh_rate_kd').value,
            rate_aw=self.get_parameter('hh_rate_aw').value,
            max_rate=self.get_parameter('hh_max_rate').value,
            deadband_deg=self.get_parameter('hh_deadband_deg').value,
            output_slew=self.get_parameter('hh_output_slew').value)
        self.yr = YawRateController(
            kp=self.get_parameter('yr_kp').value,
            ki=self.get_parameter('yr_ki').value,
            kd=self.get_parameter('yr_kd').value,
            max_output=self.get_parameter('yr_max_output').value)
        self.wd = WallDistanceController(
            kp=self.get_parameter('wd_kp').value,
            ki=self.get_parameter('wd_ki').value,
            kd=self.get_parameter('wd_kd').value,
            max_output=self.get_parameter('wd_max_output').value,
            deadband=self.get_parameter('wd_deadband').value,
            output_slew=self.get_parameter('wd_output_slew').value)

        self.yr_target_rate = self.get_parameter('yr_target_rate').value
        # A/D 수동 회전용 원래 한계 — 스캔이 yr.max_output 을 일시적으로
        # 낮추므로, 스캔 종료 시 이 값으로 되돌린다.
        self.yr_max_output = self.get_parameter('yr_max_output').value
        self.ff_pitch_to_heave = self.get_parameter('ff_pitch_to_heave').value
        self.idle_output_scale = self.get_parameter('idle_output_scale').value
        self.idle_grace_sec = self.get_parameter('idle_grace_sec').value
        self.heading_hold_sec = self.get_parameter('heading_hold_sec').value
        self.surge_cmd = self.get_parameter('surge_cmd').value
        self.sway_cmd = self.get_parameter('sway_cmd').value

        self.target_distance = self.get_parameter('target_distance').value
        self.sonar_min_range = self.get_parameter('sonar_min_range').value
        self.sonar_max_range = self.get_parameter('sonar_max_range').value
        self.sonar_min_confidence = self.get_parameter(
            'sonar_min_confidence').value
        self.sonar_stale_sec = self.get_parameter('sonar_stale_sec').value
        self.sonar_lpf_alpha = self.get_parameter('sonar_lpf_alpha').value
        self.sonar_jump_limit = self.get_parameter('sonar_jump_limit').value
        self.brake_rate_tol = math.radians(
            self.get_parameter('brake_rate_tol_deg').value)
        self.brake_settle_sec = self.get_parameter('brake_settle_sec').value
        self.brake_timeout_sec = self.get_parameter('brake_timeout_sec').value
        self.bias_sec = self.get_parameter('gyro_bias_sec').value
        self.bias_max_std = math.radians(
            self.get_parameter('gyro_bias_max_std_deg').value)
        self.bias_max_mag = math.radians(
            self.get_parameter('gyro_bias_max_deg').value)
        self.scan_yaw_rate = self.get_parameter('scan_yaw_rate').value
        self.scan_max_current = self.get_parameter('scan_max_current').value
        self.scan_sweep_rad = self.get_parameter('scan_sweep_rad').value
        self.scan_timeout_sec = self.get_parameter('scan_timeout_sec').value
        self.align_tol = math.radians(
            self.get_parameter('align_tol_deg').value)
        self.align_settle_sec = self.get_parameter('align_settle_sec').value
        self.approach_tol = self.get_parameter('approach_tol').value
        self.approach_settle_sec = self.get_parameter(
            'approach_settle_sec').value
        self.approach_timeout_sec = self.get_parameter(
            'approach_timeout_sec').value
        self.approach_grace_sec = self.get_parameter(
            'approach_grace_sec').value

        self.dvl_enabled = (self.get_parameter('dvl_enabled').value
                            and _HAS_DVL_MSG)
        self.dvl_drift_kp = self.get_parameter('dvl_drift_kp').value
        self.dvl_drift_ki = self.get_parameter('dvl_drift_ki').value
        self.dvl_drift_max = self.get_parameter('dvl_drift_max').value
        self.dvl_drift_max_int = self.get_parameter('dvl_drift_max_int').value
        self.dvl_vel_lpf_alpha = self.get_parameter('dvl_vel_lpf_alpha').value
        self.dvl_vel_deadband = self.get_parameter('dvl_vel_deadband').value
        self.dvl_stale_timeout = self.get_parameter('dvl_stale_timeout').value
        self.dvl_min_cmd = self.get_parameter('dvl_min_cmd').value

        self.depth_step = self.get_parameter('depth_step').value
        self.water_density = self.get_parameter('water_density').value
        self.gravity = self.get_parameter('gravity').value
        self.atm_pressure_pa = self.get_parameter(
            'atmospheric_pressure_pa').value

        self.depth_ctrl = CascadedDepthController(
            pos_kp=self.get_parameter('depth_pos_kp').value,
            vel_kp=self.get_parameter('depth_vel_kp').value,
            vel_ki=self.get_parameter('depth_vel_ki').value,
            vel_kd=self.get_parameter('depth_vel_kd').value)

        ms = self.get_parameter('max_current_surge').value
        mw = self.get_parameter('max_current_sway').value
        mh = self.get_parameter('max_current_heave').value
        self.max_current = [ms, ms, mw, mw, mh, mh]
        self.thruster_gain = [clamp(g, 0.0, 1.0) for g in
                              self.get_parameter('thruster_gain').value]
        self.min_current_horizontal = 0.3   # VESC 데드존 보상

        # ── 실시간 파라미터 튜닝 ────────────────────────────────────────
        self.declare_parameter('tuning_gui', True)
        setup_live_tuning(self)

        # ── CAN ────────────────────────────────────────────────────────
        try:
            self.bus = can.interface.Bus(channel='can0', interface='socketcan')
            self.get_logger().info('CAN bus initialized (can0)')
        except Exception as e:
            self.get_logger().error(f'CAN init failed: {e}')
            self.bus = None

        # ── Thrusters + TAM ────────────────────────────────────────────
        self.vesc_ids = {
            'surge_left':  0x151,
            'surge_right': 0x152,
            'sway_left':   0x153,
            'sway_right':  0x154,
            'heave_up':    0x155,
            'heave_down':  0x156,
        }

        #              surge  sway  heave  roll  pitch  yaw
        self.tam = [
            [+1.0,   0.0,   0.0,  0.0,   0.0,  -1.0],  # T1 surge_left
            [+1.0,   0.0,   0.0,  0.0,   0.0,  +1.0],  # T2 surge_right
            [ 0.0,  +1.0,   0.0,  0.0,   0.0,  +1.0],  # T3 sway_left
            [ 0.0,  +1.0,   0.0,  0.0,   0.0,  -1.0],  # T4 sway_right
            [ 0.0,   0.0,  -1.0,  0.0,   0.0,   0.0],  # T5 heave_up
            [ 0.0,   0.0,  +1.0,  0.0,   0.0,   0.0],  # T6 heave_down
        ]

        # 모터 배선/회전방향 극성 보정 — TAM은 기구학(추력 방향)만 표현하고,
        # "그 추력을 내려면 어떤 부호의 전류를 흘려야 하는가"는 여기서 보정.
        self.thruster_polarity = [+1.0, +1.0, +1.0, -1.0, +1.0, +1.0]

        # ── Input / state ──────────────────────────────────────────────
        self.force = [0.0, 0.0, 0.0]
        self.axis_last_time = [0.0, 0.0, 0.0]
        self.yaw_key_sign = 0.0
        self.yaw_last_time = 0.0
        self.was_yawing = False
        self.was_translating = False
        # 마지막으로 heading 을 잠근 시각 — 이때부터 heading_hold_sec 동안
        # target_yaw 를 붙든다 (그 후 자이로 적분 오차 추종으로 넘어감).
        self.heading_lock_time = 0.0

        self.key_timeout = 0.40       # 키 release timeout
        self.decay_rate = 0.85
        self.last_key = ''

        # IMU state — yaw 는 자이로 적분 (클래스 docstring 참조)
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_yaw_rate = 0.0
        self.target_yaw = 0.0
        self.yaw_initialized = False
        self.yaw_prev_time = None
        self.gyro_bias = None
        self.bias_samples = []
        self.bias_start_time = None
        self.last_control_time = None
        self.last_motion_time = 0.0

        # Depth state
        self.current_pressure = 0.0
        self.current_depth = 0.0
        self.filtered_depth = 0.0
        self.target_depth = 0.0
        self.depth_initialized = False
        self.depth_filter_alpha = 0.2
        self.last_pressure_time = None
        self.pressure_last_rx = 0.0
        self.pressure_stale_sec = 2.0
        self.manual_heave_cmd = 0.0
        self.manual_heave_step = 0.25
        self.manual_heave_decay = 0.85
        self.manual_heave_last_key = 0.0
        self._warned_depth_dead = False

        # ── Sonar state ────────────────────────────────────────────────
        self.sonar_raw = 0.0
        self.sonar_dist = 0.0          # LPF 적용 거리
        self.sonar_conf = 0.0
        self.sonar_last_rx = 0.0
        self.sonar_valid = False       # 최근 샘플이 게이팅 통과했는가
        self._warned_sonar_dead = False

        # ── 정렬 시퀀스 state ──────────────────────────────────────────
        self.mode = self.IDLE
        self.mode_enter_time = 0.0
        self.scan_accum_yaw = 0.0      # unwrapped 누적 회전각 (부호 유지)
        self.scan_prev_yaw = None
        self.scan_best_dist = None
        self.scan_best_yaw = None
        self.scan_sample_count = 0
        self.brake_ok_since = None
        self.align_ok_since = None
        self.approach_ok_since = None

        # DVL body-frame velocity state
        # R_dvl_to_body = [[0,0,1],[-1,0,0],[0,1,0]]  (ukfm_localization 참조)
        self.dvl_body_vx = 0.0     # surge velocity, forward +
        self.dvl_body_vy = 0.0     # sway  velocity, port +
        self.dvl_last_time = 0.0
        self.dvl_valid = False
        self.drift_int_surge = 0.0
        self.drift_int_sway = 0.0

        # 스러스터 ramping
        self.actual = {n: 0.0 for n in self.vesc_ids}
        self.ramp_step = 0.5

        # ── ROS I/O ────────────────────────────────────────────────────
        # ── 웹 GUI 키 입력 ─────────────────────────────────────────────
        # gui_server 가 /gui/key 로 키를 보낸다. stdin(터미널) 경로는 그대로
        # 살아 있고, 이건 추가 입력 채널일 뿐이다 — 둘 중 아무거나 써도 된다.
        # C 키(정렬 시퀀스)와 중단 키도 이 경로로 들어온다.
        self._pending_gui_key = ''
        self.create_subscription(String, '/gui/key', self._gui_key_cb, 10)

        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(FluidPressure, '/bar10xt/pressure',
                                 self.pressure_callback,
                                 qos_profile_sensor_data)
        # ping1d_node 는 RELIABLE QoS 로 퍼블리시 → 맞춰야 수신됨
        # (Bar30/DVL 의 BEST_EFFORT 와 다르다 — 실기 확인된 차이)
        sonar_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.create_subscription(
            Float32, self.get_parameter('sonar_topic').value,
            self.sonar_callback, sonar_qos)
        self.create_subscription(
            Float32, self.get_parameter('sonar_conf_topic').value,
            self.sonar_conf_callback, sonar_qos)

        if self.dvl_enabled:
            dvl_topic = self.get_parameter('dvl_topic').value
            dvl_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10)
            self.create_subscription(
                DVL, dvl_topic, self.dvl_callback, dvl_qos)
            self.get_logger().info(
                f'DVL drift compensation enabled on {dvl_topic} '
                f'(BEST_EFFORT)')
        elif not _HAS_DVL_MSG:
            self.get_logger().warn(
                'dvl_msgs not importable — DVL drift comp disabled')

        self.force_pub = self.create_publisher(
            Float64MultiArray, '/teleop/force', 10)
        self.thruster_pub = self.create_publisher(
            Float64MultiArray, '/teleop/thruster_currents', 10)
        self.key_pub = self.create_publisher(String, '/teleop/key', 10)
        self.debug_pub = self.create_publisher(
            Float64MultiArray, '/teleop/yaw_debug', 10)
        self.depth_debug_pub = self.create_publisher(
            Float64MultiArray, '/teleop/depth_debug', 10)
        self.wall_debug_pub = self.create_publisher(
            Float64MultiArray, '/teleop/wall_debug', 10)
        self.mode_pub = self.create_publisher(
            String, '/teleop/wall_mode', 10)

        # ── Terminal ───────────────────────────────────────────────────
        self.running = True
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except termios.error:
            self.settings = None

        # 50 Hz control loop
        self.timer = self.create_timer(0.02, self.control_loop)
        self.print_instructions()

    # ─── UI ─────────────────────────────────────────────────────────
    def print_instructions(self):
        print("""
╔═══════════════════════════════════════════════════════╗
║  PKRC Wall-Align Teleop - 원본 + C키 벽면 정렬        ║
╠═══════════════════════════════════════════════════════╣
║   C    : 벽면 정렬 시작 / 취소                        ║
║          360° 스캔 → 최소거리 방위 회전               ║
║          → 1m 접근 → 5초 유지 → 자세유지              ║
╠═══════════════════════════════════════════════════════╣
║   ↑/↓  : Surge (전/후)   -- heading lock              ║
║   ←/→  : Sway  (좌/우)   -- heading lock              ║
║   A/D  : Yaw Rate (우/좌 회전) 0.3 rad/s 스윽 회전    ║
║   W/S  : Depth ∓10cm (depth hold)                     ║
║   R    : Heading 리셋   T : Depth 리셋                ║
║   X    : 정지 (모든 축 + 제어기 + 모드 리셋)          ║
║   Q    : 종료                                         ║
╠═══════════════════════════════════════════════════════╣
║   ⚠ 정렬 진행 중 위 조작키를 누르면 즉시 중단됩니다   ║
╚═══════════════════════════════════════════════════════╝
""")

    # ─── IMU ────────────────────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        """roll/pitch 는 EKF quaternion, yaw 는 자이로 적분.

        이 장비(GV7-INS)는 GNSS 도 자기계도 없어 EKF 가 `Vertical Gyro`
        모드로 동작한다 — 중력으로 관측 가능한 roll/pitch 만 추정하고
        **yaw 는 추정하지 않는다**. 그래서 quaternion 의 yaw 는 회전해도
        따라오지 않고 제자리로 끌려간다(실측: 손으로 60° 왕복 → yaw 6.8°만
        변함, 반면 gyro z 는 ±30°/s 로 정상 반응). 360° 를 누적해야 하는
        SCAN 이 이 결함에 직결된다.
        → yaw 만 gyro z 적분으로 대체한다. 상대각만 쓰는 제어라 절대방위는
          필요없고, 바이어스만 제거하면 스캔 구간(≈30초)은 충분히 버틴다.
        """
        o = msg.orientation

        # roll/pitch 는 quaternion 그대로 — Vertical Gyro 가 정확히 잘하는 축
        sinp = clamp(2.0 * (o.w * o.y - o.z * o.x), -1.0, 1.0)
        self.current_pitch = math.asin(sinp)

        sinr = 2.0 * (o.w * o.x + o.y * o.z)
        cosr = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
        self.current_roll = math.atan2(sinr, cosr)

        gz = msg.angular_velocity.z
        now = self.get_clock().now().nanoseconds * 1e-9

        # ① 기동 직후: 정지 상태에서 자이로 바이어스를 측정한다.
        #    MEMS 자이로의 바이어스는 온도·전원 사이클마다 달라져
        #    상수로 박을 수 없다 — 매 기동 새로 잰다(실측 예: -2.4°/s).
        if self.gyro_bias is None:
            self._collect_gyro_bias(gz, now)
            return

        self.current_yaw_rate = gz - self.gyro_bias

        # ② 바이어스를 뺀 각속도를 적분해 yaw 를 만든다.
        if self.yaw_prev_time is not None:
            dt = now - self.yaw_prev_time
            # 큰 dt 는 콜백 유실·일시정지 구간 — 적분하면 튄다
            if 0.0 < dt < 0.5:
                self.current_yaw = normalize_angle(
                    self.current_yaw + self.current_yaw_rate * dt)
        self.yaw_prev_time = now

        if not self.yaw_initialized:
            self.target_yaw = self.current_yaw
            self.hh.reset()
            self.yaw_initialized = True
            self.get_logger().info(
                f'Heading initialized: {math.degrees(self.current_yaw):.1f}° '
                f'(자이로 적분, bias {math.degrees(self.gyro_bias):+.2f}°/s)')

    def _collect_gyro_bias(self, gz, now):
        """정지 상태 자이로 표본을 모아 bias 를 확정한다.

        측정 중 로봇이 움직이면 그 표본이 bias 에 섞여 yaw 가 통째로
        틀어진다 — 움직임을 감지하면 표본을 버리고 처음부터 다시 모은다.
        판정은 고정 임계값이 아니라 표본의 표준편차로 한다: 실제 bias 는
        일정한 치우침이라 흩어짐이 작고(실측 폭 0.4°/s), 회전이 섞이면
        흩어짐이 크게 뛴다(실측 62°/s).
        """
        if self.bias_start_time is None:
            self.bias_start_time = now
            self.get_logger().info(
                f'자이로 bias 측정 시작 — {self.bias_sec:.0f}초간 정지하세요')

        self.bias_samples.append(gz)
        elapsed = now - self.bias_start_time
        if elapsed < self.bias_sec:
            return

        n = len(self.bias_samples)
        mean = sum(self.bias_samples) / n
        std = math.sqrt(sum((s - mean) ** 2 for s in self.bias_samples) / n)

        # 흔들림(std)은 불규칙한 움직임을, 크기(mean)는 등속 회전을 거른다
        # — 등속으로 돌면 std 가 작아 std 검사만으로는 통과해 버린다.
        if std > self.bias_max_std or abs(mean) > self.bias_max_mag:
            why = (f'흔들림 {math.degrees(std):.1f}°/s'
                   if std > self.bias_max_std
                   else f'회전 중 {math.degrees(mean):+.1f}°/s')
            self.get_logger().warn(
                f'자이로 bias 측정 실패 ({why}) — 정지 후 재측정')
            self.bias_samples.clear()
            self.bias_start_time = now
            return

        self.gyro_bias = mean
        self.get_logger().info(
            f'자이로 bias 확정: {math.degrees(mean):+.3f}°/s '
            f'(표본 {n}개, 흔들림 {math.degrees(std):.2f}°/s)')

    def dvl_callback(self, msg):
        if not bool(msg.velocity_valid):
            self.dvl_valid = False
            return
        # DVL → body frame (ukfm_localization 의 R_dvl_to_body 동일 적용)
        raw_vx = msg.velocity.z      # body surge
        raw_vy = -msg.velocity.x     # body sway (port +)
        a = self.dvl_vel_lpf_alpha
        self.dvl_body_vx = a * raw_vx + (1.0 - a) * self.dvl_body_vx
        self.dvl_body_vy = a * raw_vy + (1.0 - a) * self.dvl_body_vy
        self.dvl_last_time = self.get_clock().now().nanoseconds / 1e9
        self.dvl_valid = True

    def pressure_callback(self, msg: FluidPressure):
        # fluid_pressure 는 절대압 [Pa]. 수심 = (절대압 - 대기압) / (ρ·g).
        self.current_pressure = msg.fluid_pressure
        gauge_pa = self.current_pressure - self.atm_pressure_pa
        raw_depth = gauge_pa / (self.water_density * self.gravity)
        raw_depth = max(0.0, raw_depth)

        if not self.depth_initialized:
            self.filtered_depth = raw_depth
        else:
            self.filtered_depth = (
                self.depth_filter_alpha * raw_depth
                + (1.0 - self.depth_filter_alpha) * self.filtered_depth)
        self.current_depth = self.filtered_depth

        # 속도 추정을 센서 rate (10Hz) 로 수행 — dt 정확, spike 없음
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_pressure_time is not None:
            sensor_dt = now - self.last_pressure_time
            self.depth_ctrl.update_velocity(self.current_depth, sensor_dt)
        self.last_pressure_time = now

        self.pressure_last_rx = now

        if not self.depth_initialized:
            self.target_depth = self.current_depth
            self.depth_initialized = True
            self.get_logger().info(
                f'Depth initialized: {self.current_depth:.2f}m')

    # ─── Sonar ──────────────────────────────────────────────────────
    def sonar_conf_callback(self, msg: Float32):
        self.sonar_conf = msg.data

    def sonar_callback(self, msg: Float32):
        """게이팅 3단: 범위 → confidence → 점프.

        통과한 샘플만 LPF 에 넣고, SCAN 중이면 (yaw, dist) 최소값 갱신.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        raw = msg.data
        self.sonar_raw = raw

        in_range = self.sonar_min_range <= raw <= self.sonar_max_range
        conf_ok = self.sonar_conf >= self.sonar_min_confidence
        if not (in_range and conf_ok):
            # 무효 샘플 — sonar_last_rx 를 갱신하지 않아 stale 로 흘러간다
            return

        # 점프 필터 — 직전 유효값 대비 급변은 다중경로/기포 반사로 간주.
        # 단 stale 이후 첫 복귀 샘플은 점프 판정에서 면제(연속성 없음).
        fresh = (now - self.sonar_last_rx) < self.sonar_stale_sec
        if fresh and self.sonar_valid and \
                abs(raw - self.sonar_dist) > self.sonar_jump_limit:
            return

        if not self.sonar_valid or not fresh:
            self.sonar_dist = raw          # 복귀 시 필터 초기화
        else:
            a = self.sonar_lpf_alpha
            self.sonar_dist = a * raw + (1.0 - a) * self.sonar_dist

        self.sonar_last_rx = now
        self.sonar_valid = True

        # SCAN 중이면 최소거리 방위 기록 — 소나 콜백 시점의 yaw 를 짝지어야
        # 회전 중 각도-거리 대응이 어긋나지 않는다.
        if self.mode == self.SCAN:
            self.scan_sample_count += 1
            if (self.scan_best_dist is None
                    or self.sonar_dist < self.scan_best_dist):
                self.scan_best_dist = self.sonar_dist
                self.scan_best_yaw = self.current_yaw

    def _scan_output_limit(self):
        """스캔 전류 한계 [A] → YawRateController 의 정규화 출력 한계.

        mix_thrusters 에서 current = out × max_current × gain 이므로
        out ≤ scan_max_current / max_current 면 전류가 한계 이하가 된다.
        """
        max_h = max(self.max_current[0], self.max_current[2])
        if max_h <= 0.0:
            return self.yr_max_output
        return clamp(self.scan_max_current / max_h, 0.0, self.yr_max_output)

    def _sonar_dead(self, now):
        return (not self.sonar_valid
                or self.sonar_last_rx == 0.0
                or (now - self.sonar_last_rx) > self.sonar_stale_sec)

    def _gui_key_cb(self, msg: String):
        """웹 GUI 키를 큐에 넣는다. 다음 get_key() 가 소비한다."""
        self._pending_gui_key = msg.data

    # ─── Keyboard ───────────────────────────────────────────────────
    def get_key(self):
        # 웹 GUI 로 들어온 키를 먼저 소비 (한 번만 반환)
        if self._pending_gui_key:
            k, self._pending_gui_key = self._pending_gui_key, ''
            return k
        if self.settings is None:
            # stdin 이 tty 가 아니면(웹 GUI 가 stdin=DEVNULL 로 띄운 경우)
            # 읽지 않는다. /dev/null 은 항상 readable 이라 select 가 즉시
            # 반환하고 readline() 은 EOF('')를 계속 돌려주는데, 그걸 50Hz
            # 루프에서 반복하면 executor 가 굶어 노드가 멈춘다(실측).
            # 이 경우 조종은 /gui/key 토픽으로만 들어온다.
            if not sys.stdin.isatty():
                return ''
            rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist:
                return sys.stdin.readline().strip()
            return ''
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                _ = sys.stdin.read(1)
                k3 = sys.stdin.read(1)
                key = {'A': 'UP', 'B': 'DOWN',
                       'C': 'RIGHT', 'D': 'LEFT'}.get(k3, key)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # ─── 정렬 시퀀스 ────────────────────────────────────────────────
    def _seq_active(self):
        """자동 시퀀스가 surge/yaw 를 점유 중인가 (HOLD 포함)."""
        return self.mode != self.IDLE

    def _enter_mode(self, mode, now):
        self.mode = mode
        self.mode_enter_time = now
        self.get_logger().info(f'[MODE] → {mode}')

        # 스캔이 yr.max_output 을 전류 한계에 맞춰 낮추므로, 스캔이 아닌
        # 모드로 들어갈 때는 A/D 수동 회전용 원래 한계로 되돌린다.
        if mode != self.SCAN:
            self.yr.max_output = self.yr_max_output

        if mode == self.SCAN:
            self.scan_accum_yaw = 0.0
            self.scan_prev_yaw = self.current_yaw
            self.scan_best_dist = None
            self.scan_best_yaw = None
            self.scan_sample_count = 0
            self.yr.reset()
            self.wd.reset()
        elif mode == self.BRAKE:
            self.brake_ok_since = None
            self.yr.reset()
            self.hh.reset()
            self.get_logger().info(
                f'[SCAN] 최소거리 {self.scan_best_dist:.2f}m @ '
                f'{math.degrees(self.scan_best_yaw):+.1f}° '
                f'({self.scan_sample_count} samples) — 감속 후 정렬')
        elif mode == self.TURN:
            self.align_ok_since = None
            self.target_yaw = self.scan_best_yaw
            self.hh.reset()
            self.yr.reset()
        elif mode == self.APPROACH:
            self.approach_ok_since = None
            self.wd.reset()
        elif mode == self.HOLD:
            self.get_logger().info(
                f'[HOLD] 정렬 완료 — {self.target_distance:.2f}m 를 '
                f'{self.approach_settle_sec:.0f}초 유지 달성. '
                f'heading {math.degrees(self.target_yaw):+.1f}° 자세유지')
        elif mode == self.IDLE:
            self.wd.reset()
            self.yr.reset()

    def _abort_align(self, now, reason):
        self.get_logger().warn(f'[ABORT] {reason} → IDLE')
        self.force[0] = 0.0
        self._enter_mode(self.IDLE, now)

    # ─── TAM Mixing ────────────────────────────────────────────────
    def mix_thrusters(self, cmd, yaw_thrusters=(0, 1, 2, 3)):
        """cmd: [surge, sway, heave, roll, pitch, yaw] ∈ [-1, 1]
        yaw_thrusters: yaw 명령을 배분할 스러스터 인덱스"""
        raw = [0.0] * 6
        for i in range(6):
            for j in range(6):
                if j == 5 and i not in yaw_thrusters:
                    continue
                raw[i] += self.tam[i][j] * cmd[j]

        # 독립 정규화 — yaw 보정 채널을 sway 포화에서 보호
        for a, b in ((0, 1), (2, 3), (4, 5)):
            m = max(abs(raw[a]), abs(raw[b]))
            if m > 1.0:
                raw[a] /= m
                raw[b] /= m

        current = [raw[i] * self.max_current[i] * self.thruster_gain[i]
                   * self.thruster_polarity[i]
                   for i in range(6)]

        # VESC 데드존 보상 (수평 4기)
        for i in range(4):
            if abs(current[i]) < 0.10:
                current[i] = 0.0
            elif abs(current[i]) < self.min_current_horizontal:
                current[i] = math.copysign(
                    self.min_current_horizontal, current[i])
        return current

    def _depth_dead(self, now):
        return (self.pressure_last_rx == 0.0
                or (now - self.pressure_last_rx) > self.pressure_stale_sec)

    def _pub_array(self, pub, data):
        m = Float64MultiArray()
        m.data = data
        pub.publish(m)

    # ─── Control Loop (50 Hz) ──────────────────────────────────────
    def control_loop(self):
        if not self.running:
            self.stop_all()
            rclpy.shutdown()
            return

        now = self.get_clock().now().nanoseconds / 1e9
        key = self.get_key()

        # ── 키 입력 처리 (축별 독립) ─────────────────────────────
        if key:
            self.last_key = (key if key in ('UP', 'DOWN', 'LEFT', 'RIGHT')
                             else key.upper())
            k = key.lower() if len(key) == 1 else key

            # ── 요청 4: 조작키 = 시퀀스 즉시 중단 ────────────────
            # 판정을 키 동작보다 먼저 한다. 그래야 그 키의 원래 동작이
            # 중단된 IDLE 상태에서 정상 수행된다 (예: ↑ 는 시퀀스를 끊고
            # 곧바로 수동 surge 를 건다).
            if k in self.ABORT_KEYS and self._seq_active():
                self.get_logger().warn(
                    f"[ABORT] 수동 조작 '{self.last_key}' → 정렬 중단")
                self.force[0] = 0.0
                self._enter_mode(self.IDLE, now)

            if key == 'UP':
                self.force[0] = +self.surge_cmd
                self.axis_last_time[0] = now
            elif key == 'DOWN':
                self.force[0] = -self.surge_cmd
                self.axis_last_time[0] = now
            elif key == 'LEFT':
                # 실기 검증: sway+ 가 실제로는 우현 방향이라 좌/우 반전
                self.force[1] = -self.sway_cmd
                self.axis_last_time[1] = now
            elif key == 'RIGHT':
                self.force[1] = +self.sway_cmd
                self.axis_last_time[1] = now
            elif k == 'c':
                if self.mode == self.IDLE:
                    if not self.yaw_initialized:
                        self.get_logger().warn(
                            'IMU 미초기화(자이로 bias 측정 중) — 정렬 불가')
                    else:
                        # 수동 surge 잔량이 있으면 거리 PID 와 다투므로 정리
                        self.force[0] = 0.0
                        self._enter_mode(self.SCAN, now)
                else:
                    self.get_logger().info('정렬 취소')
                    self.force[0] = 0.0
                    self._enter_mode(self.IDLE, now)
            elif k == 'w':
                if self._depth_dead(now):
                    # 센서 사망 → 직접 heave 명령 (상승 = -)
                    self.manual_heave_cmd = clamp(
                        self.manual_heave_cmd - self.manual_heave_step,
                        -1.0, 1.0)
                    self.manual_heave_last_key = now
                else:
                    # 상승 = target_depth 감소 (수심이 얕아짐)
                    self.target_depth = max(
                        0.0, self.target_depth - self.depth_step)
                    self.get_logger().info(
                        f'Target depth: {self.target_depth:.2f}m '
                        f'(↑ {self.depth_step * 100:.0f}cm)')
            elif k == 's':
                if self._depth_dead(now):
                    self.manual_heave_cmd = clamp(
                        self.manual_heave_cmd + self.manual_heave_step,
                        -1.0, 1.0)
                    self.manual_heave_last_key = now
                else:
                    # 하강 = target_depth 증가 (수심이 깊어짐)
                    self.target_depth += self.depth_step
                    self.get_logger().info(
                        f'Target depth: {self.target_depth:.2f}m '
                        f'(↓ {self.depth_step * 100:.0f}cm)')
            elif k == 'a':
                # A → 오른쪽 회전 (yaw-)
                self.yaw_key_sign = -1.0
                self.yaw_last_time = now
            elif k == 'd':
                # D → 왼쪽 회전 (yaw+)
                self.yaw_key_sign = +1.0
                self.yaw_last_time = now
            elif k == 'r':
                self.target_yaw = self.current_yaw
                self.heading_lock_time = now
                self.hh.reset()
                self.get_logger().info(
                    f'Heading reset: {math.degrees(self.current_yaw):.1f}° '
                    f'({self.heading_hold_sec:.0f}초간 유지)')
            elif k == 't':
                # 현재 수심을 target 으로 잡고 controller 적분기 리셋
                if self.depth_initialized:
                    self.target_depth = self.current_depth
                    self.depth_ctrl.reset()
                    self.get_logger().info(
                        f'Depth target reset: {self.current_depth:.2f}m')
                else:
                    self.target_depth = 0.0
                    self.depth_ctrl.reset()
                    self.get_logger().info(
                        'Depth target reset: 0.00m (sensor not ready)')
            elif k == 'x':
                self.force = [0.0, 0.0, 0.0]
                self.yaw_key_sign = 0.0
                self.hh.reset()
                self.yr.reset()
                self.wd.reset()
                self.depth_ctrl.reset()
                self.drift_int_surge = 0.0
                self.drift_int_sway = 0.0
                self.manual_heave_cmd = 0.0
                if self.yaw_initialized:
                    self.target_yaw = self.current_yaw
                    self.heading_lock_time = now
                if self.depth_initialized:
                    self.target_depth = self.current_depth
                self.get_logger().info('STOP (all axes + controllers reset)')
            elif k == 'q':
                self.running = False
                return

        # ── 축별 독립 decay (키 떼면 자연스럽게 0으로) ────────────
        # surge / sway 만 decay. heave 는 depth hold 가 상시 관리.
        # APPROACH/HOLD 는 surge 를 거리 PID 가 매 루프 덮어쓰므로 제외.
        seq_owns_surge = self.mode in (self.APPROACH, self.HOLD)
        for i in range(2):
            if i == 0 and seq_owns_surge:
                continue
            if now - self.axis_last_time[i] > self.key_timeout:
                self.force[i] *= self.decay_rate
                if abs(self.force[i]) < 0.05:
                    self.force[i] = 0.0

        # dt 계산 (DVL 적분기 / heading hold 공용)
        dt = 0.02 if self.last_control_time is None \
            else now - self.last_control_time
        self.last_control_time = now

        # ── 소나 상태 판정 ───────────────────────────────────────
        sonar_dead = self._sonar_dead(now)
        if sonar_dead and not self._warned_sonar_dead:
            self.get_logger().warn(
                f'Sonar lost (conf {self.sonar_conf:.0f}%, '
                f'raw {self.sonar_raw:.2f}m)')
            self._warned_sonar_dead = True
        elif (not sonar_dead) and self._warned_sonar_dead:
            self.get_logger().info('Sonar recovered')
            self._warned_sonar_dead = False
            self.wd.reset()

        # ── 수동 yaw 판정 ────────────────────────────────────────
        is_yawing = ((now - self.yaw_last_time) < self.key_timeout
                     and abs(self.yaw_key_sign) > 0.5)

        # ── 정렬 시퀀스 상태머신 ─────────────────────────────────
        # SCAN 은 yaw rate 제어를 쓰므로 is_yawing 과 같은 계층에서 처리.
        scan_yaw_cmd = None    # SCAN 이 yaw 를 점유하면 여기에 값이 들어감
        dist_err = 0.0

        if self.mode == self.SCAN:
            # 누적 회전각 적분 — atan2 불연속(±π) 을 넘어도 정확히 센다.
            if self.scan_prev_yaw is not None:
                self.scan_accum_yaw += normalize_angle(
                    self.current_yaw - self.scan_prev_yaw)
            self.scan_prev_yaw = self.current_yaw

            # 출력 한계를 제어기 "안"에서 조인다 — 바깥에서 clamp 하면
            # 제어기가 포화를 모른 채 적분을 쌓아 windup 이 된다.
            self.yr.max_output = self._scan_output_limit()
            scan_yaw_cmd = self.yr.compute(
                self.scan_yaw_rate, self.current_yaw_rate, dt)
            self.target_yaw = self.current_yaw   # 스캔 중 heading hold 비활성

            if abs(self.scan_accum_yaw) >= self.scan_sweep_rad:
                if self.scan_best_yaw is None:
                    self._abort_align(
                        now, f'스캔 완료했으나 유효 소나 샘플 0개 '
                             f'(범위 {self.sonar_min_range:.2f}~'
                             f'{self.sonar_max_range:.2f}m, '
                             f'conf ≥ {self.sonar_min_confidence:.0f}%)')
                else:
                    self._enter_mode(self.BRAKE, now)
            elif (now - self.mode_enter_time) > self.scan_timeout_sec:
                self._abort_align(now, '스캔 타임아웃')

        elif self.mode == self.BRAKE:
            # 회전이 실제로 멎을 때까지 기다린다. 각속도만 보는 이유:
            # 여기서 heading 오차를 같이 보면 TURN 을 앞당겨 하는 셈이 돼
            # 애초에 없애려던 관성 문제가 그대로 돌아온다.
            if abs(self.current_yaw_rate) < self.brake_rate_tol:
                if self.brake_ok_since is None:
                    self.brake_ok_since = now
                elif (now - self.brake_ok_since) > self.brake_settle_sec:
                    self._enter_mode(self.TURN, now)
            else:
                self.brake_ok_since = None
            if (now - self.mode_enter_time) > self.brake_timeout_sec:
                # 못 멈춰도 정렬은 시도한다 — 스캔 결과를 버리는 것보다 낫다
                self.get_logger().warn(
                    f'[BRAKE] 감속 타임아웃 '
                    f'({math.degrees(self.current_yaw_rate):+.1f}°/s) '
                    f'— 정렬 강행')
                self._enter_mode(self.TURN, now)

        elif self.mode == self.TURN:
            err = normalize_angle(self.target_yaw - self.current_yaw)
            if abs(err) < self.align_tol:
                if self.align_ok_since is None:
                    self.align_ok_since = now
                elif (now - self.align_ok_since) > self.align_settle_sec:
                    self._enter_mode(self.APPROACH, now)
            else:
                self.align_ok_since = None
            if (now - self.mode_enter_time) > self.scan_timeout_sec:
                self._abort_align(now, '회전 정렬 타임아웃')

        elif self.mode == self.APPROACH:
            if sonar_dead:
                # 소나 없이 벽으로 접근하는 것은 충돌 위험 — 중단.
                # 단 진입 직후는 유예한다: TURN 은 회전 중 먼 벽(>max_range)을
                # 지나며 lost 상태로 끝나기 쉬운데, 정렬을 마친 방위에는 가까운
                # 벽이 있으므로 곧 유효값이 돌아온다.
                self.approach_ok_since = None
                if (now - self.mode_enter_time) > self.approach_grace_sec:
                    self._abort_align(now, '접근 중 소나 상실')
            else:
                dist_err = self.sonar_dist - self.target_distance
                # 요청 3: 0.97~1.03m 를 5초 이상 연속 유지해야 HOLD 진입.
                # 한 번이라도 밖으로 나가면 타이머가 처음부터 다시 간다.
                if abs(dist_err) < self.approach_tol:
                    if self.approach_ok_since is None:
                        self.approach_ok_since = now
                    elif ((now - self.approach_ok_since)
                          >= self.approach_settle_sec):
                        self._enter_mode(self.HOLD, now)
                else:
                    self.approach_ok_since = None
                if (now - self.mode_enter_time) > self.approach_timeout_sec:
                    self._abort_align(now, '접근 타임아웃')

        elif self.mode == self.HOLD and not sonar_dead:
            # HOLD 도 거리 PID 를 계속 돌려 1m 를 능동 유지한다
            # (조류·드리프트로 밀려나도 스스로 복귀).
            dist_err = self.sonar_dist - self.target_distance

        # ── surge 명령 소유권 ────────────────────────────────────
        # APPROACH/HOLD 동안만 거리 PID 가 surge 를 덮어쓴다.
        # 그 외(IDLE 등)는 원본과 동일하게 ↑/↓ 수동 입력이 유지된다.
        if self.mode in (self.APPROACH, self.HOLD):
            if sonar_dead:
                # 소나 죽으면 즉시 0 — 충돌 위험축만 차단
                self.force[0] = 0.0
                self.wd.reset()
            else:
                self.force[0] = clamp(self.wd.compute(dist_err, dt), -1.0, 1.0)

        # ── DVL 기반 측방 드리프트 보상 (일자 주행) ────────────
        # surge 주명령 중 → body_sway_vel 을 0 으로 밀어내는 sway 보정
        # sway  주명령 중 → body_surge_vel 을 0 으로 밀어내는 surge 보정
        drift_fb_surge = 0.0
        drift_fb_sway = 0.0
        # 주명령 활성 판정 — DVL drift 보정과 yaw 라우팅이 공용.
        # 보정 주입 "전"의 명령 기준 (보정 thrust 가 교차축을 활성으로
        # 오판시키지 않도록 반드시 force 수정 전에 판정).
        surge_active = abs(self.force[0]) > self.dvl_min_cmd
        sway_active = abs(self.force[1]) > self.dvl_min_cmd
        dvl_fresh = (self.dvl_valid
                     and self.dvl_enabled
                     and (now - self.dvl_last_time)
                         < self.dvl_stale_timeout)
        # surge 축이 거리 PID 로 이미 폐루프인 APPROACH/HOLD 에서는
        # surge 보정을 넣지 않는다 — 두 제어기가 같은 축을 다투면 서로의
        # 적분기를 밀어낸다.
        surge_closed_loop = (self.mode in (self.APPROACH, self.HOLD)
                             and not sonar_dead)
        if dvl_fresh:
            # surge 중: sway 방향 드리프트를 잡음
            if surge_active and not sway_active:
                vy = self.dvl_body_vy
                if abs(vy) < self.dvl_vel_deadband:
                    vy = 0.0
                    self.drift_int_sway *= 0.95
                else:
                    self.drift_int_sway += vy * dt
                    self.drift_int_sway = clamp(self.drift_int_sway,
                                                -self.dvl_drift_max_int,
                                                self.dvl_drift_max_int)
                # 측방 양(+y, port)으로 밀리면 -y 쪽으로 sway thrust 주입
                drift_fb_sway = -(self.dvl_drift_kp * vy
                                  + self.dvl_drift_ki * self.drift_int_sway)
                drift_fb_sway = clamp(drift_fb_sway,
                                      -self.dvl_drift_max,
                                      self.dvl_drift_max)
                self.drift_int_surge *= 0.9
            # sway 중: surge 방향 드리프트를 잡음
            elif sway_active and not surge_active and not surge_closed_loop:
                vx = self.dvl_body_vx
                if abs(vx) < self.dvl_vel_deadband:
                    vx = 0.0
                    self.drift_int_surge *= 0.95
                else:
                    self.drift_int_surge += vx * dt
                    self.drift_int_surge = clamp(self.drift_int_surge,
                                                 -self.dvl_drift_max_int,
                                                 self.dvl_drift_max_int)
                drift_fb_surge = -(self.dvl_drift_kp * vx
                                   + self.dvl_drift_ki * self.drift_int_surge)
                drift_fb_surge = clamp(drift_fb_surge,
                                       -self.dvl_drift_max,
                                       self.dvl_drift_max)
                self.drift_int_sway *= 0.9
            else:
                # 입력 없거나 대각 이동 — 적분기 감쇠
                self.drift_int_surge *= 0.9
                self.drift_int_sway *= 0.9
        else:
            # DVL 유효하지 않을 때 적분기 리셋 (bias 누적 방지)
            self.drift_int_surge = 0.0
            self.drift_int_sway = 0.0

        # 주명령에 보정 더하되, 최종 force 를 [-1, 1] 로 제한
        self.force[0] = clamp(self.force[0] + drift_fb_surge, -1.0, 1.0)
        self.force[1] = clamp(self.force[1] + drift_fb_sway, -1.0, 1.0)

        # ── Yaw 상태 전이 ────────────────────────────────────────
        if is_yawing and not self.was_yawing:
            # 수동 회전 진입 — heading hold 적분 리셋 (누적 bias 제거)
            self.hh.reset()
        if (not is_yawing) and self.was_yawing:
            # 수동 회전 종료 — 현재 방향을 새로운 target 으로 lock
            self.target_yaw = self.current_yaw
            self.heading_lock_time = now
            self.hh.reset()
            self.yr.reset()
        self.was_yawing = is_yawing

        # 병진 키 뗌 → 그 순간 방위를 새 target 으로 lock.
        # 판정은 수동 키 유효시각 기준(force 아님) — DVL 보정·감쇠 잔량이
        # 섞이지 않은 "키를 뗀 시점" 그대로를 잡기 위함.
        # 정렬 시퀀스 중에는 발동 안 함 (target_yaw 는 벽 법선이라 고정).
        translating = any((now - self.axis_last_time[i]) < self.key_timeout
                          for i in range(2))
        if (self.was_translating and not translating
                and not is_yawing and self.yaw_initialized
                and self.mode == self.IDLE):
            self.target_yaw = self.current_yaw
            self.heading_lock_time = now
            self.hh.reset()
            self.get_logger().info(
                f'Heading re-locked on key release: '
                f'{math.degrees(self.current_yaw):.1f}° '
                f'({self.heading_hold_sec:.0f}초간 유지)')
        self.was_translating = translating

        # ── Yaw command 계산 ──────────────────────────────────────
        # 정렬 시퀀스(TURN·APPROACH)와 도달 후 자세유지(HOLD) 동안
        # target_yaw 를 고정한다. heading 이 벽 법선에서 θ 틀어지면
        # 측정거리가 d/cos(θ) 로 부풀어 거리 PID 가 벽으로 밀어붙인다.
        aligned = self.mode in (self.TURN, self.APPROACH, self.HOLD)
        holding = False        # 자세유지 창 활성 여부 (아래 heading hold 에서 갱신)

        if not self.yaw_initialized:
            yaw_command = 0.0
            yaw_err_deg = 0.0
        elif is_yawing:
            # Rate control — 저속 정속 회전 (수동 A/D)
            target_rate = self.yaw_key_sign * self.yr_target_rate
            yaw_command = self.yr.compute(
                target_rate, self.current_yaw_rate, dt)
            self.target_yaw = self.current_yaw
            yaw_err_deg = 0.0
        elif scan_yaw_cmd is not None:
            # SCAN — 제자리 저속 회전
            yaw_command = scan_yaw_cmd
            yaw_err_deg = 0.0
        elif self.mode == self.BRAKE:
            # 목표 각속도 0 — heading 은 신경쓰지 않는다. 여기서 heading 을
            # 잡으려 들면 아직 도는 중인 차체에 각도 제어가 걸려 관성을
            # 없애기는커녕 TURN 의 오버슈트를 그대로 재현한다.
            yaw_command = self.yr.compute(0.0, self.current_yaw_rate, dt)
            self.target_yaw = self.current_yaw
            yaw_err_deg = 0.0
        else:
            # Heading hold — 외란 억제 (항상 활성)
            yaw_err = normalize_angle(self.target_yaw - self.current_yaw)
            yaw_err_deg = math.degrees(yaw_err)

            # 모션 상태 판정
            is_moving = (abs(self.force[0]) > 0.05
                         or abs(self.force[1]) > 0.05)
            if is_moving:
                self.last_motion_time = now
            time_since_motion = now - self.last_motion_time
            is_idle = (not is_moving
                       and time_since_motion > self.idle_grace_sec)

            # 정렬 시퀀스(TURN·APPROACH)는 heading 을 빨리 수렴시켜야 하므로
            # 정지 감쇠를 적용하지 않는다. HOLD 는 도달 후 자세유지라
            # 원본 teleop 의 정지 상태와 똑같이 다룬다.
            settling = self.mode in (self.TURN, self.APPROACH)

            # 움직일 때 + 정렬 중에는 deadband 바이패스 (tight correction)
            original_deadband = self.hh.deadband
            if is_moving or settling:
                self.hh.deadband = 0.0
            fb = self.hh.compute(yaw_err, self.current_yaw_rate, dt)
            self.hh.deadband = original_deadband

            # 순수 피드백 — TAM 커플링은 rate PID 의 I 항이 흡수.
            yaw_command = clamp(fb, -1.0, 1.0)

            # 정지 중: 출력 감쇠 + 적분기 감쇠
            # (실측 확정: 정지 직후 heading hold 과출력이 진동 원인)
            # 키 뗌 직후 heading_hold_sec 동안은 잠근 방위를 그대로 지킨다
            # (자세유지 구간). 이 창 안에서는 아래 target 추종도, 출력 감쇠도
            # 걸지 않는다 — 둘 다 자세유지를 무력화한다.
            holding = ((now - self.heading_lock_time) < self.heading_hold_sec)

            if is_idle and not settling and not holding:
                yaw_command *= self.idle_output_scale
                self.hh.integral *= 0.95
                self.hh.angle_integral *= 0.95
                # 자세유지 창이 끝나면 target 을 현재 heading 으로 끌고 간다.
                # yaw 는 자이로 적분이라 절대 기준이 없어 측정값이 시간당
                # 흘러가고(분당 약 7°, 실측) 차체도 흔들린다. 과거 target 을
                # 계속 붙들면 없앨 수 없는 오차를 쫓아 제자리에서 혼자 돈다.
                # 단 HOLD 는 예외 — 그 heading 이 벽 법선이라 놓으면 정렬이
                # 풀리고 측정거리가 d/cos(θ) 로 부풀어 1m 유지가 깨진다.
                if not aligned:
                    self.target_yaw = self.current_yaw

        # 수동 yaw 키가 더 이상 유효하지 않으면 sign 초기화
        if not is_yawing:
            self.yaw_key_sign = 0.0

        # ── Heave command (depth hold + pitch FF) ────────────────
        depth_dead = self._depth_dead(now)
        if depth_dead and not self._warned_depth_dead:
            self.get_logger().warn(
                'Pressure sensor stale — depth hold OFF, '
                'W/S now directly commands heave')
            self._warned_depth_dead = True
        elif (not depth_dead) and self._warned_depth_dead:
            self.get_logger().info('Pressure sensor recovered — depth hold ON')
            self._warned_depth_dead = False
            self.manual_heave_cmd = 0.0

        if self.depth_initialized and not depth_dead:
            depth_err = self.target_depth - self.current_depth
            heave_command = self.depth_ctrl.compute(
                depth_err, self.current_depth, dt)
        else:
            depth_err = 0.0
            # manual fallback — 키 떼면 decay
            if now - self.manual_heave_last_key > self.key_timeout:
                self.manual_heave_cmd *= self.manual_heave_decay
                if abs(self.manual_heave_cmd) < 0.05:
                    self.manual_heave_cmd = 0.0
            heave_command = self.manual_heave_cmd

        horizontal_effort = abs(self.force[0]) + abs(self.force[1])
        if horizontal_effort > 0.01:
            # 수평 이동 중 pitch 에 의한 수직 드리프트 상쇄
            heave_command += (self.ff_pitch_to_heave
                              * math.sin(self.current_pitch)
                              * horizontal_effort)
        heave_command = clamp(heave_command, -1.0, 1.0)

        # ── TAM 믹싱 ─────────────────────────────────────────────
        # Yaw 보정 라우팅: surge 중엔 놀고 있는 T3/T4 차동으로만,
        # sway 중엔 T1/T2 차동으로만 yaw 보정 → 주추진 쌍은 차동 없이
        # 깨끗한 추력 유지. 대각 이동·정지·A/D 회전은 수평 4기 전체.
        if surge_active and not sway_active:
            yaw_thrusters = (2, 3)          # T3/T4
        elif sway_active and not surge_active:
            yaw_thrusters = (0, 1)          # T1/T2
        else:
            yaw_thrusters = (0, 1, 2, 3)
        cmd = [self.force[0], self.force[1], heave_command,
               0.0, 0.0, yaw_command]
        thrust = self.mix_thrusters(cmd, yaw_thrusters)

        # ── Ramping (급가속 방지) ────────────────────────────────
        target = {
            'surge_left':  thrust[0],
            'surge_right': thrust[1],
            'sway_left':   thrust[2],
            'sway_right':  thrust[3],
            'heave_up':    thrust[4],
            'heave_down':  thrust[5],
        }
        for name in self.vesc_ids:
            diff = target[name] - self.actual[name]
            if abs(diff) > self.ramp_step:
                self.actual[name] += (self.ramp_step if diff > 0
                                      else -self.ramp_step)
            else:
                self.actual[name] = target[name]

        # ── CAN 전송 ─────────────────────────────────────────────
        for name, cid in self.vesc_ids.items():
            self.send_current(cid, self.actual[name])

        # ── 디버그 publish ───────────────────────────────────────
        self._pub_array(self.force_pub, [self.force[0], self.force[1],
                                         heave_command, 0.0, 0.0, yaw_command])
        self._pub_array(self.thruster_pub, list(thrust))

        kmsg = String()
        kmsg.data = self.last_key
        self.key_pub.publish(kmsg)

        mmsg = String()
        mmsg.data = self.mode
        self.mode_pub.publish(mmsg)

        self._pub_array(self.debug_pub, [
            math.degrees(self.target_yaw) if self.yaw_initialized else 0.0,
            math.degrees(self.current_yaw),
            yaw_err_deg,
            math.degrees(self.current_yaw_rate),
            yaw_command,
            heave_command,
            math.degrees(self.current_pitch),
            1.0 if is_yawing else 0.0,
            self.dvl_body_vx if dvl_fresh else 0.0,
            self.dvl_body_vy if dvl_fresh else 0.0,
            drift_fb_surge,
            drift_fb_sway,
        ])

        self._pub_array(self.depth_debug_pub, [
            self.target_depth,
            self.current_depth,
            depth_err if self.depth_initialized else 0.0,
            heave_command,
            self.depth_ctrl.estimated_velocity,
            self.current_pressure,
        ])

        self._pub_array(self.wall_debug_pub, [
            self.target_distance,
            self.sonar_dist,
            dist_err,
            self.force[0],                     # surge command
            self.sonar_conf,
            0.0 if sonar_dead else 1.0,
            math.degrees(self.scan_accum_yaw),
            self.scan_best_dist if self.scan_best_dist is not None else -1.0,
            (math.degrees(self.scan_best_yaw)
             if self.scan_best_yaw is not None else 0.0),
            float(self.scan_sample_count),
            self.sonar_raw,
        ])

        # ── 주기적 로그 ──────────────────────────────────────────
        if self.mode == self.SCAN:
            self.get_logger().info(
                f'[SCAN] {math.degrees(self.scan_accum_yaw):+.0f}°/'
                f'{math.degrees(self.scan_sweep_rad):.0f}° '
                f'min:{self.scan_best_dist if self.scan_best_dist else -1:.2f}m '
                f'now:{self.sonar_raw:.2f}m conf:{self.sonar_conf:.0f}%',
                throttle_duration_sec=1.0)
        elif self.mode == self.BRAKE:
            self.get_logger().info(
                f'[BRAKE] rate:{math.degrees(self.current_yaw_rate):+.1f}°/s '
                f'(정지 판정 {math.degrees(self.brake_rate_tol):.1f}) '
                f'cmd:{yaw_command:+.3f}',
                throttle_duration_sec=0.5)
        elif self.mode == self.TURN:
            self.get_logger().info(
                f'[TURN] err:{yaw_err_deg:+.1f}° '
                f'tgt:{math.degrees(self.target_yaw):+.1f}° '
                f'cmd:{yaw_command:+.3f}',
                throttle_duration_sec=0.5)
        elif self.mode == self.APPROACH:
            held = (0.0 if self.approach_ok_since is None
                    else now - self.approach_ok_since)
            self.get_logger().info(
                f'[APPROACH] d:{self.sonar_dist:.3f}m err:{dist_err:+.3f}m '
                f'유지:{held:.1f}/{self.approach_settle_sec:.0f}s '
                f'surge:{self.force[0]:+.3f} yaw_err:{yaw_err_deg:+.1f}°',
                throttle_duration_sec=0.5)
        elif self.mode == self.HOLD:
            self.get_logger().info(
                f'[HOLD] d:{self.sonar_dist:.3f}m err:{dist_err:+.3f}m '
                f'surge:{self.force[0]:+.3f} yaw_err:{yaw_err_deg:+.1f}° '
                f'sway:{self.force[1]:+.2f}',
                throttle_duration_sec=0.5)
        elif (not is_yawing) and self.yaw_initialized and horizontal_effort > 0.1:
            self.get_logger().info(
                f'[HH] err:{yaw_err_deg:+.1f}° '
                f'rate:{math.degrees(self.current_yaw_rate):+.1f}°/s '
                f'cmd:{yaw_command:+.3f} '
                f'sway:{self.force[1]:+.2f} surge:{self.force[0]:+.2f}',
                throttle_duration_sec=0.3)
        elif holding and self.yaw_initialized:
            # 정지 자세유지 구간 — force 가 0 이라 위 [HH] 로그가 안 뜬다.
            # 자세유지가 실제로 걸리는지 확인할 유일한 계측 지점.
            remain = self.heading_hold_sec - (now - self.heading_lock_time)
            self.get_logger().info(
                f'[LOCK] err:{yaw_err_deg:+.1f}° '
                f'rate:{math.degrees(self.current_yaw_rate):+.1f}°/s '
                f'cmd:{yaw_command:+.3f} 남은:{remain:.0f}s',
                throttle_duration_sec=1.0)
        elif is_yawing:
            self.get_logger().info(
                f'[YR] tgt:'
                f'{math.degrees(self.yaw_key_sign * self.yr_target_rate):+.1f}°/s '
                f'act:{math.degrees(self.current_yaw_rate):+.1f}°/s '
                f'cmd:{yaw_command:+.3f}',
                throttle_duration_sec=0.3)

    # ─── CAN I/O ────────────────────────────────────────────────────
    def send_current(self, can_id, current):
        if not self.bus:
            return
        current = clamp(current, -5.0, 5.0)
        scaled = int(current * 1000)
        if scaled < 0:
            scaled &= 0xFFFFFFFF
        data = [(scaled >> 24) & 0xFF, (scaled >> 16) & 0xFF,
                (scaled >> 8) & 0xFF, scaled & 0xFF]
        try:
            self.bus.send(can.Message(arbitration_id=can_id, data=data,
                                      is_extended_id=True, dlc=4))
        except Exception:
            pass

    def stop_all(self):
        for name, cid in self.vesc_ids.items():
            self.send_current(cid, 0.0)
            self.actual[name] = 0.0

    def destroy_node(self):
        self.running = False
        self.stop_all()
        if self.bus:
            self.bus.shutdown()
        if self.settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopWallAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
