#!/usr/bin/env python3
"""
PKRC Robust Teleop — IMU 기반 외란 강건 제어기
================================================================================
설계 목표
  1. Surge/Sway 중 yaw가 흔들리지 않고 일자로 주행 (외란 억제)
  2. A/D 입력 시 저출력으로 일정하게 스윽 돌기 (yaw rate 제어)
  3. W/S 입력 시 heave 직접 제어 (pitch feedforward 로 수평 이동 중 고도 유지)

키 매핑
  ↑/↓   Surge (전/후진) — heading hold 동작
  ←/→   Sway  (좌/우 이동) — heading hold + sway→yaw FF 로 직진 유지
  A/D   Yaw Rate (좌/우 회전) — 목표 0.3 rad/s, 출력 한계 0.5
  W/S   Heave (상/하) — pitch feedforward 포함
  R     Heading 리셋    X  정지    Q  종료

제어 구조
  Heading Hold (surge/sway 중, 정지 중):
    캐스케이드 PID — 각도 P  →  목표 각속도  →  rate PID  →  yaw cmd
    + TAM 커플링 사전보상 FF (sway→yaw 0.6, surge→yaw 0.05)
    + Anti-windup (conditional integration + 부호 반전 감쇠)

  Yaw Rate (A/D):
    순수 rate PID — 목표 0.3 rad/s, 출력 0.5 로 제한 → 저속 정속 회전
    외란이 들어와도 I 항이 즉시 보상하여 회전 속도 유지

  Heave (W/S):
    직접 추력 명령 + pitch feedforward
    (수평 이동 시 기체가 기울어 생기는 수직 드리프트 상쇄)

IMU 활용
  orientation.*        → yaw, pitch, roll (quaternion → euler)
  angular_velocity.z   → yaw rate (rate PID 피드백, 외란 감지)

TAM 독립 정규화
  surge(T1,T2) / sway(T3,T4) / heave(T5,T6) 각 그룹을 독립 정규화
  → yaw 보정 채널(T1/T2 차동)이 sway 포화에 잡아먹히지 않음
"""

import math
import select
import sys
import termios
import tty

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray, String
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

    외란 대응
      - rate_ki : 정상 외란(조류/부력 모멘트) 보상
      - rate_kp : 과도 외란(순간 충격) 반응
      - rate_kd : 진동 감쇠
      - conditional integration + 부호 반전 감쇠 로 와인드업 제어
      - deadband + 출력 LPF 로 정지점 채터링 방지
    """

    def __init__(self,
                 angle_kp=3.5,
                 rate_kp=3.0,
                 rate_ki=1.8,
                 rate_kd=0.65,
                 max_rate=1.2,
                 max_integral=2.0,
                 deadband_deg=0.5,
                 smoothing=0.5,
                 output_slew=4.0):
        self.angle_kp = angle_kp
        self.rate_kp = rate_kp
        self.rate_ki = rate_ki
        self.rate_kd = rate_kd
        self.max_rate = max_rate
        self.max_integral = max_integral
        self.deadband = math.radians(deadband_deg)
        self.smoothing = smoothing
        self.output_slew = output_slew  # 출력 변화율 제한 (per sec)
        self.max_output = 1.0

        self.integral = 0.0
        self.prev_rate_err = 0.0
        self.prev_output = 0.0

    def compute(self, yaw_error, yaw_rate, dt):
        if dt <= 0.0:
            return self.prev_output

        err = normalize_angle(yaw_error)

        # 정지점 근처에서는 조용히 감쇠 (채터링 방지)
        if abs(err) < self.deadband:
            self.prev_output *= 0.9
            return self.prev_output

        # 외부 P : 각도 → 목표 각속도 (deadband 지점에서 선형 시작)
        err_adj = err - math.copysign(self.deadband, err)
        target_rate = clamp(self.angle_kp * err_adj,
                            -self.max_rate, self.max_rate)

        # 내부 PID : 각속도 추종
        rate_err = target_rate - yaw_rate

        p = self.rate_kp * rate_err

        # Anti-windup : 출력 포화 시 적분 정지
        if abs(self.prev_output) < self.max_output * 0.9:
            self.integral += rate_err * dt
        # 부호 반전 감쇠 (오버슈트 방지)
        if rate_err * self.integral < 0.0:
            self.integral *= 0.9
        self.integral = clamp(self.integral,
                              -self.max_integral, self.max_integral)
        i = self.rate_ki * self.integral

        # D (미분 킥 방지용 rate error 기반, 제한 적용)
        d_raw = clamp((rate_err - self.prev_rate_err) / dt, -5.0, 5.0)
        d = self.rate_kd * d_raw
        self.prev_rate_err = rate_err

        raw = clamp(p + i + d, -self.max_output, self.max_output)
        out = self.smoothing * raw + (1.0 - self.smoothing) * self.prev_output

        # 출력 slew rate 제한 (급격한 방향 전환으로 인한 wobble 억제)
        max_step = self.output_slew * dt
        out = clamp(out,
                    self.prev_output - max_step,
                    self.prev_output + max_step)

        self.prev_output = out
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_rate_err = 0.0
        self.prev_output = 0.0


# ─── Yaw Rate Controller (A/D 수동 회전) ───────────────────────────────
class YawRateController:
    """
    목표 각속도에 정속 추종하는 순수 rate PID.

    설계 포인트
      - target_rate 를 0.3 rad/s 로 낮게 설정 → 물리적으로 "스윽" 회전
      - max_output 을 0.5 로 제한     → 추력 낮게 유지 ("출력은 낮지만")
      - ki 로 외란 보상         → 일정 속도 유지 ("일정하게")
    """

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
    """
    외부 P : depth_error         → target_vel  (max_velocity 로 포화)
    내부 PID: (target_vel - est_vel) → heave cmd ∈ [-1, +1]

    속도는 depth 미분 + 로우패스 필터로 추정.
    heave+ = 하강, heave- = 상승 (TAM 기준)
    """

    def __init__(self,
                 pos_kp=2.0,
                 vel_kp=1.2,
                 vel_ki=0.3,
                 vel_kd=0.15,
                 max_velocity=0.4,
                 max_integral=0.5,
                 smoothing=0.4,
                 vel_filter_alpha=0.4):   # 속도가 센서율(10Hz)에서 이미 계산되므로 필터 적당히
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
            # Outlier clip (갑작스런 대형 spike 제거)
            raw_vel = clamp(raw_vel, -2.0, 2.0)
            self.estimated_velocity = (
                self.vel_filter_alpha * raw_vel
                + (1.0 - self.vel_filter_alpha) * self.estimated_velocity)
        self.prev_depth = current_depth

    def compute(self, depth_error, current_depth, dt):
        if dt <= 0.0:
            return self.prev_output

        # 속도는 update_velocity() 에서 센서 rate 로 이미 갱신됨
        # (control loop 의 50Hz dt 로 미분하면 4/5는 0, 1/5는 spike → 무의미)

        # 외부 P
        target_vel = clamp(self.pos_kp * depth_error,
                           -self.max_velocity, self.max_velocity)

        # 내부 PID
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


# ─── Main Node ─────────────────────────────────────────────────────────
class KeyboardTeleopRobust(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_robust')

        # ── Parameters ─────────────────────────────────────────────────
        # Heading hold (정지·surge·sway 중 yaw 유지)
        self.declare_parameter('hh_angle_kp', 3.5)
        self.declare_parameter('hh_rate_kp', 2.8)   # 14:54: 3.0 → 2.8 (sway sat 완화)
        self.declare_parameter('hh_rate_ki', 1.8)
        self.declare_parameter('hh_rate_kd', 0.85)   # 0.45 → 0.65 (sway 진동 감쇠)
        self.declare_parameter('hh_max_rate', 0.8)
        self.declare_parameter('hh_deadband_deg', 0.5)
        self.declare_parameter('hh_output_slew', 4.0)   # yaw_cmd /sec 최대 변화율
        # 정지 시 출력 감쇠 (deadband 없이 — 항상 보정하되 약하게)
        self.declare_parameter('idle_output_scale', 0.30)   # 정지 1.5초 후 출력 30%
        self.declare_parameter('idle_grace_sec', 1.5)       # 모션 후 이 시간까지 full output

        # Yaw rate (A/D)
        self.declare_parameter('yr_target_rate', 0.30)   # rad/s ≈ 17°/s
        self.declare_parameter('yr_kp', 2.0)
        self.declare_parameter('yr_ki', 2.5)
        self.declare_parameter('yr_kd', 0.10)
        self.declare_parameter('yr_max_output', 0.50)    # 출력 한계

        # Feedforward — TAM 커플링 사전 보상
        self.declare_parameter('ff_sway_to_yaw', 0.60)   # 메모리 검증치
        self.declare_parameter('ff_surge_to_yaw', 0.05)  # 14:03 log: 0.07 살짝 과보정 → 0.05
        # LEFT sway 가 RIGHT 보다 yaw 커플링 강함 → 비대칭 FF
        # 14:08 log: 0.75/0.55 는 편향 제거했으나 oscillation → 0.60/0.45 로 감소
        self.declare_parameter('ff_sway_left_to_yaw', 0.55)   # 15:17: 0.70 포화로 T4 부호반전(yaw 회전) → 0.55
        self.declare_parameter('ff_sway_right_to_yaw', 0.40)   # 14:47: 0.50 과보정(sat 18%) → 0.40
        # Sway FF ramp-up (LEFT 키 누른 순간 yaw 획도는 현상 완화)
        # 물리적 커플링 지연 시간과 맞추기 위해 FF 를 LPF 로 서서히 증가
        self.declare_parameter('ff_ramp_time_sec', 0.40)
        self.declare_parameter('ff_pitch_to_heave', 2.5)

        # Depth hold (W/S → target_depth ∓depth_step)
        self.declare_parameter('depth_pos_kp', 2.0)
        self.declare_parameter('depth_vel_kp', 1.2)
        self.declare_parameter('depth_vel_ki', 0.3)
        self.declare_parameter('depth_vel_kd', 0.15)
        self.declare_parameter('depth_step', 0.10)              # 키 1회당 10 cm
        self.declare_parameter('water_density', 1000.0, _FIXED)   # kg/m³ (물성값)
        self.declare_parameter('gravity', 9.81, _FIXED)           # m/s² (물성값)
        self.declare_parameter('atmospheric_pressure_pa', 101325.0, _FIXED)  # 표준 대기압

        # 입력 크기
        self.declare_parameter('surge_cmd', 0.80)
        self.declare_parameter('sway_cmd', 0.80)

        # DVL 기반 측방 드리프트 보상 (일자 주행)
        # surge 중 body_sway_vel → 0, sway 중 body_surge_vel → 0 이 되도록
        # 교차 축에 보정 thrust 주입. Heading hold 는 yaw 만 잡으므로 해류에
        # 밀려 궤적이 휘는 것을 막지 못함 → DVL 로 body velocity 직접 폐루프.
        self.declare_parameter('dvl_topic', '/dvl/data', _FIXED)  # 구독 토픽(재시작 필요)
        self.declare_parameter('dvl_enabled', True)
        self.declare_parameter('dvl_drift_kp', 0.8)       # 속도 오차 → 보정 thrust
        self.declare_parameter('dvl_drift_ki', 0.6)
        self.declare_parameter('dvl_drift_max', 0.35)     # 보정 한계 (주명령 방해 X)
        self.declare_parameter('dvl_drift_max_int', 0.5)
        self.declare_parameter('dvl_vel_lpf_alpha', 0.35) # DVL 노이즈 필터
        self.declare_parameter('dvl_vel_deadband', 0.03)  # 3 cm/s 미만은 무시
        self.declare_parameter('dvl_stale_timeout', 0.5)  # 이만큼 신호 없으면 비활성
        self.declare_parameter('dvl_min_cmd', 0.10)       # 주명령 작으면 보정 안함

        # 전류 한계 (A)
        self.declare_parameter('max_current_surge', 3.0)
        self.declare_parameter('max_current_sway', 3.0)
        self.declare_parameter('max_current_heave', 3.5)

        # 파라미터 로드
        self.hh = HeadingHoldController(
            angle_kp=self.get_parameter('hh_angle_kp').value,
            rate_kp=self.get_parameter('hh_rate_kp').value,
            rate_ki=self.get_parameter('hh_rate_ki').value,
            rate_kd=self.get_parameter('hh_rate_kd').value,
            max_rate=self.get_parameter('hh_max_rate').value,
            deadband_deg=self.get_parameter('hh_deadband_deg').value,
            output_slew=self.get_parameter('hh_output_slew').value)
        self.yr = YawRateController(
            kp=self.get_parameter('yr_kp').value,
            ki=self.get_parameter('yr_ki').value,
            kd=self.get_parameter('yr_kd').value,
            max_output=self.get_parameter('yr_max_output').value)

        self.yr_target_rate = self.get_parameter('yr_target_rate').value
        self.ff_sway_to_yaw = self.get_parameter('ff_sway_to_yaw').value
        self.ff_sway_left_to_yaw = self.get_parameter('ff_sway_left_to_yaw').value
        self.ff_sway_right_to_yaw = self.get_parameter('ff_sway_right_to_yaw').value
        self.ff_surge_to_yaw = self.get_parameter('ff_surge_to_yaw').value
        self.ff_pitch_to_heave = self.get_parameter('ff_pitch_to_heave').value
        self.ff_ramp_time_sec = self.get_parameter('ff_ramp_time_sec').value
        self.idle_output_scale = self.get_parameter('idle_output_scale').value
        self.idle_grace_sec = self.get_parameter('idle_grace_sec').value
        self.surge_cmd = self.get_parameter('surge_cmd').value
        self.sway_cmd = self.get_parameter('sway_cmd').value

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
        self.min_current_horizontal = 0.3   # VESC 데드존 보상

        # ── 실시간 파라미터 튜닝 ────────────────────────────────────────
        # rqt_reconfigure 로 바꾼 값을 주행 중 즉시 반영한다.
        # tuning_gui:=false 로 GUI 자동 실행만 끌 수 있다(콜백은 항상 동작).
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

        # ── Input / state ──────────────────────────────────────────────
        # [surge, sway, heave] manual 입력
        self.force = [0.0, 0.0, 0.0]
        # 축별 키 마지막 시각 (축별 독립 decay)
        self.axis_last_time = [0.0, 0.0, 0.0]
        # 수동 yaw (A/D) 상태
        self.yaw_key_sign = 0.0
        self.yaw_last_time = 0.0
        self.was_yawing = False

        self.key_timeout = 0.40       # 키 release timeout (auto-repeat 간격보다 커야 함)
        self.decay_rate = 0.85
        self.last_key = ''

        # IMU state
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_yaw_rate = 0.0
        self.target_yaw = 0.0
        self.yaw_initialized = False
        self.last_control_time = None
        self.last_motion_time = 0.0   # 마지막 유효 모션 시각 (idle 스케일링용)
        self.sway_ff_filtered = 0.0   # sway→yaw FF 의 LPF 상태 (ramp-up)

        # Depth state
        self.current_pressure = 0.0
        self.current_depth = 0.0
        self.filtered_depth = 0.0
        self.target_depth = 0.0
        self.depth_initialized = False
        self.depth_filter_alpha = 0.2
        self.last_pressure_time = None   # sensor rate velocity 추정용
        # 센서 사망 감지 → manual heave fallback (W/S 직접 heave 명령)
        self.pressure_last_rx = 0.0        # 메시지 도착 시각
        self.pressure_stale_sec = 2.0      # 이만큼 신호 없으면 dead
        self.manual_heave_cmd = 0.0        # fallback 모드 W/S thrust
        self.manual_heave_step = 0.25      # 키 1회당 heave 증가량
        self.manual_heave_decay = 0.85     # 키 떼면 decay
        self.manual_heave_last_key = 0.0
        self._warned_depth_dead = False

        # DVL body-frame velocity state
        # R_dvl_to_body = [[0,0,1],[-1,0,0],[0,1,0]]  (ukfm_localization 참조)
        #   body_vx(surge) = dvl_vz, body_vy(sway) = -dvl_vx, body_vz(heave)=dvl_vy
        self.dvl_body_vx = 0.0     # surge velocity, forward +
        self.dvl_body_vy = 0.0     # sway  velocity, port +
        self.dvl_last_time = 0.0
        self.dvl_valid = False
        # 교차축 드리프트 보상 적분기
        self.drift_int_surge = 0.0   # sway 중 surge 방향 드리프트 적분
        self.drift_int_sway = 0.0    # surge 중 sway  방향 드리프트 적분

        # 스러스터 ramping
        self.actual = {n: 0.0 for n in self.vesc_ids}
        self.ramp_step = 0.5

        # ── ROS I/O ────────────────────────────────────────────────────
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Float64, '/pressure',
                                  self.pressure_callback, 10)
        if self.dvl_enabled:
            dvl_topic = self.get_parameter('dvl_topic').value
            # DVL driver 는 BEST_EFFORT 로 퍼블리시 → QoS 맞춰야 수신됨
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
║  PKRC Robust Teleop - IMU 외란 강건 제어              ║
╠═══════════════════════════════════════════════════════╣
║   ↑/↓  : Surge (전/후)   -- heading lock              ║
║   ←/→  : Sway  (좌/우)   -- heading lock + FF         ║
║   A/D  : Yaw Rate (우/좌 회전) 0.3 rad/s 스윽 회전    ║
║   W/S  : Depth ∓10cm (depth hold)                     ║
║         센서 죽으면 직접 heave thrust로 fallback      ║
║   R    : Heading 리셋 (현재 방향으로)                 ║
║   T    : Depth 리셋 (현재 깊이로)                     ║
║   X    : 정지 (모든 축 + 제어기 리셋)                 ║
║   Q    : 종료                                         ║
╚═══════════════════════════════════════════════════════╝
""")

    # ─── IMU ────────────────────────────────────────────────────────
    def imu_callback(self, msg: Imu):
        o = msg.orientation

        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.current_yaw = math.atan2(siny, cosy)

        sinp = clamp(2.0 * (o.w * o.y - o.z * o.x), -1.0, 1.0)
        self.current_pitch = math.asin(sinp)

        sinr = 2.0 * (o.w * o.x + o.y * o.z)
        cosr = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
        self.current_roll = math.atan2(sinr, cosr)

        self.current_yaw_rate = msg.angular_velocity.z

        if not self.yaw_initialized:
            self.target_yaw = self.current_yaw
            self.hh.reset()
            self.yaw_initialized = True
            self.get_logger().info(
                f'Heading initialized: {math.degrees(self.current_yaw):.1f}°')

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

    def pressure_callback(self, msg: Float64):
        # /pressure 는 절대압 (mbar). 수심 = (절대압 - 대기압) / (ρ·g).
        self.current_pressure = msg.data * 100.0
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

    # ─── Keyboard ───────────────────────────────────────────────────
    def get_key(self):
        if self.settings is None:
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

    # ─── TAM Mixing ────────────────────────────────────────────────
    def mix_thrusters(self, cmd):
        """cmd: [surge, sway, heave, roll, pitch, yaw] ∈ [-1, 1]"""
        raw = [0.0] * 6
        for i in range(6):
            for j in range(6):
                raw[i] += self.tam[i][j] * cmd[j]

        # 독립 정규화 — yaw 보정 채널을 sway 포화에서 보호
        sm = max(abs(raw[0]), abs(raw[1]))
        if sm > 1.0:
            raw[0] /= sm
            raw[1] /= sm
        wm = max(abs(raw[2]), abs(raw[3]))
        if wm > 1.0:
            raw[2] /= wm
            raw[3] /= wm
        hm = max(abs(raw[4]), abs(raw[5]))
        if hm > 1.0:
            raw[4] /= hm
            raw[5] /= hm

        current = [raw[i] * self.max_current[i] for i in range(6)]

        # VESC 데드존 보상 (수평 4기)
        for i in range(4):
            if abs(current[i]) < 0.10:
                current[i] = 0.0
            elif abs(current[i]) < self.min_current_horizontal:
                current[i] = math.copysign(
                    self.min_current_horizontal, current[i])
        return current

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

            if key == 'UP':
                self.force[0] = +self.surge_cmd
                self.axis_last_time[0] = now
            elif key == 'DOWN':
                self.force[0] = -self.surge_cmd
                self.axis_last_time[0] = now
            elif key == 'LEFT':
                # 메모리 기준 : LEFT → sway + 방향
                self.force[1] = +self.sway_cmd
                self.axis_last_time[1] = now
            elif key == 'RIGHT':
                self.force[1] = -self.sway_cmd
                self.axis_last_time[1] = now
            elif k == 'w':
                depth_dead = (self.pressure_last_rx == 0.0
                              or (now - self.pressure_last_rx)
                                 > self.pressure_stale_sec)
                if depth_dead:
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
                depth_dead = (self.pressure_last_rx == 0.0
                              or (now - self.pressure_last_rx)
                                 > self.pressure_stale_sec)
                if depth_dead:
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
                self.hh.reset()
                self.get_logger().info(
                    f'Heading reset: {math.degrees(self.current_yaw):.1f}°')
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
                self.depth_ctrl.reset()
                self.drift_int_surge = 0.0
                self.drift_int_sway = 0.0
                self.manual_heave_cmd = 0.0
                if self.yaw_initialized:
                    self.target_yaw = self.current_yaw
                if self.depth_initialized:
                    self.target_depth = self.current_depth
                self.get_logger().info('STOP (all axes + controllers reset)')
            elif k == 'q':
                self.running = False
                return

        # ── 축별 독립 decay (키 떼면 자연스럽게 0으로) ────────────
        # surge / sway 만 decay. heave 는 depth hold 가 상시 관리.
        for i in range(2):
            if now - self.axis_last_time[i] > self.key_timeout:
                self.force[i] *= self.decay_rate
                if abs(self.force[i]) < 0.05:
                    self.force[i] = 0.0

        # dt 계산 (DVL 적분기 / heading hold 공용)
        dt = 0.02 if self.last_control_time is None \
            else now - self.last_control_time

        # ── DVL 기반 측방 드리프트 보상 (일자 주행) ────────────
        # surge 주명령 중 → body_sway_vel 을 0 으로 밀어내는 sway 보정
        # sway  주명령 중 → body_surge_vel 을 0 으로 밀어내는 surge 보정
        # heading hold / sway→yaw FF 보다 "앞" 단계에서 force 를 수정해야
        # TAM 커플링 사전보상과 yaw FF 가 보정된 sway 명령에도 자연스럽게
        # 적용됨.
        drift_fb_surge = 0.0
        drift_fb_sway = 0.0
        dvl_fresh = (self.dvl_valid
                     and self.dvl_enabled
                     and (now - self.dvl_last_time)
                         < self.dvl_stale_timeout)
        if dvl_fresh:
            surge_active = abs(self.force[0]) > self.dvl_min_cmd
            sway_active = abs(self.force[1]) > self.dvl_min_cmd

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
            elif sway_active and not surge_active:
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

        # ── Yaw 상태머신 : rate control vs heading hold ──────────
        is_yawing = ((now - self.yaw_last_time) < self.key_timeout
                     and abs(self.yaw_key_sign) > 0.5)

        self.last_control_time = now

        # 상태 전이 처리
        if is_yawing and not self.was_yawing:
            # 수동 회전 진입 — heading hold 적분 리셋 (누적 bias 제거)
            self.hh.reset()
        if (not is_yawing) and self.was_yawing:
            # 수동 회전 종료 — 현재 방향을 새로운 target 으로 lock
            self.target_yaw = self.current_yaw
            self.hh.reset()
            self.yr.reset()
        self.was_yawing = is_yawing

        # ── Yaw command 계산 ──────────────────────────────────────
        if not self.yaw_initialized:
            yaw_command = 0.0
            yaw_err_deg = 0.0
        elif is_yawing:
            # Rate control — 저속 정속 회전
            target_rate = self.yaw_key_sign * self.yr_target_rate
            yaw_command = self.yr.compute(
                target_rate, self.current_yaw_rate, dt)
            # 회전 중에는 target_yaw 를 계속 따라가게 함
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

            # 움직일 때는 deadband 바이패스 (tight correction)
            original_deadband = self.hh.deadband
            if is_moving:
                self.hh.deadband = 0.0
            fb = self.hh.compute(yaw_err, self.current_yaw_rate, dt)
            self.hh.deadband = original_deadband

            # TAM 커플링 사전보상 (LEFT/RIGHT 비대칭)
            if self.force[1] > 0.0:
                sway_ff_gain = self.ff_sway_left_to_yaw
            else:
                sway_ff_gain = self.ff_sway_right_to_yaw
            target_sway_ff = sway_ff_gain * self.force[1]

            # FF LPF ramp-up — 물리적 커플링 지연과 타이밍 맞춤
            # (sway 시작 순간 FF 즉시 만땅 → T3/T4 비대칭 오버슈트 방지)
            if self.ff_ramp_time_sec > 0.0:
                alpha = clamp(dt / self.ff_ramp_time_sec, 0.0, 1.0)
            else:
                alpha = 1.0
            self.sway_ff_filtered += alpha * (
                target_sway_ff - self.sway_ff_filtered)

            ff = (self.sway_ff_filtered
                  + self.ff_surge_to_yaw * self.force[0])
            yaw_command = clamp(fb + ff, -1.0, 1.0)

            # 정지 중 (grace 이후): 출력 감쇠 + 적분기 감쇠
            # (deadband 없음 — 모든 err 에 대해 보정하되 부드럽게)
            if is_idle:
                yaw_command *= self.idle_output_scale
                self.hh.integral *= 0.95

        # 수동 yaw 키가 더 이상 유효하지 않으면 sign 초기화
        if not is_yawing:
            self.yaw_key_sign = 0.0

        # ── Heave command (depth hold + pitch FF) ────────────────
        depth_dead = (self.pressure_last_rx == 0.0
                      or (now - self.pressure_last_rx)
                         > self.pressure_stale_sec)
        if depth_dead and not self._warned_depth_dead:
            self.get_logger().warn(
                'Pressure sensor stale — depth hold OFF, '
                'W/S now directly commands heave')
            self._warned_depth_dead = True
        elif (not depth_dead) and self._warned_depth_dead:
            self.get_logger().info(
                'Pressure sensor recovered — depth hold ON')
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
        cmd = [self.force[0], self.force[1], heave_command,
               0.0, 0.0, yaw_command]
        thrust = self.mix_thrusters(cmd)

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
        fmsg = Float64MultiArray()
        fmsg.data = [self.force[0], self.force[1], heave_command,
                     0.0, 0.0, yaw_command]
        self.force_pub.publish(fmsg)

        tmsg = Float64MultiArray()
        tmsg.data = list(thrust)
        self.thruster_pub.publish(tmsg)

        kmsg = String()
        kmsg.data = self.last_key
        self.key_pub.publish(kmsg)

        dmsg = Float64MultiArray()
        dmsg.data = [
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
        ]
        self.debug_pub.publish(dmsg)

        ddbg = Float64MultiArray()
        ddbg.data = [
            self.target_depth,
            self.current_depth,
            depth_err if self.depth_initialized else 0.0,
            heave_command,
            self.depth_ctrl.estimated_velocity,
            self.current_pressure,
        ]
        self.depth_debug_pub.publish(ddbg)

        # ── 주기적 로그 (heading hold 작동 상황) ─────────────────
        if (not is_yawing) and self.yaw_initialized and horizontal_effort > 0.1:
            self.get_logger().info(
                f'[HH] err:{yaw_err_deg:+.1f}° '
                f'rate:{math.degrees(self.current_yaw_rate):+.1f}°/s '
                f'cmd:{yaw_command:+.3f} '
                f'sway:{self.force[1]:+.2f} surge:{self.force[0]:+.2f}',
                throttle_duration_sec=0.3)
        if is_yawing:
            self.get_logger().info(
                f'[YR] tgt:{math.degrees(self.yaw_key_sign * self.yr_target_rate):+.1f}°/s '
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
    node = KeyboardTeleopRobust()
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
