#!/usr/bin/env python3
"""
실시간 파라미터 튜닝 지원 — rqt_reconfigure 로 바꾼 값을 주행 중 즉시 반영한다.

문제: 노드들은 __init__ 에서 get_parameter() 값을 컨트롤러 객체 생성자에 복사한다.
      ros2 param set 을 해도 노드의 파라미터 값만 바뀌고 self.hh.rate_kp 같은
      객체 필드는 옛 값 그대로라 로봇이 반응하지 않는다.
해결: 파라미터 변경 콜백에서 해당 객체 필드까지 함께 갱신한다.

사용법 (노드 __init__ 끝부분에서 1줄):
    from pkrc_control.live_tuning import setup_live_tuning
    setup_live_tuning(self)

주행 중 바꿔도 안전한 튜닝 값만 대상으로 한다. dvl_topic 같은 구조적
파라미터는 제외 — 재시작이 필요하다.
"""

import atexit
import math
import os
import signal
import subprocess

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


# 주행 중 바꿀 일이 없는 값에 붙이는 서술자.
# read_only=True 면 rqt_reconfigure 목록에서 빠져 튜닝 화면이 깔끔해진다.
# (물성값·구독 토픽처럼 바꾸려면 재시작이 필요한 것들)
_FIXED = ParameterDescriptor(
    read_only=True,
    description='고정값 — 주행 중 변경 불가(변경하려면 소스 수정 후 재시작)')


# 파라미터명 → (대상 객체 속성, 그 객체의 필드명)
# 대상이 None 이면 노드 자신의 속성.
_ROUTES = {
    # ── Heading hold (self.hh) ─────────────────────────────────────────
    'hh_angle_kp':      ('hh', 'angle_kp'),
    'hh_angle_ki':      ('hh', 'angle_ki'),
    'hh_rate_kp':       ('hh', 'rate_kp'),
    'hh_rate_ki':       ('hh', 'rate_ki'),
    'hh_rate_kd':       ('hh', 'rate_kd'),
    'hh_rate_aw':       ('hh', 'rate_aw'),
    'hh_max_rate':      ('hh', 'max_rate'),
    'hh_output_slew':   ('hh', 'output_slew'),
    # deadband 는 deg → rad 변환이 필요해 아래에서 특수 처리
    'hh_deadband_deg':  ('hh', 'deadband'),

    # ── Yaw rate (self.yr) ─────────────────────────────────────────────
    'yr_kp':            ('yr', 'kp'),
    'yr_ki':            ('yr', 'ki'),
    'yr_kd':            ('yr', 'kd'),
    'yr_max_output':    ('yr', 'max_output'),

    # ── Depth (self.depth_ctrl) ────────────────────────────────────────
    'depth_pos_kp':     ('depth_ctrl', 'pos_kp'),
    'depth_vel_kp':     ('depth_ctrl', 'vel_kp'),
    'depth_vel_ki':     ('depth_ctrl', 'vel_ki'),
    'depth_vel_kd':     ('depth_ctrl', 'vel_kd'),
    'depth_max_ascent_vel': ('depth_ctrl', 'max_ascent_velocity'),

    # ── 노드 자신의 속성 ───────────────────────────────────────────────
    'yr_target_rate':       (None, 'yr_target_rate'),
    'idle_output_scale':    (None, 'idle_output_scale'),
    'idle_grace_sec':       (None, 'idle_grace_sec'),
    'release_output_scale': (None, 'release_output_scale'),
    'yaw_route_hold_sec':   (None, 'yaw_route_hold_sec'),
    'surge_cmd':            (None, 'surge_cmd'),
    'sway_cmd':             (None, 'sway_cmd'),
    'ff_pitch_to_heave':    (None, 'ff_pitch_to_heave'),
    'ff_sway_to_yaw':       (None, 'ff_sway_to_yaw'),
    'ff_surge_to_yaw':      (None, 'ff_surge_to_yaw'),
    'ff_sway_left_to_yaw':  (None, 'ff_sway_left_to_yaw'),
    'ff_sway_right_to_yaw': (None, 'ff_sway_right_to_yaw'),
    'ff_ramp_time_sec':     (None, 'ff_ramp_time_sec'),
    'depth_step':           (None, 'depth_step'),
    'dvl_drift_kp':         (None, 'dvl_drift_kp'),
    'dvl_drift_ki':         (None, 'dvl_drift_ki'),
    'dvl_drift_max':        (None, 'dvl_drift_max'),
    'dvl_drift_max_int':    (None, 'dvl_drift_max_int'),
    'dvl_vel_lpf_alpha':    (None, 'dvl_vel_lpf_alpha'),
    'dvl_vel_deadband':     (None, 'dvl_vel_deadband'),
    'dvl_stale_timeout':    (None, 'dvl_stale_timeout'),
    'dvl_min_cmd':          (None, 'dvl_min_cmd'),

    # ── 벽면추종 거리 제어 (self.wd) — wall_follow_control 전용 ─────────
    'wd_kp':                ('wd', 'kp'),
    'wd_ki':                ('wd', 'ki'),
    'wd_kd':                ('wd', 'kd'),
    'wd_max_output':        ('wd', 'max_output'),
    'wd_deadband':          ('wd', 'deadband'),
    'wd_output_slew':       ('wd', 'output_slew'),

    # ── 벽면추종 노드 속성 — wall_follow_control 전용 ──────────────────
    # scan_max_current 는 스캔 회전 전류 한계 [A]. 노드가 매 SCAN 루프에서
    # _scan_output_limit() 로 yr.max_output 에 환산 적용하므로 즉시 반영된다.
    'scan_max_current':     (None, 'scan_max_current'),
    'scan_yaw_rate':        (None, 'scan_yaw_rate'),
    'target_distance':      (None, 'target_distance'),
    'approach_tol':         (None, 'approach_tol'),
    'align_settle_sec':     (None, 'align_settle_sec'),
    'sonar_min_confidence': (None, 'sonar_min_confidence'),
    'sonar_lpf_alpha':      (None, 'sonar_lpf_alpha'),
    'sonar_jump_limit':     (None, 'sonar_jump_limit'),
    'sonar_stale_sec':      (None, 'sonar_stale_sec'),
    # gyro_bias_* 는 기동 시 1회만 쓰이므로 주행 중 조정 대상이 아니다
    # (선언은 돼 있어 ros2 param 으로 읽을 수는 있음).
}


def _apply(node, name, value):
    """파라미터 하나를 실제 객체 필드에 반영. 반영했으면 True."""
    route = _ROUTES.get(name)
    if route is None:
        return False

    owner_attr, field = route
    target = node if owner_attr is None else getattr(node, owner_attr, None)
    if target is None:
        return False
    # 노드가 선언만 하고 쓰지 않는 파라미터는 건너뛴다 (파일마다 구성이 다름)
    if not hasattr(target, field):
        return False

    if name == 'hh_deadband_deg':
        value = math.radians(value)
    setattr(target, field, value)
    return True


def setup_live_tuning(node, launch_gui=None):
    """파라미터 콜백을 등록하고, 요청 시 rqt_reconfigure 를 함께 띄운다.

    launch_gui 가 None 이면 'tuning_gui' 파라미터 값을 따른다.
    """
    def _on_set(params):
        applied = []
        for p in params:
            if _apply(node, p.name, p.value):
                applied.append(f'{p.name}={p.value}')
        if applied:
            node.get_logger().info('실시간 반영: ' + ', '.join(applied))
        # 반영 대상이 아닌 파라미터도 거절하지 않는다 — 노드가 매 루프
        # get_parameter() 로 읽는 값일 수 있고, 거절하면 그쪽이 막힌다.
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(_on_set)

    if launch_gui is None:
        try:
            launch_gui = node.get_parameter('tuning_gui').value
        except Exception:
            launch_gui = False
    if launch_gui:
        _launch_rqt(node)


def _descendants(pid):
    """pid 의 모든 하위 프로세스를 /proc 로 수집한다(psutil 의존성 없이)."""
    children = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/stat') as f:
                # comm 에 공백/괄호가 들어갈 수 있어 마지막 ')' 이후를 파싱
                fields = f.read().rpartition(')')[2].split()
            children.setdefault(int(fields[1]), []).append(int(entry))
        except (OSError, ValueError, IndexError):
            continue

    out, stack = [], [pid]
    while stack:
        for child in children.get(stack.pop(), []):
            out.append(child)
            stack.append(child)
    return out


def _terminate(proc):
    """노드 종료 시 GUI 도 함께 닫는다.

    `ros2 run` 은 래퍼이고, 그것이 띄우는 실제 rqt 프로세스는 자체
    프로세스 그룹을 새로 만들어 빠져나간다 — killpg 로는 닿지 않는다.
    그래서 래퍼의 자손을 직접 훑어 함께 종료한다.
    """
    if proc.poll() is not None:      # 이미 종료됨(사용자가 창을 먼저 닫음)
        return

    targets = _descendants(proc.pid) + [proc.pid]
    for sig in (signal.SIGTERM, signal.SIGKILL):
        alive = []
        for pid in targets:
            try:
                os.kill(pid, sig)
                alive.append(pid)
            except (ProcessLookupError, PermissionError):
                pass
        if not alive:
            return
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
        targets = [p for p in alive if os.path.exists(f'/proc/{p}')]
        if not targets:
            return


def _launch_rqt(node):
    """rqt_reconfigure 를 백그라운드로 띄운다. 실패해도 노드는 계속 동작.

    rqt 플러그인이라 PATH 에 독립 실행파일이 없다 — shutil.which() 로는
    설치돼 있어도 못 찾는다. 반드시 `ros2 run` 으로 띄운다.
    """
    if not os.environ.get('DISPLAY'):
        node.get_logger().warn(
            'DISPLAY 없음 — GUI 생략. 원격이면 ssh -X 로 접속하거나 '
            '다른 PC에서 rqt_reconfigure 를 직접 실행하세요.')
        return
    try:
        # GUI 를 자체 프로세스 그룹으로 띄운다(setsid). 종료는 atexit 에서
        # 그룹 전체에 시그널을 보내 처리한다 — `ros2 run` 래퍼와 그 자식인
        # 실제 rqt 프로세스를 한 번에 정리하기 위함.
        proc = subprocess.Popen(
            ['ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        atexit.register(_terminate, proc)
        node.get_logger().info('rqt_reconfigure 실행 — 파라미터를 실시간 조절하세요.')
    except FileNotFoundError:
        node.get_logger().warn(
            'ros2 명령을 찾을 수 없음 — GUI 생략. '
            'source /opt/ros/humble/setup.bash 후 다시 실행하세요.')
    except Exception as e:
        node.get_logger().warn(
            f'rqt_reconfigure 실행 실패: {e} — 미설치라면 '
            'sudo apt install ros-humble-rqt-reconfigure')
