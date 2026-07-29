#!/usr/bin/env python3
"""PKRC 통합 관제 웹 GUI 서버.

이 노드 하나가 ROS 노드이면서 웹서버다 — 별도 브리지 프로세스가 없다.

왜 웹인가: 이 Jetson 은 DISPLAY 가 없어(headless) rqt·PyQt 를 띄울 수 없다.
왜 stdlib 인가: flask·fastapi 가 설치돼 있지 않고, 라우트가 7개뿐이라
프레임워크를 얹을 이유가 없다.

    ros2 launch pkrc_control gui.launch.py
    → 브라우저에서 http://<젯슨IP>:8080
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import yaml
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import (DescribeParameters, GetParameters,
                               ListParameters, SetParameters)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float64MultiArray

# 프로세스 종료는 이 패키지가 이미 풀어놓은 문제다 — ros2 launch 는 래퍼이고
# 실제 자식이 별도 프로세스 그룹으로 빠져나가므로 단순 kill 이 닿지 않는다.
from pkrc_control.live_tuning import _terminate

# MJPEG multipart 경계 문자열. 브라우저가 프레임 구분에 쓴다.
BOUNDARY = 'pkrcframe'

# 누르고 있는 동안만 유효한 이동 키. 이 키를 보낸 뒤 브라우저가 조용해지면
# 통신이 끊긴 것으로 보고 강제 정지시킨다.
# w/s 는 제외 — 목표 수심을 한 번 바꾸는 1회성 키이고(그 뒤는 depth hold 가
# 유지), 홀드 반복 전송이 없어 watchdog 이 정상 조작을 단절로 오판한다.
# r/t/c/x/q 도 같은 이유로 제외.
MOTION_KEYS = frozenset({'UP', 'DOWN', 'LEFT', 'RIGHT', 'a', 'd'})

# 이 시간 동안 이동 키가 갱신되지 않으면 정지시킨다 [초].
KEY_TIMEOUT = 0.5

# 전 축 정지 + 컨트롤러 리셋 키 (두 조종 노드 공통).
STOP_KEY = 'x'

# POST /key 로 허용하는 키 전체 — 두 조종 노드가 실제로 이해하는 키만 통과시킨다.
VALID_KEYS = frozenset({
    'UP', 'DOWN', 'LEFT', 'RIGHT',
    'w', 's', 'a', 'd', 'r', 't', 'x', 'q', 'c',
})

# 레이저 카메라 — 토글이 아니다. /image_raw/compressed 를 내는 유일한
# 노드이므로 시작 시 확보한다. exploreHD(/dev/video0) 를 쓰기 때문에
# full_system 의 gscam 과는 동시 실행이 불가능하다(V4L2 배타 open).
CAMERA_CMD = ['ros2', 'launch', 'laser_camera_publisher',
              'laser_camera.launch.py']
CAMERA_TOPIC = '/image_raw/compressed'

# 토글 가능한 노드. group 이 같은 항목은 상호배타 — 하나를 켜면 다른
# 하나를 먼저 내린다.
#
# full_system.launch.py 를 통짜로 넣지 않는 이유: 그 안의 exploreHD gscam 이
# 레이저 카메라와 /dev/video0 를, stellarHD usb_cam 이 ArUco 와 /dev/video4 를
# 다툰다. 하위 launch 가 전부 개별 패키지에 존재하므로 쪼개서 쓴다.
NODE_SPECS = {
    'pressure': {
        'label': '압력 (Bar10XT)',
        'cmd': ['ros2', 'launch', 'bar10xt_ros2', 'bar10xt.launch.py'],
        'group': None,
    },
    'dvl': {
        'label': 'DVL-A50',
        'cmd': ['ros2', 'launch', 'dvl_a50', 'dvl_a50.launch.py'],
        'group': None,
    },
    'imu': {
        'label': 'IMU (GV7-INS)',
        'cmd': ['ros2', 'launch', 'microstrain_inertial_driver',
                'microstrain_launch.py'],
        'group': None,
    },
    'led': {
        'label': 'Lumen LED',
        'cmd': ['ros2', 'launch', 'lumen_led', 'lumen_led.launch.py'],
        'group': None,
    },
    'sonar': {
        'label': 'Ping1D 소나',
        'cmd': ['ros2', 'launch', 'ping1d_sonar', 'ping_sonar.launch.py',
                'use_rviz:=false'],
        'group': None,
    },
    # ── video4 (stellarHD) 상호배타 ────────────────────────────────
    # localization.launch.py 는 내부에 aruco_detector_6dof 를 포함한다
    # (localization.launch.py:62). 따라서 ArUco 단독과 동시에 뜨면
    # 같은 카메라와 같은 /aruco/pose_array 를 다툰다.
    'localization': {
        'label': '측위 (UKFM + ArUco)',
        'cmd': ['ros2', 'launch', 'pkrc_controller', 'localization.launch.py'],
        'group': 'video4',
    },
    'aruco': {
        'label': 'ArUco 단독',
        'cmd': ['ros2', 'run', 'active_marker', 'aruco_detector_6dof'],
        'group': 'video4',
    },
    # ── 조종 상호배타 ──────────────────────────────────────────────
    # 같은 CAN 버스와 같은 VESC ID(0x151~0x156)를 만지므로 절대
    # 동시에 떠서는 안 된다.
    'teleop': {
        'label': '수동 조종 (teleop)',
        'cmd': ['ros2', 'run', 'pkrc_control', 'keyboard_control_teleop',
                '--ros-args', '-p', 'tuning_gui:=false'],
        'group': 'control',
    },
    'wall_align': {
        'label': '벽면 정렬 (wall_align)',
        'cmd': ['ros2', 'run', 'pkrc_control', 'keyboard_control_wall_align',
                '--ros-args', '-p', 'tuning_gui:=false'],
        'group': 'control',
    },
}

# 조종 노드 키 → ROS 노드 이름 (파라미터 서비스 호출 대상).
# 이름은 각 파일의 super().__init__() 인수와 정확히 일치해야 한다
# (keyboard_control_teleop.py:351, keyboard_control_wall_align.py:408).
# 틀리면 파라미터 서비스가 존재하지 않아 게인 튜닝이 전부 실패한다.
CONTROL_NODE_NAMES = {
    'teleop': 'keyboard_teleop_robust',
    'wall_align': 'keyboard_teleop_wall_align',
}

# 프리셋 저장 위치. 소스 트리가 아닌 이유: 실험값이 소스를 오염시키지 않고,
# 패키지를 재빌드해도 살아남아야 한다.
PRESET_DIR = os.path.expanduser('~/.ros/pkrc_presets')


def group_siblings(key: str) -> list:
    """key 와 같은 그룹의 다른 노드 키들 (상호배타 대상)."""
    group = NODE_SPECS.get(key, {}).get('group')
    if group is None:
        return []
    return sorted(k for k, s in NODE_SPECS.items()
                  if s['group'] == group and k != key)


def mjpeg_frame(jpeg_bytes: bytes) -> bytes:
    """JPEG 바이트를 multipart/x-mixed-replace 한 프레임으로 감싼다.

    카메라가 만든 JPEG 를 그대로 넣는다 — 디코딩도 재인코딩도 하지 않는다.
    laser_camera_publisher 가 이미 MJPG passthrough 로 퍼블리시하므로
    (camera_node.py:34-38) 여기서 손댈 이유가 없다.
    """
    return (
        f'--{BOUNDARY}\r\n'
        f'Content-Type: image/jpeg\r\n'
        f'Content-Length: {len(jpeg_bytes)}\r\n\r\n'
    ).encode() + jpeg_bytes + b'\r\n'


class FrameStore:
    """최신 JPEG 프레임 한 장만 들고 있는 스레드 안전 슬롯.

    ROS 콜백(rclpy 스레드)이 put 하고, HTTP 스트림 스레드들이 get 한다.
    한 장만 유지하는 것은 퍼블리셔의 depth=1 과 같은 의도다 — 늦은
    소비자에게 오래된 프레임을 몰아주지 않는다.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._data = None
        self._stamp = 0.0

    def put(self, data: bytes, stamp: float):
        with self._lock:
            self._data = data
            self._stamp = stamp

    def get(self):
        """(JPEG 바이트, 수신 시각) 또는 (None, 0.0)"""
        with self._lock:
            return self._data, self._stamp

    def age(self, now: float = None) -> float:
        """마지막 프레임 이후 경과 초. 프레임이 없으면 inf."""
        with self._lock:
            if self._data is None:
                return float('inf')
            return (time.time() if now is None else now) - self._stamp


class KeyWatchdog:
    """브라우저와의 통신이 끊기면 로봇을 정지시킨다.

    탭을 닫거나 와이파이가 끊기면 마지막 이동 명령이 노드에 남아 계속
    추력을 낸다. 물속 로봇이 명령 없이 밀고 나가는 것은 회수 불가
    상황이므로, 무응답을 감지해 정지 키를 강제 발행한다.

    노드 자체에도 key_timeout 0.4초 decay 가 있지만(keyboard_control_teleop.py:537)
    그것은 추력을 0 으로 줄일 뿐 적분기와 heading target 을 정리하지
    않는다. x 는 컨트롤러까지 리셋한다.

    스레드 안전성: touch() 는 HTTP 워커 스레드에서, check() 는 rclpy 타이머
    스레드에서 호출되지만 lock 이 없다. CPython 의 GIL 이 단일 속성
    읽기/쓰기를 원자적으로 만들어 주므로 위험한 인터리빙(정지가 필요한데
    억제되는 경우)이 생기지 않아 현재는 안전하다. 필드를 추가하거나
    여러 필드를 함께 갱신하게 되면 이 가정이 깨지니 그때는 lock 을 재검토할 것.
    """

    def __init__(self, timeout: float = KEY_TIMEOUT):
        self.timeout = timeout
        self._last_motion = 0.0
        self._armed = False       # 감시 중인가 (이동 키를 받은 상태)

    def touch(self, key: str, now: float):
        """키 수신을 기록한다. 이동 키면 감시를 (재)시작한다."""
        if key in MOTION_KEYS:
            self._last_motion = now
            self._armed = True

    def check(self, now: float):
        """정지가 필요하면 STOP_KEY, 아니면 None.

        한 번 발동하면 disarm 되어 새 이동 키가 올 때까지 재발동하지
        않는다 — x 를 10Hz 로 계속 쏘면 로그가 폭주하고 조종 복귀를 방해한다.
        """
        if not self._armed:
            return None
        if now - self._last_motion > self.timeout:
            self._armed = False
            return STOP_KEY
        return None


class TopicCache:
    """토픽 최신값을 들고 있다가 오래되면 None 을 반환한다.

    노드가 죽었을 때 마지막 값을 계속 보여주면 살아있는 것으로 오인한다.
    stale_sec 을 넘기면 없는 것으로 취급해 UI 가 비활성으로 그리게 한다.
    """

    def __init__(self, stale_sec: float = 1.0):
        self.stale_sec = stale_sec
        self._lock = threading.Lock()
        self._value = None
        self._stamp = 0.0

    def put(self, value, now: float = None):
        with self._lock:
            self._value = value
            self._stamp = time.time() if now is None else now

    def get(self, now: float = None):
        with self._lock:
            if self._value is None:
                return None
            t = time.time() if now is None else now
            if t - self._stamp > self.stale_sec:
                return None
            return self._value


class ProcManager:
    """launch/run 프로세스를 띄우고 내린다.

    상호배타 그룹을 강제하는 것이 이 클래스의 핵심 책임이다. 같은 V4L2
    장치나 같은 CAN 버스를 두 노드가 잡으면 조용히 실패하거나 로봇이
    오작동하므로, 조합 자체를 만들 수 없게 한다.
    """

    def __init__(self, logger, on_before_stop=None):
        self._logger = logger
        self._procs = {}          # key → Popen
        self._lock = threading.Lock()
        # control 그룹을 내리기 전에 정지 키를 보내기 위한 훅
        self._on_before_stop = on_before_stop

    def _alive(self, key: str) -> bool:
        p = self._procs.get(key)
        return p is not None and p.poll() is None

    def status(self) -> dict:
        with self._lock:
            # 죽은 프로세스 정리
            for k in [k for k in self._procs if not self._alive(k)]:
                self._procs.pop(k, None)
            return {k: self._alive(k) for k in NODE_SPECS}

    def start(self, key: str):
        spec = NODE_SPECS.get(key)
        if spec is None:
            return False, f'알 수 없는 노드: {key}'

        with self._lock:
            if self._alive(key):
                return True, f'{spec["label"]} 이미 실행 중'

            # 상호배타: 같은 그룹의 형제를 먼저 내린다
            for sib in group_siblings(key):
                if self._alive(sib):
                    self._logger.info(
                        f'{NODE_SPECS[sib]["label"]} 종료 — '
                        f'{spec["label"]} 과 같은 그룹'
                        f'({spec["group"]}) 이라 동시 실행 불가')
                    self._stop_locked(sib)

            try:
                p = subprocess.Popen(
                    spec['cmd'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True)
            except (OSError, FileNotFoundError) as e:
                return False, f'{spec["label"]} 실행 실패: {e}'

            # 즉시 죽는 경우를 잡는다 (패키지 미빌드, 장치 없음 등)
            time.sleep(0.5)
            if p.poll() is not None:
                return False, (f'{spec["label"]} 이 바로 종료됨 '
                               f'(코드 {p.returncode}) — 패키지 빌드와 '
                               f'장치 연결을 확인하세요')

            self._procs[key] = p
            self._logger.info(f'{spec["label"]} 시작 (pid {p.pid})')
            return True, f'{spec["label"]} 시작'

    def _stop_locked(self, key: str):
        """락을 이미 쥔 상태에서 호출. control 그룹은 먼저 정지시킨다."""
        p = self._procs.pop(key, None)
        if p is None:
            return
        if (NODE_SPECS.get(key, {}).get('group') == 'control'
                and self._on_before_stop is not None):
            # 정지 없이 죽이면 VESC 가 마지막 전류 명령을 계속 유지한다.
            self._on_before_stop()
            time.sleep(0.3)
        _terminate(p)

    def stop(self, key: str):
        spec = NODE_SPECS.get(key)
        if spec is None:
            return False, f'알 수 없는 노드: {key}'
        with self._lock:
            if not self._alive(key):
                return True, f'{spec["label"]} 실행 중 아님'
            self._stop_locked(key)
            self._logger.info(f'{spec["label"]} 종료')
            return True, f'{spec["label"]} 종료'

    def stop_all(self):
        with self._lock:
            for key in list(self._procs):
                self._stop_locked(key)


class _Handler(BaseHTTPRequestHandler):
    """HTTP 요청 처리. self.server.node 로 GuiServer 에 접근한다."""

    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        """기본 stderr 접근 로그를 끈다 — 10Hz 폴링이라 로그가 폭주한다."""
        pass

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream_mjpeg(self, node):
        """multipart/x-mixed-replace 로 프레임을 계속 밀어낸다.

        이 응답은 스레드 하나를 계속 붙잡는다 — 그래서 서버가
        ThreadingHTTPServer 여야 한다. Content-Length 를 줄 수 없으므로
        HTTP/1.0 으로 응답해 연결 종료로 끝을 알린다.
        """
        self.protocol_version = 'HTTP/1.0'
        self.send_response(200)
        self.send_header(
            'Content-Type',
            f'multipart/x-mixed-replace; boundary={BOUNDARY}')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()

        last_stamp = 0.0
        try:
            while True:
                data, stamp = node.get_frame()
                if data is not None and stamp != last_stamp:
                    self.wfile.write(mjpeg_frame(data))
                    self.wfile.flush()
                    last_stamp = stamp
                else:
                    # 새 프레임 대기 — 30fps 주기보다 촘촘히 폴링
                    time.sleep(0.005)
        except (BrokenPipeError, ConnectionResetError):
            # 브라우저가 탭을 닫거나 새로고침한 정상 종료
            pass

    def do_GET(self):
        node = self.server.node
        if self.path in ('/', '/index.html'):
            try:
                with open(node.html_path, 'rb') as f:
                    self._send(200, 'text/html; charset=utf-8', f.read())
            except OSError as e:
                self._send(500, 'text/plain; charset=utf-8',
                           f'gui.html 을 읽을 수 없음: {e}'.encode())
        elif self.path == '/stream':
            self._stream_mjpeg(node)
        elif self.path == '/state':
            self._send(200, 'application/json',
                       json.dumps(node.state_snapshot()).encode())
        elif self.path.startswith('/params'):
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            node_name = (q.get('node') or [''])[0]
            if not node_name:
                self._send(400, 'application/json',
                           json.dumps({'ok': False,
                                       'msg': 'node 인수가 필요합니다',
                                       'params': []}).encode())
                return
            params, err = node.list_tunable(node_name)
            self._send(200, 'application/json', json.dumps({
                'ok': params is not None,
                'msg': err,
                'params': params or [],
            }).encode())
        else:
            self._send(404, 'text/plain; charset=utf-8', b'not found')

    def _read_json(self):
        """요청 본문을 JSON 으로 파싱. 실패하면 None."""
        try:
            n = int(self.headers.get('Content-Length', 0))
            if n <= 0:
                return None
            return json.loads(self.rfile.read(n))
        except (ValueError, TypeError):
            return None

    def do_POST(self):
        node = self.server.node
        body = self._read_json()
        if body is None:
            self._send(400, 'application/json',
                       json.dumps({'ok': False,
                                   'error': 'JSON 본문이 필요합니다'}).encode())
            return

        if self.path == '/key':
            key = body.get('key', '')
            if not key:
                self._send(400, 'application/json',
                           json.dumps({'ok': False,
                                       'error': 'key 가 비었습니다'}).encode())
                return
            if key not in VALID_KEYS:
                self._send(400, 'application/json',
                           json.dumps({'ok': False,
                                       'error': f'알 수 없는 키: {key!r}'}).encode())
                return
            node.publish_key(key)
            self._send(200, 'application/json',
                       json.dumps({'ok': True}).encode())

        elif self.path == '/node':
            key = body.get('key', '')
            on = bool(body.get('on'))
            ok, msg = (node.procs.start(key) if on
                       else node.procs.stop(key))
            self._send(200, 'application/json', json.dumps({
                'ok': ok, 'msg': msg, 'nodes': node.procs.status(),
            }).encode())

        elif self.path == '/param':
            ok, msg = node.set_param(
                body.get('node', ''), body.get('name', ''),
                body.get('value', 0.0))
            self._send(200, 'application/json',
                       json.dumps({'ok': ok, 'msg': msg}).encode())

        elif self.path == '/preset':
            action = body.get('action', '')
            name = body.get('name', '')
            node_name = body.get('node', '')
            if action == 'list':
                ok, msg = True, ''
            elif action == 'save':
                ok, msg = node.preset_save(name, node_name)
            elif action == 'load':
                ok, msg = node.preset_load(name, node_name)
            else:
                ok, msg = False, f'알 수 없는 action: {action}'
            self._send(200, 'application/json', json.dumps({
                'ok': ok, 'msg': msg, 'presets': node.preset_list(),
            }).encode())

        else:
            self._send(404, 'application/json',
                       json.dumps({'ok': False,
                                   'error': 'not found'}).encode())


class GuiServer(Node):
    def __init__(self):
        super().__init__('gui_server')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        self.html_path = self._find_html()

        # ── 카메라 영상 ────────────────────────────────────────────────
        self.frames = FrameStore()
        # QoS 를 퍼블리셔와 정확히 맞춰야 한다. laser_camera_publisher 는
        # BEST_EFFORT / KEEP_LAST / depth=1 로 퍼블리시하므로
        # (camera_node.py:59-63) 기본 RELIABLE 로 구독하면 한 프레임도
        # 받지 못한다. 같은 함정이 이 패키지의 압력·DVL 구독에도 있다.
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.create_subscription(
            CompressedImage, '/image_raw/compressed',
            self._image_cb, img_qos)

        # ── 조종 키 ────────────────────────────────────────────────────
        # 두 조종 노드가 /gui/key 를 구독한다. stdin(터미널) 경로와
        # 병행 동작하므로 둘 중 아무거나 써도 된다.
        self.key_pub = self.create_publisher(String, '/gui/key', 10)
        self.watchdog = KeyWatchdog()
        # 10Hz 로 통신 생존 확인 — 끊기면 강제 정지
        self.create_timer(0.1, self._watchdog_tick)

        # ── 텔레메트리 ─────────────────────────────────────────────────
        # 조종 노드들이 이미 전부 퍼블리시한다 — 새로 계산할 것이 없다.
        self.tc_yaw = TopicCache()
        self.tc_depth = TopicCache()
        self.tc_thrust = TopicCache()
        self.tc_wall = TopicCache()
        self.tc_wall_mode = TopicCache()

        self.create_subscription(
            Float64MultiArray, '/teleop/yaw_debug',
            lambda m: self.tc_yaw.put(list(m.data)), 10)
        self.create_subscription(
            Float64MultiArray, '/teleop/depth_debug',
            lambda m: self.tc_depth.put(list(m.data)), 10)
        self.create_subscription(
            Float64MultiArray, '/teleop/thruster_currents',
            lambda m: self.tc_thrust.put(list(m.data)), 10)
        self.create_subscription(
            Float64MultiArray, '/teleop/wall_debug',
            lambda m: self.tc_wall.put(list(m.data)), 10)
        self.create_subscription(
            String, '/teleop/wall_mode',
            lambda m: self.tc_wall_mode.put(m.data), 10)

        # ── 노드 프로세스 관리 ─────────────────────────────────────────
        self.procs = ProcManager(self.get_logger(),
                                 on_before_stop=self._emit_stop)
        # 파라미터 서비스 클라이언트 캐시 (노드명 → {서비스명: client})
        self._param_clients = {}
        os.makedirs(PRESET_DIR, exist_ok=True)

        # 레이저 카메라 확보 — 이미 돌면 그걸 쓰고, 없으면 띄우고 소유한다.
        self._owns_camera = False
        self.create_timer(1.0, self._ensure_camera_once)

        self._httpd = ThreadingHTTPServer((host, port), _Handler)
        self._httpd.node = self
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

        self.get_logger().info(
            f'웹 GUI 서버 시작 — http://{host}:{port} '
            f'(같은 망의 브라우저에서 접속)')

    def _image_cb(self, msg: CompressedImage):
        """카메라가 만든 JPEG 바이트를 그대로 저장한다. 디코딩하지 않는다."""
        self.frames.put(bytes(msg.data), time.time())

    def get_frame(self):
        return self.frames.get()

    def frame_age(self) -> float:
        return self.frames.age()

    def publish_key(self, key: str):
        """키를 /gui/key 로 발행하고 watchdog 을 갱신한다."""
        msg = String()
        msg.data = key
        self.key_pub.publish(msg)
        self.watchdog.touch(key, time.time())

    def _emit_stop(self):
        """조종 노드를 내리기 전 전 축 정지 + 컨트롤러 리셋."""
        msg = String()
        msg.data = STOP_KEY
        self.key_pub.publish(msg)
        self.get_logger().info('조종 노드 종료 전 정지 명령 발행')

    def _ensure_camera_once(self):
        """레이저 카메라를 한 번만 확보한다. 타이머는 즉시 자기를 끈다."""
        for t in list(self.timers):
            if t.callback == self._ensure_camera_once:
                self.destroy_timer(t)

        # 이미 퍼블리셔가 있으면 남이 띄운 것 — 소유하지 않는다.
        if self.count_publishers(CAMERA_TOPIC) > 0:
            self.get_logger().info(
                f'{CAMERA_TOPIC} 퍼블리셔가 이미 있음 — 그것을 사용하고 '
                f'gui_server 종료 시에도 살려둡니다')
            return

        try:
            self._camera_proc = subprocess.Popen(
                CAMERA_CMD,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL, start_new_session=True)
            self._owns_camera = True
            self.get_logger().info(
                f'레이저 카메라 시작 (pid {self._camera_proc.pid}) — '
                f'gui_server 종료 시 함께 정리합니다')
        except OSError as e:
            self.get_logger().error(
                f'레이저 카메라 실행 실패: {e} — 영상이 나오지 않습니다')

    # ─── 파라미터 ───────────────────────────────────────────────────
    def _client(self, node_name: str, srv_type, srv_name: str):
        """서비스 클라이언트를 캐시해 재사용한다."""
        full = f'/{node_name}/{srv_name}'
        cache = self._param_clients.setdefault(node_name, {})
        if full not in cache:
            cache[full] = self.create_client(srv_type, full)
        return cache[full]

    def _call(self, client, request, timeout=2.0):
        """서비스를 동기 호출한다. HTTP 스레드에서 호출되므로
        spin 하지 않고 future 이벤트를 기다린다 (rclpy 는 메인 스레드에서
        이미 spin 중이다)."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    def list_tunable(self, node_name: str):
        """튜닝 가능한 파라미터 목록. read_only 는 제외한다.

        목록을 하드코딩하지 않는 이유: live_tuning.py 의 _ROUTES 가 SSOT 이고
        노드마다 선언 집합이 다르다(teleop 약 40개 vs wall_align 약 60개).
        _FIXED 로 read_only 표시된 물성값·구독 토픽은 자동 제외된다.
        """
        lc = self._client(node_name, ListParameters, 'list_parameters')
        res = self._call(lc, ListParameters.Request())
        if res is None:
            return None, f'{node_name} 노드에 연결할 수 없습니다'

        names = [n for n in res.result.names if n != 'use_sim_time']
        if not names:
            return [], ''

        dc = self._client(node_name, DescribeParameters,
                          'describe_parameters')
        dreq = DescribeParameters.Request()
        dreq.names = names
        dres = self._call(dc, dreq)
        if dres is None:
            return None, f'{node_name} 파라미터 서술자를 읽을 수 없습니다'

        numeric = (ParameterType.PARAMETER_DOUBLE,
                   ParameterType.PARAMETER_INTEGER)
        keep = [d.name for d in dres.descriptors
                if not d.read_only and d.type in numeric]
        if not keep:
            return [], ''

        gc = self._client(node_name, GetParameters, 'get_parameters')
        greq = GetParameters.Request()
        greq.names = keep
        gres = self._call(gc, greq)
        if gres is None:
            return None, f'{node_name} 파라미터 값을 읽을 수 없습니다'

        out = []
        for name, pv in zip(keep, gres.values):
            if pv.type == ParameterType.PARAMETER_DOUBLE:
                out.append({'name': name, 'value': pv.double_value,
                            'type': 'double'})
            elif pv.type == ParameterType.PARAMETER_INTEGER:
                out.append({'name': name, 'value': pv.integer_value,
                            'type': 'integer'})
        return sorted(out, key=lambda p: p['name']), ''

    def set_param(self, node_name: str, name: str, value):
        """파라미터 하나를 설정한다. live_tuning 콜백이 즉시 반영한다."""
        # 현재 타입을 먼저 확인해 double/integer 를 맞춘다 —
        # 타입이 틀리면 rclpy 가 거절한다.
        params, err = self.list_tunable(node_name)
        if params is None:
            return False, err
        match = next((p for p in params if p['name'] == name), None)
        if match is None:
            return False, f'{name} 은 {node_name} 의 튜닝 대상이 아닙니다'

        pv = ParameterValue()
        if match['type'] == 'integer':
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(round(float(value)))
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)

        sc = self._client(node_name, SetParameters, 'set_parameters')
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name=name, value=pv)]
        res = self._call(sc, req)
        if res is None:
            return False, f'{node_name} 에 파라미터를 설정할 수 없습니다'
        if not res.results or not res.results[0].successful:
            reason = (res.results[0].reason if res.results else '알 수 없음')
            return False, f'{name} 설정 거절됨: {reason}'
        return True, f'{name} = {value}'

    def get_max_current(self, node_name: str):
        """활성 조종 노드의 실제 전류 한계 [surge,surge,sway,sway,heave,heave].

        gui.html 의 T_LIMITS 는 max_current_* 파라미터의 *기본값*을
        하드코딩한 것이다. 이 파라미터들은 런타임에 오버라이드될 수 있고
        (예: 1.0A 캡으로 실행한 적이 실제로 있다), 그 경우 진짜 한계보다
        큰 값으로 바를 그려 포화를 숨긴다. 노드에서 직접 읽어 그 문제를
        없앤다. 연결할 노드가 없으면 None — JS 가 하드코딩 기본값으로
        폴백한다.
        """
        names = ['max_current_surge', 'max_current_sway',
                 'max_current_heave']
        gc = self._client(node_name, GetParameters, 'get_parameters')
        req = GetParameters.Request()
        req.names = names
        res = self._call(gc, req, timeout=0.5)
        if res is None or len(res.values) != len(names):
            return None
        ms, mw, mh = (v.double_value for v in res.values)
        return [ms, ms, mw, mw, mh, mh]

    # ─── 프리셋 ─────────────────────────────────────────────────────
    def preset_path(self, name: str) -> str:
        """경로 탈출을 막는다 — 이름에서 디렉터리 성분을 제거."""
        safe = os.path.basename(name).replace('/', '_').strip()
        if not safe or safe.startswith('.'):
            raise ValueError('프리셋 이름이 올바르지 않습니다')
        if not safe.endswith('.yaml'):
            safe += '.yaml'
        return os.path.join(PRESET_DIR, safe)

    def preset_list(self):
        try:
            return sorted(f[:-5] for f in os.listdir(PRESET_DIR)
                          if f.endswith('.yaml'))
        except OSError:
            return []

    def preset_save(self, name: str, node_name: str):
        params, err = self.list_tunable(node_name)
        if params is None:
            return False, err
        try:
            path = self.preset_path(name)
            with open(path, 'w') as f:
                yaml.safe_dump(
                    {'node': node_name,
                     'params': {p['name']: p['value'] for p in params}},
                    f, allow_unicode=True, sort_keys=True)
        except (OSError, ValueError) as e:
            return False, f'프리셋 저장 실패: {e}'
        return True, f'{name} 저장 ({len(params)}개 파라미터)'

    def preset_load(self, name: str, node_name: str):
        try:
            with open(self.preset_path(name)) as f:
                data = yaml.safe_load(f) or {}
        except (OSError, ValueError, yaml.YAMLError) as e:
            return False, f'프리셋 읽기 실패: {e}'

        saved_node = data.get('node')
        if saved_node != node_name:
            # 파라미터 집합이 다르므로 섞어 쓰면 엉뚱한 값이 들어간다.
            return False, (f'이 프리셋은 {saved_node} 용입니다 '
                           f'(현재 {node_name})')

        ok, fail = 0, []
        for pname, pvalue in (data.get('params') or {}).items():
            good, msg = self.set_param(node_name, pname, pvalue)
            ok += 1 if good else 0
            if not good:
                fail.append(pname)
        if fail:
            return True, (f'{ok}개 적용, {len(fail)}개 실패: '
                          f'{", ".join(fail[:5])}')
        return True, f'{name} 적용 ({ok}개 파라미터)'

    def _watchdog_tick(self):
        """브라우저 무응답 감시 — 끊기면 정지 키를 강제 발행."""
        stop = self.watchdog.check(time.time())
        if stop is not None:
            msg = String()
            msg.data = stop
            self.key_pub.publish(msg)
            self.get_logger().warn(
                f'브라우저 무응답 {self.watchdog.timeout}초 — '
                f'강제 정지({stop}) 발행')

    def _find_html(self) -> str:
        """gui.html 경로를 찾는다. 설치본 우선, 없으면 소스 트리."""
        try:
            from ament_index_python.packages import get_package_share_directory
            p = os.path.join(
                get_package_share_directory('pkrc_control'), 'gui.html')
            if os.path.exists(p):
                return p
        except Exception:
            pass
        # 소스 트리 폴백 (colcon build 전에도 돌게)
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'resource', 'gui.html')

    def state_snapshot(self) -> dict:
        """브라우저에 보낼 현재 상태 전체."""
        age = self.frame_age()
        nodes = self.procs.status()
        # 활성 조종 노드가 있으면 그 노드의 실제 전류 한계를 읽는다 —
        # max_current_* 는 런타임에 오버라이드될 수 있어 UI 의 하드코딩
        # 기본값만으로는 포화가 숨겨질 수 있다 (예: 1.0A 캡 운용 시).
        max_current = None
        for key, node_name in CONTROL_NODE_NAMES.items():
            if nodes.get(key):
                max_current = self.get_max_current(node_name)
                break
        return {
            'camera': {
                # 1초 넘게 프레임이 없으면 카메라 노드가 죽은 것으로 본다.
                # 검은 화면만 보여주면 원인을 알 수 없으므로 명시한다.
                'alive': age < 1.0,
                'age': None if age == float('inf') else round(age, 2),
            },
            'yaw': self.tc_yaw.get(),
            'depth': self.tc_depth.get(),
            'thrusters': self.tc_thrust.get(),
            'wall': self.tc_wall.get(),
            'wall_mode': self.tc_wall_mode.get(),
            'nodes': nodes,
            'max_current': max_current,
        }

    def shutdown(self):
        self._httpd.shutdown()
        self._httpd.server_close()
        self.procs.stop_all()
        # 내가 띄운 카메라만 정리한다 — 남이 띄운 것은 건드리지 않는다.
        if self._owns_camera and getattr(self, '_camera_proc', None):
            _terminate(self._camera_proc)
            self.get_logger().info('레이저 카메라 정리 완료')


def _selftest():
    """rclpy 없이 순수 로직만 검증한다."""
    payload = b'\xff\xd8TEST\xff\xd9'
    out = mjpeg_frame(payload)
    assert out.startswith(f'--{BOUNDARY}\r\n'.encode()), 'MJPEG 경계 오류'
    assert b'Content-Type: image/jpeg' in out, 'MJPEG 헤더 오류'
    assert out.split(b'\r\n\r\n', 1)[1] == payload + b'\r\n', 'JPEG 본문 변조'
    print('selftest: MJPEG 프레이밍 OK')

    wd = KeyWatchdog(timeout=0.5)
    wd.touch('UP', now=100.0)
    assert wd.check(now=100.3) is None, 'watchdog 조기 발동'
    assert wd.check(now=100.6) == STOP_KEY, 'watchdog 미발동'
    assert wd.check(now=101.0) is None, 'watchdog 중복 발동'
    wd.touch('r', now=200.0)
    assert wd.check(now=201.0) is None, '비이동 키에 발동'
    print('selftest: watchdog OK')

    # 인터록: 같은 그룹은 서로 형제, 다른 그룹·무그룹은 형제 없음
    assert group_siblings('localization') == ['aruco'], 'video4 인터록 오류'
    assert group_siblings('teleop') == ['wall_align'], 'control 인터록 오류'
    assert group_siblings('sonar') == [], '무그룹에 형제가 있음'
    # 제외 대상이 실수로 들어가지 않았는지
    joined = ' '.join(' '.join(s['cmd']) for s in NODE_SPECS.values()).lower()
    assert 'gscam' not in joined, 'gscam 이 포함됨 (video0 충돌)'
    assert 'usb_cam' not in joined, 'usb_cam 이 포함됨 (video4 충돌)'
    print('selftest: 인터록 OK')

    # ProcManager 인터록이 실제로 형제를 내리는지 (가짜 프로세스로)
    class _FakeLogger:
        def info(self, *a): pass
        def warn(self, *a): pass
        def error(self, *a): pass

    pm = ProcManager(_FakeLogger())

    class _FakeProc:
        def __init__(self): self.pid, self.killed = 1234, False
        def poll(self): return None

    pm._procs['localization'] = _FakeProc()
    assert pm._alive('localization')
    # aruco 를 켜면 localization 이 내려가야 한다 (실제 실행은 하지 않고
    # _stop_locked 경로만 확인)
    pm._stop_locked('localization')
    assert 'localization' not in pm._procs, '형제 종료 실패'
    print('selftest: ProcManager OK')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--selftest', action='store_true',
                        help='rclpy 없이 자체 검증만 하고 종료')
    known, remaining = parser.parse_known_args(
        args if args is not None else sys.argv[1:])
    if known.selftest:
        _selftest()
        return

    rclpy.init(args=remaining)
    node = GuiServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
