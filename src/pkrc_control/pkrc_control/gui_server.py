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
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float64MultiArray

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
            'nodes': {},
        }

    def shutdown(self):
        self._httpd.shutdown()
        self._httpd.server_close()


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
