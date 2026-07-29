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

# MJPEG multipart 경계 문자열. 브라우저가 프레임 구분에 쓴다.
BOUNDARY = 'pkrcframe'


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
        else:
            self._send(404, 'text/plain; charset=utf-8', b'not found')


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
