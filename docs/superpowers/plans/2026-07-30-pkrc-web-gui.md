# PKRC 통합 관제 웹 GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** headless Jetson에서 브라우저로 PKRC의 센서·측위 노드를 토글하고, 레이저 카메라 영상을 실시간으로 보고, 조종 모드를 전환하고, 제어 게인을 실시간 튜닝하는 단일 웹 GUI를 만든다.

**Architecture:** rclpy 노드 하나(`gui_server`)가 stdlib `http.server.ThreadingHTTPServer`를 데몬 스레드로 띄워 웹서버 역할까지 겸한다. 별도 브리지 프로세스가 없다. 영상은 이미 JPEG인 `/image_raw/compressed` 바이트를 multipart MJPEG로 그대로 중계해 재인코딩이 0이다. 조종 명령은 새로 추가하는 `/gui/key` 토픽으로 전달하고, 기존 stdin 경로는 손대지 않는다.

**Tech Stack:** ROS 2 Humble, rclpy, Python 3.10, stdlib `http.server`/`subprocess`/`threading`, PyYAML 5.4.1 (기설치), 바닐라 JS + Canvas (프레임워크·라이브러리 없음)

## Global Constraints

- **새 런타임 의존성 0개.** flask·fastapi·rosbridge·web_video_server를 설치하지 않는다. stdlib과 이미 설치된 것(rclpy, PyYAML 5.4.1)만 쓴다.
- **`/image_raw/compressed` 구독 QoS는 반드시 `BEST_EFFORT, KEEP_LAST, depth=1`.** 퍼블리셔가 그 설정(`camera_node.py:59-63`)이므로 기본 RELIABLE·depth=10으로 구독하면 한 프레임도 받지 못한다.
- **영상은 절대 디코딩·재인코딩하지 않는다.** `msg.data`(카메라가 만든 JPEG 바이트)를 그대로 HTTP 본문에 넣는다. cv2를 gui_server에 import하지 않는다.
- **제어 코드 수정 전 `.bak-20260730` 사본을 먼저 만든다.** `hero_ws`는 git 저장소가 아니라 복구 수단이 파일 사본뿐이다.
- **기존 stdin 키 입력 경로를 제거하거나 변경하지 않는다.** `/gui/key`는 추가 경로일 뿐이며, 터미널 조작이 계속 동작해야 한다.
- **exploreHD gscam과 stellarHD usb_cam은 UI에 넣지 않는다.** 각각 `/dev/video0`(레이저 카메라)과 `/dev/video4`(ArUco)를 다툰다.
- **바인드 주소는 `0.0.0.0`, 포트 `8080`** (launch 인수로 변경 가능).
- **프로세스 종료는 `live_tuning.py:158-207`의 `_descendants()` + `_terminate()`를 재사용한다.** 새로 구현하지 않는다.
- **한국어 주석·로그를 유지한다.** 기존 파일들의 주석 밀도와 스타일을 따른다.
- **검증은 프레임워크 없이 `--selftest` 플래그 하나로 한다.** pytest 픽스처·suite를 만들지 않는다.

---

## File Structure

| 경로 | 책임 | 상태 |
|:---|:---|:---|
| `src/pkrc_control/pkrc_control/gui_server.py` | rclpy 노드 + HTTP 핸들러 + 프로세스 관리 + watchdog | 신규 |
| `src/pkrc_control/resource/gui.html` | 단일 페이지 UI (인라인 CSS/JS) | 신규 |
| `src/pkrc_control/launch/gui.launch.py` | gui_server 기동 (host·port 인수) | 신규 (`launch/` 디렉터리도 신규) |
| `src/pkrc_control/setup.py` | entry_point + data_files 추가 | 수정 |
| `src/pkrc_control/pkrc_control/keyboard_control_teleop.py` | `/gui/key` 구독 추가 (약 15줄) | 수정 |
| `src/pkrc_control/pkrc_control/keyboard_control_wall_align.py` | `/gui/key` 구독 추가 (약 15줄) | 수정 |

`gui_server.py` 하나에 모든 서버 로직을 담는다. 파일을 쪼개지 않는 이유는 라우트가 7개뿐이고, 공유 상태(최신 프레임·텔레메트리·heartbeat)를 하나의 Lock으로 보호해야 해서 분리하면 오히려 결합이 드러나기 때문이다. 파일이 600줄을 넘어가면 그때 프로세스 관리를 분리한다.

## Task 순서 근거

Task 1(백업)이 맨 앞인 이유는 되돌릴 수단을 먼저 확보해야 하기 때문이다. Task 2(노드 수정)를 서버보다 먼저 하는 이유는 `/gui/key` 구독이 없으면 서버의 조종 기능을 테스트할 수 없기 때문이다. Task 3~7은 서버를 기능 단위로 쌓아 올린다.

---

### Task 1: 제어 코드 백업

**Files:**
- Create: `src/pkrc_control/pkrc_control/keyboard_control_teleop.py.bak-20260730`
- Create: `src/pkrc_control/pkrc_control/keyboard_control_wall_align.py.bak-20260730`

**Interfaces:**
- Consumes: 없음
- Produces: 원본 사본 2개. 이후 모든 태스크에서 문제 발생 시 `cp <파일>.bak-20260730 <파일>`로 복구한다.

- [ ] **Step 1: 사본 생성**

```bash
cd /home/hero/hero_ws/src/pkrc_control/pkrc_control
cp keyboard_control_teleop.py keyboard_control_teleop.py.bak-20260730
cp keyboard_control_wall_align.py keyboard_control_wall_align.py.bak-20260730
```

- [ ] **Step 2: 사본이 원본과 동일한지 검증**

```bash
cd /home/hero/hero_ws/src/pkrc_control/pkrc_control
diff keyboard_control_teleop.py keyboard_control_teleop.py.bak-20260730 && echo "teleop 사본 OK"
diff keyboard_control_wall_align.py keyboard_control_wall_align.py.bak-20260730 && echo "wall_align 사본 OK"
```

Expected: 두 `diff` 모두 출력이 없고 "사본 OK" 두 줄이 찍힌다.

- [ ] **Step 3: colcon이 `.bak-*`를 무시하는지 확인**

```bash
cd /home/hero/hero_ws
colcon build --packages-select pkrc_control 2>&1 | tail -5
ls install/pkrc_control/lib/pkrc_control/
```

Expected: 빌드 성공. `install/.../lib/pkrc_control/`에 `keyboard_control_teleop.py.bak-20260730` 같은 실행파일이 **생기지 않는다**. `find_packages`는 `.py`로 끝나는 모듈만 잡으므로 `.bak-20260730` 확장자는 패키지에 포함되지 않는다.

---

### Task 2: 두 조종 노드에 `/gui/key` 구독 추가

**Files:**
- Modify: `src/pkrc_control/pkrc_control/keyboard_control_teleop.py` (`__init__` 583행 근처, `get_key` 712행)
- Modify: `src/pkrc_control/pkrc_control/keyboard_control_wall_align.py` (`__init__` 744행 근처, `get_key` 1020행)

**Interfaces:**
- Consumes: Task 1의 백업 사본
- Produces: 두 노드가 `std_msgs/String` 타입 `/gui/key` 토픽을 구독한다. 발행하는 문자열은 노드의 기존 키 문자열과 정확히 같은 형식이다 — `'UP'`, `'DOWN'`, `'LEFT'`, `'RIGHT'` (방향키, 대문자 그대로), `'w'`, `'s'`, `'a'`, `'d'`, `'r'`, `'t'`, `'x'`, `'q'`, `'c'` (단일 문자, 소문자). 노드는 이 값을 stdin에서 온 키와 완전히 동일하게 처리한다.

**배경 (구현자가 알아야 할 것):** 두 노드는 `tty.setraw(sys.stdin.fileno())`로 자기 터미널의 stdin을 raw 모드로 읽는다. 웹서버는 그 노드에 stdin을 줄 수 없다. 그래서 `get_key()`가 stdin을 읽기 **전에** 토픽으로 들어온 키를 먼저 반환하게 한다. 기존 stdin 로직은 한 줄도 지우지 않는다 — 터미널 조작이 계속 동작해야 한다.

`String`은 두 파일 모두 이미 import되어 있다(`from std_msgs.msg import ... String`). 추가 import가 필요 없다.

- [ ] **Step 1: 실패하는 테스트 작성**

Create: `src/pkrc_control/test/test_gui_key.py`

```python
#!/usr/bin/env python3
"""/gui/key 토픽으로 들어온 키가 get_key() 로 나오는지 검증.

rclpy 노드 전체를 띄우지 않는다 — CAN 버스·IMU·터미널이 없는 환경에서도
돌아야 하므로, get_key 의 큐 우선순위 로직만 떼어내 검증한다.
"""


class _FakeNode:
    """get_key 의 큐 분기만 재현한 최소 스텁."""

    def __init__(self):
        self._pending_gui_key = ''
        self.settings = None
        self.stdin_result = ''

    def _gui_key_cb(self, msg):
        self._pending_gui_key = msg.data

    def get_key(self):
        # 실제 노드와 동일한 우선순위: 큐 먼저, 없으면 stdin
        if self._pending_gui_key:
            k, self._pending_gui_key = self._pending_gui_key, ''
            return k
        return self.stdin_result


class _Msg:
    def __init__(self, data):
        self.data = data


def test_gui_key_returned_once():
    """큐에 넣은 키가 한 번만 나오고, 그 다음엔 비어야 한다."""
    n = _FakeNode()
    n._gui_key_cb(_Msg('UP'))
    assert n.get_key() == 'UP'
    assert n.get_key() == ''


def test_stdin_still_works_when_queue_empty():
    """큐가 비면 기존 stdin 경로가 그대로 동작해야 한다."""
    n = _FakeNode()
    n.stdin_result = 'w'
    assert n.get_key() == 'w'


def test_gui_key_takes_priority_over_stdin():
    """큐에 키가 있으면 stdin 보다 먼저 나온다."""
    n = _FakeNode()
    n.stdin_result = 'w'
    n._gui_key_cb(_Msg('x'))
    assert n.get_key() == 'x'
    assert n.get_key() == 'w'   # 큐 소진 후 stdin 복귀


if __name__ == '__main__':
    test_gui_key_returned_once()
    test_stdin_still_works_when_queue_empty()
    test_gui_key_takes_priority_over_stdin()
    print('test_gui_key: 3 passed')
```

- [ ] **Step 2: 테스트를 실행해 통과를 확인 (이 테스트는 계약 문서 역할)**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_key.py
```

Expected: `test_gui_key: 3 passed`

이 테스트는 스텁 기반이라 처음부터 통과한다. 목적은 두 노드에 넣을 로직의 계약(큐 우선, 한 번만 반환, stdin 폴백)을 고정하는 것이다. Step 3~4에서 실제 노드가 이 계약과 같은 코드를 갖게 한다.

- [ ] **Step 3: `keyboard_control_teleop.py` 수정**

`self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)` (584행) **바로 앞**에 다음을 삽입한다:

```python
        # ── 웹 GUI 키 입력 ─────────────────────────────────────────────
        # gui_server 가 /gui/key 로 키를 보낸다. stdin(터미널) 경로는 그대로
        # 살아 있고, 이건 추가 입력 채널일 뿐이다 — 둘 중 아무거나 써도 된다.
        # get_key() 가 큐를 먼저 확인하므로 웹 입력이 우선한다.
        self._pending_gui_key = ''
        self.create_subscription(String, '/gui/key', self._gui_key_cb, 10)

```

`def get_key(self):` (712행) **바로 다음 줄**에 다음을 삽입한다:

```python
        # 웹 GUI 로 들어온 키를 먼저 소비 (한 번만 반환)
        if self._pending_gui_key:
            k, self._pending_gui_key = self._pending_gui_key, ''
            return k
```

`get_key` 메서드 **앞**에 콜백을 추가한다:

```python
    def _gui_key_cb(self, msg: String):
        """웹 GUI 키를 큐에 넣는다. 다음 get_key() 가 소비한다."""
        self._pending_gui_key = msg.data

```

- [ ] **Step 4: `keyboard_control_wall_align.py` 에 동일 수정**

`self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)` (745행) **바로 앞**에 삽입:

```python
        # ── 웹 GUI 키 입력 ─────────────────────────────────────────────
        # gui_server 가 /gui/key 로 키를 보낸다. stdin(터미널) 경로는 그대로
        # 살아 있고, 이건 추가 입력 채널일 뿐이다 — 둘 중 아무거나 써도 된다.
        # C 키(정렬 시퀀스)와 중단 키도 이 경로로 들어온다.
        self._pending_gui_key = ''
        self.create_subscription(String, '/gui/key', self._gui_key_cb, 10)

```

`def get_key(self):` (1020행) **바로 다음 줄**에 삽입:

```python
        # 웹 GUI 로 들어온 키를 먼저 소비 (한 번만 반환)
        if self._pending_gui_key:
            k, self._pending_gui_key = self._pending_gui_key, ''
            return k
```

`get_key` 메서드 **앞**에 콜백 추가:

```python
    def _gui_key_cb(self, msg: String):
        """웹 GUI 키를 큐에 넣는다. 다음 get_key() 가 소비한다."""
        self._pending_gui_key = msg.data

```

- [ ] **Step 5: 문법 검증 및 diff 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control/pkrc_control
python3 -m py_compile keyboard_control_teleop.py keyboard_control_wall_align.py && echo "문법 OK"
diff keyboard_control_teleop.py.bak-20260730 keyboard_control_teleop.py
diff keyboard_control_wall_align.py.bak-20260730 keyboard_control_wall_align.py
```

Expected: "문법 OK". 각 `diff`가 삽입된 3덩어리(구독 등록, 콜백, get_key 분기)만 보여준다. **기존 줄이 삭제되거나 변경된 것이 하나도 없어야 한다** — `<` 로 시작하는 삭제 줄이 있으면 잘못된 것이니 백업에서 복구하고 다시 한다.

- [ ] **Step 6: 빌드 및 노드 기동 확인**

```bash
cd /home/hero/hero_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select pkrc_control 2>&1 | tail -3
source install/setup.bash
timeout 5 ros2 run pkrc_control keyboard_control_teleop 2>&1 | head -20
```

Expected: 빌드 성공. 노드가 뜨고 배너가 출력된다. CAN 버스가 없으면 `CAN init failed` 경고가 나오는데 정상이다(`keyboard_control_teleop.py:496-498`이 예외를 잡고 계속 진행). `_pending_gui_key`나 `_gui_key_cb` 관련 `AttributeError`가 나오면 삽입 위치가 틀렸다.

- [ ] **Step 7: 토픽 구독 실제 확인**

터미널 A:
```bash
cd /home/hero/hero_ws && source install/setup.bash
ros2 run pkrc_control keyboard_control_teleop
```

터미널 B:
```bash
cd /home/hero/hero_ws && source install/setup.bash
ros2 topic info /gui/key
ros2 topic pub --once /gui/key std_msgs/msg/String "{data: 'r'}"
```

Expected: `ros2 topic info`가 `Subscription count: 1`을 보고한다. `r` 발행 후 터미널 A에 `Heading reset:` 로그가 찍힌다(IMU가 없으면 `yaw_initialized`가 False라 로그가 안 나올 수 있다 — 그 경우 `x`를 보내 `STOP (all axes + controllers reset)` 로그로 확인한다).

---

### Task 3: gui_server 골격 — 노드 + HTTP 서버 + 정적 페이지

**Files:**
- Create: `src/pkrc_control/pkrc_control/gui_server.py`
- Create: `src/pkrc_control/resource/gui.html` (이 태스크에서는 최소 플레이스홀더)
- Create: `src/pkrc_control/launch/gui.launch.py`
- Modify: `src/pkrc_control/setup.py`

**Interfaces:**
- Consumes: 없음 (Task 2와 독립)
- Produces:
  - `class GuiServer(Node)` — rclpy 노드. 생성자 인수 없음. 파라미터 `host`(str, 기본 `'0.0.0.0'`), `port`(int, 기본 `8080`).
  - `GuiServer.html_path` → `str`: 서빙할 `gui.html` 절대 경로.
  - `GuiServer.shutdown()` → `None`: HTTP 서버와 소유 프로세스를 정리한다.
  - `_Handler(BaseHTTPRequestHandler)` — `self.server.node` 로 `GuiServer` 인스턴스에 접근한다.
  - 엔트리포인트 `gui_server = pkrc_control.gui_server:main`
  - `main()` 이 `--selftest` 인수를 받으면 rclpy 없이 자체 검증만 하고 종료한다.

**배경:** `ThreadingHTTPServer`를 써야 한다. 단일 스레드 서버면 Task 4의 MJPEG 스트림 응답이 스레드를 영구 점유해 다른 모든 요청을 막는다.

`gui.html` 경로는 ament의 share 디렉터리에서 찾는다. `ament_index_python.packages.get_package_share_directory('pkrc_control')` 아래 `gui.html`이 설치된다.

- [ ] **Step 1: 실패하는 테스트 작성**

Create: `src/pkrc_control/test/test_gui_server.py`

```python
#!/usr/bin/env python3
"""gui_server 의 순수 로직 검증 — rclpy·하드웨어 없이 돌아야 한다.

여기서 검증하는 것은 ROS 와 무관한 부분뿐이다:
  - multipart MJPEG 프레이밍이 규격에 맞는지
  - watchdog 이 정확한 시점에 정지 키를 내는지
  - 인터록이 같은 그룹의 이전 노드를 내리는지
실제 영상·추력·CAN 은 하드웨어를 붙여 눈으로 확인한다.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pkrc_control.gui_server import mjpeg_frame, BOUNDARY


def test_mjpeg_frame_structure():
    """multipart 프레임이 경계·헤더·본문 순서를 지키는지."""
    payload = b'\xff\xd8\xff\xe0JPEGBYTES\xff\xd9'
    out = mjpeg_frame(payload)

    assert out.startswith(b'--' + BOUNDARY.encode() + b'\r\n')
    assert b'Content-Type: image/jpeg\r\n' in out
    assert b'Content-Length: ' + str(len(payload)).encode() + b'\r\n' in out
    # 헤더와 본문 사이는 빈 줄 하나
    assert b'\r\n\r\n' + payload in out
    assert out.endswith(payload + b'\r\n')


def test_mjpeg_frame_does_not_alter_payload():
    """JPEG 바이트가 한 바이트도 바뀌지 않아야 한다 (재인코딩 금지)."""
    payload = bytes(range(256))
    out = mjpeg_frame(payload)
    assert payload in out
    # 본문 부분만 떼어내 원본과 완전 일치 확인
    body = out.split(b'\r\n\r\n', 1)[1]
    assert body == payload + b'\r\n'


if __name__ == '__main__':
    test_mjpeg_frame_structure()
    test_mjpeg_frame_does_not_alter_payload()
    print('test_gui_server(frame): 2 passed')
```

- [ ] **Step 2: 테스트를 실행해 실패를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
```

Expected: FAIL — `ModuleNotFoundError: No module named 'pkrc_control.gui_server'`

- [ ] **Step 3: `gui_server.py` 골격 작성**

Create: `src/pkrc_control/pkrc_control/gui_server.py`

```python
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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

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

    def do_GET(self):
        node = self.server.node
        if self.path in ('/', '/index.html'):
            try:
                with open(node.html_path, 'rb') as f:
                    self._send(200, 'text/html; charset=utf-8', f.read())
            except OSError as e:
                self._send(500, 'text/plain; charset=utf-8',
                           f'gui.html 을 읽을 수 없음: {e}'.encode())
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

        self._httpd = ThreadingHTTPServer((host, port), _Handler)
        self._httpd.node = self
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

        self.get_logger().info(
            f'웹 GUI 서버 시작 — http://{host}:{port} '
            f'(같은 망의 브라우저에서 접속)')

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
```

- [ ] **Step 4: 테스트를 실행해 통과를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
python3 pkrc_control/gui_server.py --selftest
```

Expected: `test_gui_server(frame): 2 passed` 그리고 `selftest: MJPEG 프레이밍 OK`

- [ ] **Step 5: 최소 `gui.html` 작성**

Create: `src/pkrc_control/resource/gui.html`

```html
<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PKRC 관제</title>
</head>
<body>
<h1>PKRC 관제</h1>
<p>서버 동작 중. UI 는 이후 태스크에서 구현한다.</p>
</body>
</html>
```

- [ ] **Step 6: launch 파일 작성**

Create: `src/pkrc_control/launch/gui.launch.py`

```python
#!/usr/bin/env python3
"""PKRC 웹 GUI 서버 실행.

    ros2 launch pkrc_control gui.launch.py
    ros2 launch pkrc_control gui.launch.py port:=9000
    ros2 launch pkrc_control gui.launch.py host:=127.0.0.1   # 로컬만 허용
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='0.0.0.0',
            description='바인드 주소. 0.0.0.0 은 같은 망에서 접속 허용, '
                        '127.0.0.1 은 로컬(SSH 포트포워딩)만 허용',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='HTTP 포트',
        ),
        Node(
            package='pkrc_control',
            executable='gui_server',
            name='gui_server',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
            }],
        ),
    ])
```

- [ ] **Step 7: `setup.py` 수정**

`src/pkrc_control/setup.py`의 `data_files` 리스트를 다음으로 교체한다:

```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 웹 GUI 정적 페이지 — gui_server 가 share 에서 읽는다
        ('share/' + package_name, ['resource/gui.html']),
        ('share/' + package_name + '/launch', ['launch/gui.launch.py']),
    ],
```

`entry_points`의 `console_scripts` 리스트에 다음 한 줄을 추가한다 (알파벳 순서상 `keyboard_control_robust_original` 앞):

```python
             'gui_server = pkrc_control.gui_server:main',
```

- [ ] **Step 8: 빌드 및 접속 확인**

```bash
cd /home/hero/hero_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select pkrc_control 2>&1 | tail -3
source install/setup.bash
ls install/pkrc_control/share/pkrc_control/gui.html
ros2 launch pkrc_control gui.launch.py &
sleep 3
curl -s http://127.0.0.1:8080/ | head -5
curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8080/nonexistent
kill %1
```

Expected: 빌드 성공. `gui.html`이 share에 설치됨. `curl`이 HTML을 반환하고, 없는 경로는 `404`.

---

### Task 4: 영상 중계 — `/stream` MJPEG

**Files:**
- Modify: `src/pkrc_control/pkrc_control/gui_server.py`
- Modify: `src/pkrc_control/resource/gui.html`

**Interfaces:**
- Consumes: Task 3의 `GuiServer`, `mjpeg_frame()`, `BOUNDARY`, `_Handler.do_GET`
- Produces:
  - `GuiServer.get_frame()` → `tuple[bytes | None, float]`: (최신 JPEG 바이트, 수신 시각 epoch초). 프레임이 없으면 `(None, 0.0)`.
  - `GET /stream` → `multipart/x-mixed-replace; boundary=pkrcframe` 무한 응답.
  - `GuiServer.frame_age()` → `float`: 마지막 프레임 이후 경과 초. 프레임이 없으면 `float('inf')`.

**배경 (반드시 지킬 것):** `/image_raw/compressed`는 `BEST_EFFORT, KEEP_LAST, depth=1`로 퍼블리시된다(`camera_node.py:59-63`). 구독 QoS를 이와 맞추지 않으면 **한 프레임도 받지 못한다**. 이 코드베이스는 같은 함정을 `keyboard_control_teleop.py:586-589`(압력)와 `:592-596`(DVL)에서 이미 주석으로 기록했다.

- [ ] **Step 1: 실패하는 테스트 추가**

`src/pkrc_control/test/test_gui_server.py`의 import 줄을 다음으로 교체한다:

```python
from pkrc_control.gui_server import mjpeg_frame, BOUNDARY, FrameStore
```

파일 끝의 `if __name__ == '__main__':` 블록 **앞**에 다음을 추가한다:

```python
def test_frame_store_returns_latest():
    """FrameStore 는 항상 가장 최근 프레임만 준다 (depth=1 의미)."""
    fs = FrameStore()
    assert fs.get() == (None, 0.0)

    fs.put(b'first', 100.0)
    fs.put(b'second', 101.0)
    data, ts = fs.get()
    assert data == b'second'
    assert ts == 101.0


def test_frame_store_age():
    """프레임 없으면 age 가 무한, 있으면 경과 시간."""
    fs = FrameStore()
    assert fs.age(now=200.0) == float('inf')

    fs.put(b'x', 100.0)
    assert fs.age(now=100.5) == 0.5
```

`if __name__ == '__main__':` 블록을 다음으로 교체한다:

```python
if __name__ == '__main__':
    test_mjpeg_frame_structure()
    test_mjpeg_frame_does_not_alter_payload()
    test_frame_store_returns_latest()
    test_frame_store_age()
    print('test_gui_server(frame): 4 passed')
```

- [ ] **Step 2: 테스트를 실행해 실패를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
```

Expected: FAIL — `ImportError: cannot import name 'FrameStore'`

- [ ] **Step 3: `FrameStore` 와 구독 구현**

`gui_server.py`의 import 블록에 추가:

```python
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
```

`mjpeg_frame` 함수 **다음**에 클래스를 추가:

```python
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
```

`GuiServer.__init__`에서 `self.html_path = self._find_html()` **다음**에 삽입:

```python
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
```

`GuiServer` 에 메서드를 추가 (`_find_html` 앞):

```python
    def _image_cb(self, msg: CompressedImage):
        """카메라가 만든 JPEG 바이트를 그대로 저장한다. 디코딩하지 않는다."""
        self.frames.put(bytes(msg.data), time.time())

    def get_frame(self):
        return self.frames.get()

    def frame_age(self) -> float:
        return self.frames.age()
```

- [ ] **Step 4: `/stream` 라우트 구현**

`_Handler.do_GET` 의 `if self.path in ('/', '/index.html'):` 블록 **다음**, `else:` **앞**에 추가:

```python
        elif self.path == '/stream':
            self._stream_mjpeg(node)
```

`_Handler` 에 메서드를 추가 (`do_GET` 앞):

```python
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
```

- [ ] **Step 5: 테스트를 실행해 통과를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
python3 pkrc_control/gui_server.py --selftest
```

Expected: `test_gui_server(frame): 4 passed`, `selftest: MJPEG 프레이밍 OK`

- [ ] **Step 6: `gui.html` 에 영상 표시 추가**

`src/pkrc_control/resource/gui.html` 의 `<body>` 내용을 다음으로 교체:

```html
<h1>PKRC 관제</h1>
<div id="cam-wrap">
  <img id="cam" src="/stream" alt="레이저 카메라">
  <p id="cam-status">영상 수신 대기…</p>
</div>
<style>
  body { font-family: system-ui, sans-serif; margin: 1rem;
         background: #111; color: #eee; }
  #cam { max-width: 100%; border: 1px solid #444; background: #000; }
  #cam-status { color: #f80; }
</style>
<script>
  // 영상이 실제로 오는지 img 이벤트로 판단한다. 검은 화면만 보이면
  // 원인을 알 수 없으므로 상태를 글자로 명시한다.
  const img = document.getElementById('cam');
  const status = document.getElementById('cam-status');
  img.addEventListener('load', () => { status.textContent = ''; });
  img.addEventListener('error', () => {
    status.textContent =
      '영상 없음 — laser_camera_publisher 가 떠 있는지 확인하세요.';
  });
</script>
```

- [ ] **Step 7: 실제 카메라로 영상 확인**

```bash
cd /home/hero/hero_ws && source install/setup.bash
colcon build --packages-select pkrc_control 2>&1 | tail -3
source install/setup.bash

# 카메라 노드 먼저
ros2 launch laser_camera_publisher laser_camera.launch.py &
sleep 5
ros2 topic hz /image_raw/compressed --window 20 &
sleep 5; kill %2

# GUI 서버
ros2 launch pkrc_control gui.launch.py &
sleep 3
# 스트림이 실제 JPEG 를 내는지 확인 (SOI 마커 ff d8 검사)
timeout 3 curl -s http://127.0.0.1:8080/stream | head -c 400 | xxd | head -8
kill %1 %2 2>/dev/null
```

Expected: `ros2 topic hz`가 약 30Hz를 보고한다. `curl` 출력에 `--pkrcframe`, `Content-Type: image/jpeg`, 그리고 JPEG SOI 마커 `ffd8` 가 보인다.

브라우저에서 `http://<젯슨IP>:8080` 을 열어 영상이 실제로 보이는지 눈으로 확인한다. 이건 자동 검증이 불가능한 부분이다.

---

### Task 5: 조종 — `/key` 라우트 + watchdog 자동 정지

**Files:**
- Modify: `src/pkrc_control/pkrc_control/gui_server.py`
- Modify: `src/pkrc_control/resource/gui.html`

**Interfaces:**
- Consumes: Task 2의 `/gui/key` 구독, Task 3의 `GuiServer`/`_Handler`
- Produces:
  - `class KeyWatchdog` — 생성자 `KeyWatchdog(timeout=0.5)`. 메서드:
    - `touch(key: str, now: float) -> None`: 키 수신 기록.
    - `check(now: float) -> str | None`: 정지가 필요하면 `'x'`, 아니면 `None`. 한 번 발동하면 다시 이동 키가 올 때까지 재발동하지 않는다.
  - `MOTION_KEYS: frozenset` — 놓으면 위험한 이동 키 집합: `{'UP', 'DOWN', 'LEFT', 'RIGHT', 'a', 'd', 'w', 's'}`
  - `POST /key` — 본문 `{"key": "UP"}` JSON. 응답 `{"ok": true}`.
  - `GuiServer.publish_key(key: str) -> None`

**배경 (안전 필수):** 브라우저가 방향키 버튼을 누르는 동안 20Hz로 `POST /key`를 반복한다. 노드의 auto-repeat 전제(`key_timeout = 0.40`, `keyboard_control_teleop.py:537`)를 그대로 활용하므로 제어 로직은 바꿀 필요가 없다.

watchdog은 **단순화 대상이 아니다.** 탭을 닫든 와이파이가 끊기든 노트북 배터리가 나가든 로봇이 추력을 물고 달아나면 회수 불가 상황이 된다. 0.5초 무응답이면 `x`(전 축 정지 + 컨트롤러 리셋)를 자동 발행한다.

- [ ] **Step 1: 실패하는 테스트 추가**

`src/pkrc_control/test/test_gui_server.py` 의 import 줄을 교체:

```python
from pkrc_control.gui_server import (
    mjpeg_frame, BOUNDARY, FrameStore, KeyWatchdog, MOTION_KEYS)
```

`if __name__ == '__main__':` 블록 **앞**에 추가:

```python
def test_watchdog_fires_after_timeout():
    """이동 키 후 0.5초 무응답이면 정지 키를 내야 한다."""
    wd = KeyWatchdog(timeout=0.5)
    wd.touch('UP', now=100.0)

    assert wd.check(now=100.3) is None       # 아직 유효
    assert wd.check(now=100.49) is None      # 경계 직전
    assert wd.check(now=100.51) == 'x'       # 발동


def test_watchdog_fires_only_once():
    """한 번 정지시킨 뒤 재발동하지 않는다 (x 폭주 방지)."""
    wd = KeyWatchdog(timeout=0.5)
    wd.touch('UP', now=100.0)
    assert wd.check(now=101.0) == 'x'
    assert wd.check(now=102.0) is None
    assert wd.check(now=200.0) is None


def test_watchdog_rearms_on_new_motion_key():
    """새 이동 키가 오면 다시 감시를 시작한다."""
    wd = KeyWatchdog(timeout=0.5)
    wd.touch('UP', now=100.0)
    assert wd.check(now=101.0) == 'x'

    wd.touch('LEFT', now=102.0)
    assert wd.check(now=102.2) is None
    assert wd.check(now=103.0) == 'x'


def test_watchdog_ignores_nonmotion_keys():
    """r·t·x 같은 1회성 키는 감시 대상이 아니다 — 놓아도 위험하지 않다."""
    wd = KeyWatchdog(timeout=0.5)
    wd.touch('r', now=100.0)
    assert wd.check(now=101.0) is None

    wd.touch('x', now=100.0)
    assert wd.check(now=101.0) is None


def test_motion_keys_content():
    """이동 키 집합이 노드의 위험 키와 일치하는지."""
    assert MOTION_KEYS == frozenset(
        {'UP', 'DOWN', 'LEFT', 'RIGHT', 'a', 'd', 'w', 's'})
```

`if __name__ == '__main__':` 블록을 교체:

```python
if __name__ == '__main__':
    test_mjpeg_frame_structure()
    test_mjpeg_frame_does_not_alter_payload()
    test_frame_store_returns_latest()
    test_frame_store_age()
    test_watchdog_fires_after_timeout()
    test_watchdog_fires_only_once()
    test_watchdog_rearms_on_new_motion_key()
    test_watchdog_ignores_nonmotion_keys()
    test_motion_keys_content()
    print('test_gui_server: 9 passed')
```

- [ ] **Step 2: 테스트를 실행해 실패를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
```

Expected: FAIL — `ImportError: cannot import name 'KeyWatchdog'`

- [ ] **Step 3: `KeyWatchdog` 구현**

`gui_server.py` 의 import 블록에 추가:

```python
from std_msgs.msg import String
```

`BOUNDARY = 'pkrcframe'` **다음**에 추가:

```python
# 누르고 있는 동안만 유효한 이동 키. 이 키를 보낸 뒤 브라우저가 조용해지면
# 통신이 끊긴 것으로 보고 강제 정지시킨다. r/t/x/c/q 는 1회성이라 제외.
MOTION_KEYS = frozenset({'UP', 'DOWN', 'LEFT', 'RIGHT', 'a', 'd', 'w', 's'})

# 이 시간 동안 이동 키가 갱신되지 않으면 정지시킨다 [초].
KEY_TIMEOUT = 0.5

# 전 축 정지 + 컨트롤러 리셋 키 (두 조종 노드 공통).
STOP_KEY = 'x'
```

`FrameStore` 클래스 **다음**에 추가:

```python
class KeyWatchdog:
    """브라우저와의 통신이 끊기면 로봇을 정지시킨다.

    탭을 닫거나 와이파이가 끊기면 마지막 이동 명령이 노드에 남아 계속
    추력을 낸다. 물속 로봇이 명령 없이 밀고 나가는 것은 회수 불가
    상황이므로, 무응답을 감지해 정지 키를 강제 발행한다.

    노드 자체에도 key_timeout 0.4초 decay 가 있지만(keyboard_control_teleop.py:537)
    그것은 추력을 0 으로 줄일 뿐 적분기와 heading target 을 정리하지
    않는다. x 는 컨트롤러까지 리셋한다.
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
```

- [ ] **Step 4: `GuiServer` 에 키 발행과 watchdog 타이머 추가**

`GuiServer.__init__` 의 카메라 구독 블록 **다음**에 삽입:

```python
        # ── 조종 키 ────────────────────────────────────────────────────
        # 두 조종 노드가 /gui/key 를 구독한다. stdin(터미널) 경로와
        # 병행 동작하므로 둘 중 아무거나 써도 된다.
        self.key_pub = self.create_publisher(String, '/gui/key', 10)
        self.watchdog = KeyWatchdog()
        # 10Hz 로 통신 생존 확인 — 끊기면 강제 정지
        self.create_timer(0.1, self._watchdog_tick)
```

`GuiServer` 에 메서드 추가 (`_find_html` 앞):

```python
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
```

- [ ] **Step 5: `POST /key` 라우트 구현**

`_Handler` 에 메서드 추가 (`do_GET` 다음):

```python
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
            node.publish_key(key)
            self._send(200, 'application/json',
                       json.dumps({'ok': True}).encode())
        else:
            self._send(404, 'application/json',
                       json.dumps({'ok': False,
                                   'error': 'not found'}).encode())
```

- [ ] **Step 6: `_selftest()` 에 watchdog 검증 추가**

`_selftest()` 함수의 `print('selftest: MJPEG 프레이밍 OK')` **다음**에 추가:

```python
    wd = KeyWatchdog(timeout=0.5)
    wd.touch('UP', now=100.0)
    assert wd.check(now=100.3) is None, 'watchdog 조기 발동'
    assert wd.check(now=100.6) == STOP_KEY, 'watchdog 미발동'
    assert wd.check(now=101.0) is None, 'watchdog 중복 발동'
    wd.touch('r', now=200.0)
    assert wd.check(now=201.0) is None, '비이동 키에 발동'
    print('selftest: watchdog OK')
```

- [ ] **Step 7: 테스트를 실행해 통과를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
python3 pkrc_control/gui_server.py --selftest
```

Expected: `test_gui_server: 9 passed`, 그리고 `selftest: MJPEG 프레이밍 OK` + `selftest: watchdog OK`

- [ ] **Step 8: `gui.html` 에 조종 버튼 추가**

`gui.html` 의 `<div id="cam-wrap">` 블록 **다음**에 추가:

```html
<h2>조종</h2>
<div id="pad">
  <div class="row">
    <button data-key="LEFT" data-hold="1">← 좌</button>
    <button data-key="UP" data-hold="1">↑ 전진</button>
    <button data-key="RIGHT" data-hold="1">우 →</button>
  </div>
  <div class="row">
    <button data-key="a" data-hold="1">A 우회전</button>
    <button data-key="DOWN" data-hold="1">↓ 후진</button>
    <button data-key="d" data-hold="1">D 좌회전</button>
  </div>
  <div class="row">
    <button data-key="w">W 상승 10cm</button>
    <button data-key="s">S 하강 10cm</button>
  </div>
  <div class="row">
    <button data-key="r">R 방위 리셋</button>
    <button data-key="t">T 수심 리셋</button>
    <button data-key="c">C 벽 정렬</button>
    <button data-key="x" class="stop">X 정지</button>
  </div>
</div>
```

`<style>` 블록 안에 추가:

```css
  #pad .row { display: flex; gap: .5rem; margin-bottom: .5rem;
              flex-wrap: wrap; }
  #pad button { padding: .8rem 1rem; font-size: 1rem; cursor: pointer;
                background: #333; color: #eee; border: 1px solid #555;
                border-radius: 4px; min-width: 6rem; }
  #pad button:active { background: #06c; }
  #pad button.stop { background: #833; }
```

`<script>` 블록 안에 추가:

```javascript
  // 누르고 있는 동안 20Hz 로 키를 반복 전송한다. 노드가 0.4초 auto-repeat
  // 타임아웃을 전제로 만들어져 있어(key_timeout) 이 방식이 그대로 맞는다.
  // 떼면 전송이 멈추고 노드가 자연 decay 한다.
  const HOLD_HZ = 20;
  let holdTimer = null;

  function sendKey(key) {
    fetch('/key', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({key: key}),
    }).catch(() => {});   // 통신 실패는 서버 watchdog 이 처리한다
  }

  function startHold(key) {
    stopHold();
    sendKey(key);
    holdTimer = setInterval(() => sendKey(key), 1000 / HOLD_HZ);
  }

  function stopHold() {
    if (holdTimer !== null) { clearInterval(holdTimer); holdTimer = null; }
  }

  document.querySelectorAll('#pad button').forEach(btn => {
    const key = btn.dataset.key;
    if (btn.dataset.hold) {
      // 누르는 동안 반복. pointer 이벤트로 마우스·터치를 함께 처리.
      btn.addEventListener('pointerdown', e => {
        e.preventDefault(); startHold(key);
      });
      ['pointerup', 'pointerleave', 'pointercancel'].forEach(ev =>
        btn.addEventListener(ev, stopHold));
    } else {
      btn.addEventListener('click', () => sendKey(key));
    }
  });

  // 탭이 숨겨지면 즉시 전송을 멈춘다 — 서버 watchdog 이 0.5초 후 정지시킨다.
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) stopHold();
  });
  window.addEventListener('blur', stopHold);
```

- [ ] **Step 9: 실기 확인 — 키 전달과 watchdog 발동**

터미널 A:
```bash
cd /home/hero/hero_ws && source install/setup.bash
colcon build --packages-select pkrc_control 2>&1 | tail -3
source install/setup.bash
ros2 run pkrc_control keyboard_control_teleop
```

터미널 B:
```bash
cd /home/hero/hero_ws && source install/setup.bash
ros2 launch pkrc_control gui.launch.py
```

터미널 C:
```bash
cd /home/hero/hero_ws && source install/setup.bash
# 키 전달 확인
curl -s -X POST http://127.0.0.1:8080/key \
  -H 'Content-Type: application/json' -d '{"key":"x"}'
echo
# watchdog 발동 확인: 이동 키 한 번 보내고 조용히 기다린다
curl -s -X POST http://127.0.0.1:8080/key \
  -H 'Content-Type: application/json' -d '{"key":"UP"}'
echo "UP 전송 — 0.5초 후 자동 정지 로그를 기다립니다"
sleep 2
```

Expected:
- `{"ok": true}` 응답.
- 터미널 A에 `STOP (all axes + controllers reset)` 로그 (x 전달 확인).
- 터미널 B에 `브라우저 무응답 0.5초 — 강제 정지(x) 발행` 경고 (watchdog 발동 확인).
- 터미널 A에 다시 `STOP` 로그.

브라우저에서 방향키 버튼을 누른 채 탭을 닫아 watchdog이 실제로 로봇을 세우는지 확인한다. **이 검증을 건너뛰지 말 것** — 이 기능이 동작하지 않으면 수조에서 로봇을 잃을 수 있다.

---

### Task 6: 텔레메트리 — `/state` + 모니터링 UI

**Files:**
- Modify: `src/pkrc_control/pkrc_control/gui_server.py`
- Modify: `src/pkrc_control/resource/gui.html`

**Interfaces:**
- Consumes: Task 3~5의 `GuiServer`, `_Handler`
- Produces:
  - `GET /state` → JSON. 최상위 키: `camera` (`{"alive": bool, "age": float}`), `yaw` (12개 배열 또는 `null`), `depth` (6개 배열 또는 `null`), `thrusters` (6개 배열 또는 `null`), `wall` (11개 배열 또는 `null`), `wall_mode` (문자열 또는 `null`), `nodes` (Task 7에서 채움, 지금은 `{}`).
  - `GuiServer.state_snapshot() -> dict`: 위 JSON의 파이썬 dict.

**배경:** 노드들이 이미 필요한 값을 전부 퍼블리시한다. 새로 계산할 것이 없다.

| 토픽 | 개수 | 근거 |
|:---|:---|:---|
| `/teleop/yaw_debug` | 12 | `keyboard_control_teleop.py:1106-1119` |
| `/teleop/depth_debug` | 6 | `:1121-1128` |
| `/teleop/thruster_currents` | 6 | `:1100` |
| `/teleop/wall_debug` | 11 | `keyboard_control_wall_align.py:1677-1689` |
| `/teleop/wall_mode` | String | `:788-789` |

`yaw_debug` 12개 순서: `[target_yaw_deg, current_yaw_deg, yaw_err_deg, yaw_rate_deg, yaw_cmd, heave_cmd, pitch_deg, is_yawing, dvl_vx, dvl_vy, drift_fb_surge, drift_fb_sway]`

`depth_debug` 6개 순서: `[target_depth, current_depth, depth_err, heave_cmd, est_velocity, pressure_pa]`

`wall_debug` 11개 순서: `[target_dist, sonar_dist, dist_err, surge_cmd, sonar_conf, sonar_alive, scan_accum_deg, scan_best_dist, scan_best_yaw_deg, scan_samples, sonar_raw]`

- [ ] **Step 1: 실패하는 테스트 추가**

`test/test_gui_server.py` 의 import 줄을 교체:

```python
from pkrc_control.gui_server import (
    mjpeg_frame, BOUNDARY, FrameStore, KeyWatchdog, MOTION_KEYS,
    TopicCache)
```

`if __name__ == '__main__':` **앞**에 추가:

```python
def test_topic_cache_stale():
    """오래된 값은 None 으로 나와야 한다 — 죽은 노드의 마지막 값을
    살아있는 것처럼 보여주면 안 된다."""
    tc = TopicCache(stale_sec=1.0)
    assert tc.get(now=100.0) is None

    tc.put([1.0, 2.0], now=100.0)
    assert tc.get(now=100.5) == [1.0, 2.0]
    assert tc.get(now=101.5) is None      # 1초 초과 → 죽은 것으로 간주


def test_topic_cache_overwrites():
    """항상 최신 값만."""
    tc = TopicCache(stale_sec=1.0)
    tc.put([1.0], now=100.0)
    tc.put([2.0], now=100.1)
    assert tc.get(now=100.2) == [2.0]
```

`if __name__ == '__main__':` 블록에 두 줄 추가하고 개수를 11로 바꾼다:

```python
    test_topic_cache_stale()
    test_topic_cache_overwrites()
    print('test_gui_server: 11 passed')
```

- [ ] **Step 2: 테스트를 실행해 실패를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
```

Expected: FAIL — `ImportError: cannot import name 'TopicCache'`

- [ ] **Step 3: `TopicCache` 구현**

`gui_server.py` 의 import 블록에 추가:

```python
from std_msgs.msg import Float64MultiArray
```

`KeyWatchdog` 클래스 **다음**에 추가:

```python
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
```

- [ ] **Step 4: 텔레메트리 구독과 `/state` 스냅샷 구현**

`GuiServer.__init__` 의 조종 키 블록 **다음**에 삽입:

```python
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
```

`GuiServer` 에 메서드 추가:

```python
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
```

`_Handler.do_GET` 의 `elif self.path == '/stream':` 블록 **다음**에 추가:

```python
        elif self.path == '/state':
            self._send(200, 'application/json',
                       json.dumps(node.state_snapshot()).encode())
```

- [ ] **Step 5: 테스트를 실행해 통과를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
python3 pkrc_control/gui_server.py --selftest
```

Expected: `test_gui_server: 11 passed`

- [ ] **Step 6: `gui.html` 에 모니터링 UI 추가**

`gui.html` 의 `<div id="pad">` 블록 **다음**에 추가:

```html
<h2>상태</h2>
<div id="mon">
  <table id="nums">
    <tr><th>방위 목표</th><td id="v-yaw-t">–</td>
        <th>방위 현재</th><td id="v-yaw-c">–</td>
        <th>오차</th><td id="v-yaw-e">–</td></tr>
    <tr><th>수심 목표</th><td id="v-dep-t">–</td>
        <th>수심 현재</th><td id="v-dep-c">–</td>
        <th>압력</th><td id="v-pres">–</td></tr>
    <tr><th>DVL surge</th><td id="v-dvl-x">–</td>
        <th>DVL sway</th><td id="v-dvl-y">–</td>
        <th>pitch</th><td id="v-pitch">–</td></tr>
    <tr><th>소나 거리</th><td id="v-sonar">–</td>
        <th>소나 신뢰도</th><td id="v-conf">–</td>
        <th>벽정렬 상태</th><td id="v-wmode">–</td></tr>
  </table>

  <h3>스러스터 전류</h3>
  <div id="thrusters"></div>

  <h3>추이 (최근 30초)</h3>
  <canvas id="chart" width="600" height="160"></canvas>
  <p class="legend"><span style="color:#4af">■</span> 방위 오차 [°]
     <span style="color:#fa4">■</span> 수심 오차 [m]</p>
</div>
```

`<style>` 블록에 추가:

```css
  #nums { border-collapse: collapse; margin-bottom: 1rem; }
  #nums th, #nums td { border: 1px solid #444; padding: .3rem .6rem;
                       text-align: right; font-variant-numeric: tabular-nums; }
  #nums th { background: #222; color: #aaa; font-weight: normal; }
  .tbar { display: flex; align-items: center; gap: .5rem;
          margin-bottom: .2rem; }
  .tbar .name { width: 2.5rem; color: #aaa; }
  .tbar .track { position: relative; width: 260px; height: 14px;
                 background: #222; border: 1px solid #444; }
  .tbar .fill { position: absolute; top: 0; bottom: 0; left: 50%;
                background: #4af; }
  .tbar .val { width: 4rem; font-variant-numeric: tabular-nums; }
  .tbar .limit { position: absolute; top: -2px; bottom: -2px;
                 width: 1px; background: #f44; }
  #chart { background: #1a1a1a; border: 1px solid #444; max-width: 100%; }
  .legend { color: #888; font-size: .9rem; }
  .dead { color: #f55 !important; }
```

`<script>` 블록에 추가:

```javascript
  // ── 상태 폴링 (10Hz) ────────────────────────────────────────────
  // WebSocket 을 쓰지 않는 이유: 10Hz 폴링에 과하고 의존성이 늘어난다.
  const T_LIMITS = [3.0, 3.0, 3.0, 3.0, 5.0, 5.0];  // max_current_* 기본값
  const hist = {yaw: [], depth: []};
  const HIST_MAX = 300;   // 30초 × 10Hz

  function fmt(v, digits, unit) {
    if (v === null || v === undefined) return '–';
    return v.toFixed(digits) + (unit || '');
  }

  function setText(id, text, dead) {
    const el = document.getElementById(id);
    el.textContent = text;
    el.classList.toggle('dead', !!dead);
  }

  function buildThrusters() {
    const box = document.getElementById('thrusters');
    box.innerHTML = '';
    for (let i = 0; i < 6; i++) {
      const row = document.createElement('div');
      row.className = 'tbar';
      row.innerHTML =
        `<span class="name">T${i + 1}</span>` +
        `<span class="track"><span class="fill" id="tf${i}"></span></span>` +
        `<span class="val" id="tv${i}">–</span>`;
      box.appendChild(row);
    }
  }
  buildThrusters();

  function drawThrusters(cur) {
    for (let i = 0; i < 6; i++) {
      const fill = document.getElementById('tf' + i);
      const val = document.getElementById('tv' + i);
      if (!cur) { fill.style.width = '0'; val.textContent = '–'; continue; }
      const a = cur[i] || 0;
      // 중앙 기준 좌우 확장. 한계 대비 비율로 폭을 정해 포화가 눈에 보인다.
      const frac = Math.min(Math.abs(a) / T_LIMITS[i], 1) * 50;
      fill.style.width = frac + '%';
      fill.style.left = a >= 0 ? '50%' : (50 - frac) + '%';
      fill.style.background = Math.abs(a) >= T_LIMITS[i] * 0.98
        ? '#f44' : '#4af';
      val.textContent = a.toFixed(2) + 'A';
    }
  }

  function drawChart() {
    const c = document.getElementById('chart');
    const g = c.getContext('2d');
    g.clearRect(0, 0, c.width, c.height);

    // 0 기준선
    g.strokeStyle = '#444';
    g.beginPath();
    g.moveTo(0, c.height / 2);
    g.lineTo(c.width, c.height / 2);
    g.stroke();

    const series = [
      {data: hist.yaw, color: '#4af', scale: 30},   // ±30° 를 화면 절반
      {data: hist.depth, color: '#fa4', scale: 0.5}, // ±0.5m
    ];
    for (const s of series) {
      if (s.data.length < 2) continue;
      g.strokeStyle = s.color;
      g.beginPath();
      s.data.forEach((v, i) => {
        const x = (i / (HIST_MAX - 1)) * c.width;
        const y = c.height / 2
                - Math.max(-1, Math.min(1, v / s.scale)) * (c.height / 2 - 4);
        i === 0 ? g.moveTo(x, y) : g.lineTo(x, y);
      });
      g.stroke();
    }
  }

  function push(arr, v) {
    arr.push(v);
    while (arr.length > HIST_MAX) arr.shift();
  }

  async function poll() {
    let s;
    try {
      s = await (await fetch('/state')).json();
    } catch (e) {
      setText('cam-status', '서버 연결 끊김', true);
      return;
    }

    // 카메라 생존
    if (!s.camera.alive) {
      status.textContent = s.camera.age === null
        ? '영상 없음 — laser_camera_publisher 가 떠 있는지 확인하세요.'
        : `영상 정지 ${s.camera.age}초 — 카메라 노드를 확인하세요.`;
      status.classList.add('dead');
    } else {
      status.textContent = '';
      status.classList.remove('dead');
    }

    const y = s.yaw, d = s.depth, w = s.wall;
    setText('v-yaw-t', y ? fmt(y[0], 1, '°') : '–', !y);
    setText('v-yaw-c', y ? fmt(y[1], 1, '°') : '–', !y);
    setText('v-yaw-e', y ? fmt(y[2], 1, '°') : '–', !y);
    setText('v-dep-t', d ? fmt(d[0], 2, 'm') : '–', !d);
    setText('v-dep-c', d ? fmt(d[1], 2, 'm') : '–', !d);
    setText('v-pres', d ? fmt(d[5] / 1000, 1, 'kPa') : '–', !d);
    setText('v-dvl-x', y ? fmt(y[8], 3, 'm/s') : '–', !y);
    setText('v-dvl-y', y ? fmt(y[9], 3, 'm/s') : '–', !y);
    setText('v-pitch', y ? fmt(y[6], 1, '°') : '–', !y);
    setText('v-sonar', w ? fmt(w[1], 2, 'm') : '–', !w);
    setText('v-conf', w ? fmt(w[4], 0, '%') : '–', !w);
    setText('v-wmode', s.wall_mode || '–', !s.wall_mode);

    drawThrusters(s.thrusters);
    push(hist.yaw, y ? y[2] : 0);
    push(hist.depth, d ? d[2] : 0);
    drawChart();
  }

  setInterval(poll, 100);
  poll();
```

- [ ] **Step 7: 실기 확인**

```bash
cd /home/hero/hero_ws && source install/setup.bash
colcon build --packages-select pkrc_control 2>&1 | tail -3
source install/setup.bash

ros2 run pkrc_control keyboard_control_teleop &
sleep 3
ros2 launch pkrc_control gui.launch.py &
sleep 3
curl -s http://127.0.0.1:8080/state | python3 -m json.tool
kill %1 %2
```

Expected: JSON에 `camera`, `yaw`, `depth`, `thrusters`, `wall`, `wall_mode`, `nodes` 키가 모두 있다. teleop이 떠 있으므로 `yaw`·`depth`·`thrusters`는 배열이고, wall_align은 안 떠 있으므로 `wall`과 `wall_mode`는 `null`이다.

브라우저에서 수치가 갱신되고 전류 바·그래프가 움직이는지 확인한다.

---

### Task 7: 노드 관리 — 토글, 인터록, 모드 전환, 게인 튜닝, 프리셋

**Files:**
- Modify: `src/pkrc_control/pkrc_control/gui_server.py`
- Modify: `src/pkrc_control/resource/gui.html`

**Interfaces:**
- Consumes: Task 3~6 전체
- Produces:
  - `NODE_SPECS: dict[str, dict]` — 각 항목 `{'label': str, 'cmd': list[str], 'group': str | None}`. `group`이 같은 항목은 상호배타다.
  - `class ProcManager` — 메서드:
    - `start(key: str) -> tuple[bool, str]`: (성공, 메시지). 같은 그룹의 다른 노드를 먼저 내린다.
    - `stop(key: str) -> tuple[bool, str]`
    - `status() -> dict[str, bool]`: 키 → 실행 중 여부.
    - `stop_all() -> None`
  - `POST /node` — 본문 `{"key": "sonar", "on": true}`. 응답 `{"ok": bool, "msg": str, "nodes": {...}}`.
  - `POST /param` — 본문 `{"node": "keyboard_teleop_robust", "name": "hh_rate_kp", "value": 2.5}`. 응답 `{"ok": bool, "msg": str}`.
  - `GET /params?node=<노드명>` — 응답 `{"ok": bool, "params": [{"name": str, "value": float, "type": str}], "msg": str}`.
  - `POST /preset` — 본문 `{"action": "save"|"load"|"list", "name": str, "node": str}`.

**배경:**

프로세스 종료는 `live_tuning.py:158-207`의 `_descendants()` + `_terminate()`를 재사용한다. `ros2 launch`는 래퍼이고 실제 자식이 별도 프로세스 그룹으로 빠져나가므로 단순 `kill`이 닿지 않는다. 이 코드베이스는 이미 그 문제를 풀어놨으니 다시 구현하지 않는다.

**인터록이 필요한 이유** (`v4l2-ctl --info`로 확인한 사실):

| 그룹 | 다투는 대상 | 이유 |
|:---|:---|:---|
| `video4` | Localization ↔ ArUco 단독 | 둘 다 `/dev/video4`(stellarHD)를 열고, `localization.launch.py:62`가 내부에 `aruco_detector_6dof`를 포함 |
| `control` | teleop ↔ wall_align | 같은 CAN 버스, 같은 VESC ID(`0x151`~`0x156`) |

`control` 그룹 전환은 순서가 중요하다. 정지 없이 죽이면 VESC가 마지막 전류 명령을 유지한다. `x` 발행 → 0.3초 대기 → 종료 → 새 노드 실행.

**레이저 카메라는 토글이 아니다.** `/image_raw/compressed`를 내는 유일한 노드이므로 시작 시 확보한다. 이미 떠 있으면 그것을 쓰고 소유하지 않는다(gui_server 종료 시에도 살려둔다). 없으면 띄우고 소유한다.

**파라미터는 하드코딩하지 않는다.** `live_tuning.py`의 `_ROUTES`가 SSOT이고 노드마다 선언 집합이 다르다(teleop 약 40개 vs wall_align 약 60개). `list_parameters` → `describe_parameters` → `get_parameters` 서비스로 읽고, `read_only`인 것(`live_tuning.py:30-32`의 `_FIXED`)은 제외한다.

- [ ] **Step 1: 실패하는 테스트 추가**

`test/test_gui_server.py` 의 import 줄을 교체:

```python
from pkrc_control.gui_server import (
    mjpeg_frame, BOUNDARY, FrameStore, KeyWatchdog, MOTION_KEYS,
    TopicCache, NODE_SPECS, group_siblings, CONTROL_NODE_NAMES)
```

`if __name__ == '__main__':` **앞**에 추가:

```python
def test_node_specs_shape():
    """모든 노드 스펙이 필수 키를 갖는지."""
    for key, spec in NODE_SPECS.items():
        assert 'label' in spec, f'{key}: label 없음'
        assert 'cmd' in spec, f'{key}: cmd 없음'
        assert isinstance(spec['cmd'], list), f'{key}: cmd 가 리스트 아님'
        assert spec['cmd'], f'{key}: cmd 가 비었음'
        assert 'group' in spec, f'{key}: group 없음'


def test_excluded_cameras_absent():
    """exploreHD gscam 과 stellarHD usb_cam 은 UI 에 없어야 한다 —
    각각 레이저 카메라(/dev/video0)와 ArUco(/dev/video4)를 다툰다."""
    joined = ' '.join(
        ' '.join(s['cmd']) for s in NODE_SPECS.values()).lower()
    assert 'gscam' not in joined
    assert 'usb_cam' not in joined
    assert 'full_system' not in joined


def test_group_siblings_video4():
    """video4 그룹은 localization 과 aruco 가 서로 형제다."""
    assert group_siblings('localization') == ['aruco']
    assert group_siblings('aruco') == ['localization']


def test_group_siblings_control():
    """control 그룹은 teleop 과 wall_align 이 서로 형제다."""
    assert group_siblings('teleop') == ['wall_align']
    assert group_siblings('wall_align') == ['teleop']


def test_group_siblings_none_for_ungrouped():
    """그룹 없는 센서는 형제가 없다 — 자유롭게 켜고 끈다."""
    assert group_siblings('sonar') == []
    assert group_siblings('imu') == []


def test_control_group_has_exactly_two():
    """조종 노드는 정확히 둘이어야 한다. 셋이 되면 CAN 충돌 위험이
    커지므로 인터록 설계를 다시 봐야 한다."""
    ctrl = [k for k, s in NODE_SPECS.items() if s['group'] == 'control']
    assert sorted(ctrl) == ['teleop', 'wall_align']


def test_control_node_names_match_source():
    """CONTROL_NODE_NAMES 가 실제 super().__init__() 인수와 일치하는지.

    이름이 틀리면 파라미터 서비스 경로(/노드명/set_parameters)가 존재하지
    않아 게인 튜닝이 조용히 전부 실패한다. 소스에서 직접 읽어 비교한다.
    """
    import re

    src_dir = os.path.join(os.path.dirname(__file__), '..', 'pkrc_control')
    files = {
        'teleop': 'keyboard_control_teleop.py',
        'wall_align': 'keyboard_control_wall_align.py',
    }
    for key, fname in files.items():
        with open(os.path.join(src_dir, fname)) as f:
            m = re.search(r"super\(\)\.__init__\('([^']+)'\)", f.read())
        assert m, f'{fname}: super().__init__() 를 찾을 수 없음'
        actual = m.group(1)
        assert CONTROL_NODE_NAMES[key] == actual, (
            f'{key}: CONTROL_NODE_NAMES 는 '
            f'{CONTROL_NODE_NAMES[key]!r} 인데 소스는 {actual!r}')
```

`if __name__ == '__main__':` 블록에 6줄 추가하고 개수를 17로 바꾼다:

```python
    test_node_specs_shape()
    test_excluded_cameras_absent()
    test_group_siblings_video4()
    test_group_siblings_control()
    test_group_siblings_none_for_ungrouped()
    test_control_group_has_exactly_two()
    test_control_node_names_match_source()
    print('test_gui_server: 18 passed')
```

- [ ] **Step 2: 테스트를 실행해 실패를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
```

Expected: FAIL — `ImportError: cannot import name 'NODE_SPECS'`

- [ ] **Step 3: `NODE_SPECS` 와 `group_siblings` 구현**

`gui_server.py` 의 import 블록에 추가:

```python
import subprocess

import yaml
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import (DescribeParameters, GetParameters,
                               ListParameters, SetParameters)

# 프로세스 종료는 이 패키지가 이미 풀어놓은 문제다 — ros2 launch 는 래퍼이고
# 실제 자식이 별도 프로세스 그룹으로 빠져나가므로 단순 kill 이 닿지 않는다.
from pkrc_control.live_tuning import _terminate
```

`STOP_KEY = 'x'` **다음**에 추가:

```python
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
```

- [ ] **Step 4: 테스트를 실행해 통과를 확인**

```bash
cd /home/hero/hero_ws/src/pkrc_control && python3 test/test_gui_server.py
```

Expected: `test_gui_server: 18 passed`

`ModuleNotFoundError: No module named 'rcl_interfaces'` 가 나오면 ROS를 먼저 소싱한다: `source /opt/ros/humble/setup.bash`

`test_control_node_names_match_source` 가 실패하면 `CONTROL_NODE_NAMES` 의 값을 소스의 `super().__init__()` 인수와 맞춘다 — 이 이름이 틀리면 게인 튜닝이 조용히 전부 실패한다.

- [ ] **Step 5: `ProcManager` 구현**

`TopicCache` 클래스 **다음**에 추가:

```python
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
```

- [ ] **Step 6: `GuiServer` 에 프로세스·파라미터·프리셋 기능 추가**

`GuiServer.__init__` 의 텔레메트리 블록 **다음**에 삽입:

```python
        # ── 노드 프로세스 관리 ─────────────────────────────────────────
        self.procs = ProcManager(self.get_logger(),
                                 on_before_stop=self._emit_stop)
        # 파라미터 서비스 클라이언트 캐시 (노드명 → {서비스명: client})
        self._param_clients = {}
        os.makedirs(PRESET_DIR, exist_ok=True)

        # 레이저 카메라 확보 — 이미 돌면 그걸 쓰고, 없으면 띄우고 소유한다.
        self._owns_camera = False
        self.create_timer(1.0, self._ensure_camera_once)
```

`GuiServer` 에 메서드를 추가:

```python
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
```

`state_snapshot()` 의 `'nodes': {},` 를 다음으로 교체:

```python
            'nodes': self.procs.status(),
```

`shutdown()` 을 다음으로 교체:

```python
    def shutdown(self):
        self._httpd.shutdown()
        self._httpd.server_close()
        self.procs.stop_all()
        # 내가 띄운 카메라만 정리한다 — 남이 띄운 것은 건드리지 않는다.
        if self._owns_camera and getattr(self, '_camera_proc', None):
            _terminate(self._camera_proc)
            self.get_logger().info('레이저 카메라 정리 완료')
```

- [ ] **Step 7: 라우트 추가**

`_Handler.do_GET` 의 `elif self.path == '/state':` 블록 **다음**에 추가:

```python
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
```

`_Handler.do_POST` 의 `if self.path == '/key':` 블록 **다음**, `else:` **앞**에 추가:

```python
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
```

- [ ] **Step 8: `_selftest()` 에 인터록 검증 추가**

`_selftest()` 의 `print('selftest: watchdog OK')` **다음**에 추가:

```python
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
```

- [ ] **Step 9: 테스트와 selftest 실행**

```bash
cd /home/hero/hero_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/pkrc_control
python3 test/test_gui_server.py
python3 test/test_gui_key.py
python3 pkrc_control/gui_server.py --selftest
```

Expected:
```
test_gui_server: 18 passed
test_gui_key: 3 passed
selftest: MJPEG 프레이밍 OK
selftest: watchdog OK
selftest: 인터록 OK
selftest: ProcManager OK
```

- [ ] **Step 10: `gui.html` 에 노드 토글·게인·프리셋 UI 추가**

`gui.html` 의 `<h2>조종</h2>` **앞**에 추가:

```html
<h2>노드</h2>
<div id="nodes"></div>

<h2>게인 튜닝</h2>
<div id="tune">
  <p id="tune-status">조종 노드를 켜면 게인 슬라이더가 나타납니다.</p>
  <div id="sliders"></div>
  <div id="preset-box">
    <input id="preset-name" placeholder="프리셋 이름" size="16">
    <button id="preset-save">저장</button>
    <select id="preset-sel"></select>
    <button id="preset-load">불러오기</button>
    <span id="preset-msg"></span>
  </div>
</div>
```

`<style>` 블록에 추가:

```css
  .nodegrp { border: 1px solid #444; padding: .5rem; margin-bottom: .5rem; }
  .nodegrp legend { color: #aaa; font-size: .9rem; }
  .nodebtn { padding: .5rem .8rem; margin: .2rem; cursor: pointer;
             background: #333; color: #eee; border: 1px solid #555;
             border-radius: 4px; }
  .nodebtn.on { background: #060; border-color: #0a0; }
  .slider { display: flex; align-items: center; gap: .5rem;
            margin-bottom: .2rem; }
  .slider label { width: 12rem; font-size: .85rem; color: #bbb;
                  text-align: right; }
  .slider input[type=range] { width: 200px; }
  .slider input[type=number] { width: 5.5rem; background: #222;
                               color: #eee; border: 1px solid #555; }
  #preset-box { margin-top: .8rem; display: flex; gap: .4rem;
                align-items: center; flex-wrap: wrap; }
  #preset-box input, #preset-box select, #preset-box button {
    background: #222; color: #eee; border: 1px solid #555; padding: .3rem; }
  #preset-msg, #tune-status { color: #8a8; font-size: .9rem; }
  .msg-err { color: #f77 !important; }
```

`<script>` 블록 끝에 추가:

```javascript
  // ── 노드 토글 ───────────────────────────────────────────────────
  // 그룹이 같은 노드는 서버가 상호배타를 강제한다. UI 도 같은 그룹을
  // 한 상자에 묶어 사용자가 충돌 조합을 만들 수 없다는 걸 보이게 한다.
  const NODE_GROUPS = [
    {title: '센서 (자유롭게 켜고 끄기)',
     keys: ['pressure', 'dvl', 'imu', 'led', 'sonar']},
    {title: '측위 — 하나만 (stellarHD 카메라 공유)',
     keys: ['localization', 'aruco']},
    {title: '조종 — 하나만 (CAN 버스 공유)',
     keys: ['teleop', 'wall_align']},
  ];
  const NODE_LABELS = {
    pressure: '압력 (Bar10XT)', dvl: 'DVL-A50', imu: 'IMU (GV7-INS)',
    led: 'Lumen LED', sonar: 'Ping1D 소나',
    localization: '측위 (UKFM + ArUco)', aruco: 'ArUco 단독',
    teleop: '수동 조종 (teleop)', wall_align: '벽면 정렬 (wall_align)',
  };
  // 서버의 CONTROL_NODE_NAMES 와 정확히 일치해야 한다.
  const CONTROL_NODE_NAMES = {
    teleop: 'keyboard_teleop_robust',
    wall_align: 'keyboard_teleop_wall_align',
  };
  let nodeState = {};

  function buildNodes() {
    const box = document.getElementById('nodes');
    box.innerHTML = '';
    for (const g of NODE_GROUPS) {
      const fs = document.createElement('fieldset');
      fs.className = 'nodegrp';
      fs.innerHTML = `<legend>${g.title}</legend>`;
      for (const k of g.keys) {
        const b = document.createElement('button');
        b.className = 'nodebtn';
        b.id = 'nb-' + k;
        b.textContent = NODE_LABELS[k];
        b.addEventListener('click', () => toggleNode(k));
        fs.appendChild(b);
      }
      box.appendChild(fs);
    }
  }
  buildNodes();

  async function toggleNode(key) {
    const btn = document.getElementById('nb-' + key);
    btn.disabled = true;
    try {
      const r = await (await fetch('/node', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({key: key, on: !nodeState[key]}),
      })).json();
      if (!r.ok) alert(r.msg);
      nodeState = r.nodes || nodeState;
      paintNodes();
      refreshTune();
    } finally {
      btn.disabled = false;
    }
  }

  function paintNodes() {
    for (const k in NODE_LABELS) {
      const b = document.getElementById('nb-' + k);
      if (b) b.classList.toggle('on', !!nodeState[k]);
    }
  }

  // ── 게인 튜닝 ───────────────────────────────────────────────────
  // 파라미터 목록을 하드코딩하지 않는다 — 서버가 노드에서 직접 읽어온다.
  // 노드마다 선언 집합이 다르고(teleop 약 40, wall_align 약 60),
  // read_only 로 표시된 물성값은 서버가 이미 걸러낸다.
  let tuneNode = null;

  function activeControlNode() {
    if (nodeState.teleop) return CONTROL_NODE_NAMES.teleop;
    if (nodeState.wall_align) return CONTROL_NODE_NAMES.wall_align;
    return null;
  }

  async function refreshTune() {
    const target = activeControlNode();
    if (target === tuneNode) return;      // 변화 없음
    tuneNode = target;
    const sl = document.getElementById('sliders');
    const st = document.getElementById('tune-status');
    sl.innerHTML = '';
    if (!target) {
      st.textContent = '조종 노드를 켜면 게인 슬라이더가 나타납니다.';
      return;
    }
    st.textContent = `${target} 파라미터를 읽는 중…`;
    const r = await (await fetch(
      '/params?node=' + encodeURIComponent(target))).json();
    if (!r.ok) { st.textContent = r.msg; st.classList.add('msg-err'); return; }
    st.classList.remove('msg-err');
    st.textContent = `${target} — ${r.params.length}개 파라미터`;

    for (const p of r.params) {
      // 범위는 현재 값 기준으로 잡는다. 0 이면 고정 범위를 준다.
      const base = Math.abs(p.value) > 1e-9 ? Math.abs(p.value) : 1;
      const max = base * 4, step = p.type === 'integer' ? 1 : base / 200;
      const row = document.createElement('div');
      row.className = 'slider';
      row.innerHTML =
        `<label for="sr-${p.name}">${p.name}</label>` +
        `<input type="range" id="sr-${p.name}" min="0" max="${max}"` +
        ` step="${step}" value="${p.value}">` +
        `<input type="number" id="sn-${p.name}" step="${step}"` +
        ` value="${p.value}">`;
      document.getElementById('sliders').appendChild(row);

      const range = document.getElementById('sr-' + p.name);
      const num = document.getElementById('sn-' + p.name);
      const apply = v => {
        range.value = v; num.value = v;
        fetch('/param', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(
            {node: target, name: p.name, value: parseFloat(v)}),
        }).then(r => r.json()).then(r => {
          if (!r.ok) { st.textContent = r.msg; st.classList.add('msg-err'); }
        }).catch(() => {});
      };
      range.addEventListener('input', () => apply(range.value));
      num.addEventListener('change', () => apply(num.value));
    }
  }

  // ── 프리셋 ──────────────────────────────────────────────────────
  const pmsg = document.getElementById('preset-msg');

  async function presetCall(action, name) {
    const target = activeControlNode();
    if (!target) { pmsg.textContent = '조종 노드를 먼저 켜세요'; return; }
    const r = await (await fetch('/preset', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({action: action, name: name, node: target}),
    })).json();
    pmsg.textContent = r.msg || '';
    pmsg.classList.toggle('msg-err', !r.ok);
    const sel = document.getElementById('preset-sel');
    sel.innerHTML = '';
    for (const p of (r.presets || [])) {
      const o = document.createElement('option');
      o.value = o.textContent = p;
      sel.appendChild(o);
    }
    if (action === 'load' && r.ok) { tuneNode = null; refreshTune(); }
  }

  document.getElementById('preset-save').addEventListener('click', () => {
    const n = document.getElementById('preset-name').value.trim();
    if (!n) { pmsg.textContent = '이름을 입력하세요'; return; }
    presetCall('save', n);
  });
  document.getElementById('preset-load').addEventListener('click', () => {
    const n = document.getElementById('preset-sel').value;
    if (!n) { pmsg.textContent = '불러올 프리셋이 없습니다'; return; }
    presetCall('load', n);
  });
  presetCall('list', '');
```

`poll()` 함수 안의 `drawThrusters(s.thrusters);` **앞**에 추가:

```javascript
    if (s.nodes) { nodeState = s.nodes; paintNodes(); refreshTune(); }
```

- [ ] **Step 11: 빌드 및 전체 통합 확인**

```bash
cd /home/hero/hero_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select pkrc_control 2>&1 | tail -3
source install/setup.bash

ros2 launch pkrc_control gui.launch.py &
sleep 6

echo "=== 노드 상태 ==="
curl -s http://127.0.0.1:8080/state | python3 -c \
  "import json,sys; print(json.load(sys.stdin)['nodes'])"

echo "=== 소나 켜기 ==="
curl -s -X POST http://127.0.0.1:8080/node \
  -H 'Content-Type: application/json' -d '{"key":"sonar","on":true}' \
  | python3 -m json.tool

echo "=== teleop 켜기 ==="
curl -s -X POST http://127.0.0.1:8080/node \
  -H 'Content-Type: application/json' -d '{"key":"teleop","on":true}' \
  | python3 -m json.tool
sleep 4

echo "=== 파라미터 읽기 ==="
curl -s "http://127.0.0.1:8080/params?node=keyboard_teleop_robust" \
  | python3 -c "import json,sys; d=json.load(sys.stdin); print(d['msg'] or f\"{len(d['params'])}개\"); print([p['name'] for p in d['params'][:6]])"

echo "=== 게인 설정 ==="
curl -s -X POST http://127.0.0.1:8080/param \
  -H 'Content-Type: application/json' \
  -d '{"node":"keyboard_teleop_robust","name":"hh_rate_kp","value":3.3}' \
  | python3 -m json.tool

echo "=== 프리셋 저장 ==="
curl -s -X POST http://127.0.0.1:8080/preset \
  -H 'Content-Type: application/json' \
  -d '{"action":"save","name":"test1","node":"keyboard_teleop_robust"}' \
  | python3 -m json.tool
cat ~/.ros/pkrc_presets/test1.yaml | head -8

echo "=== 인터록 확인: wall_align 켜면 teleop 이 내려가야 함 ==="
curl -s -X POST http://127.0.0.1:8080/node \
  -H 'Content-Type: application/json' -d '{"key":"wall_align","on":true}' \
  | python3 -c "import json,sys; n=json.load(sys.stdin)['nodes']; print('teleop:',n['teleop'],' wall_align:',n['wall_align'])"

kill %1
```

Expected:
- `nodes` 에 9개 키가 모두 `false`로 시작.
- 소나·teleop 켜기가 `{"ok": true}`.
- 파라미터 읽기가 30개 이상을 반환하고, 이름에 `hh_rate_kp` 등이 보인다. `water_density`·`gravity`·`dvl_topic` 같은 `read_only` 항목은 **없어야** 한다.
- 게인 설정이 `{"ok": true, "msg": "hh_rate_kp = 3.3"}`.
- `test1.yaml` 에 `node: keyboard_teleop_robust` 와 `params:` 아래 값들.
- **인터록: `teleop: False   wall_align: True`** — 이게 핵심이다. teleop이 True로 남으면 인터록이 동작하지 않은 것이니 CAN 충돌 위험이 있다.

- [ ] **Step 12: 브라우저 최종 확인**

```bash
cd /home/hero/hero_ws && source install/setup.bash
ros2 launch pkrc_control gui.launch.py
```

브라우저에서 `http://192.168.0.14:8080` (또는 `hostname -I` 로 나온 IP)를 열어 아래를 눈으로 확인한다. 자동 검증이 불가능한 부분이다.

- [ ] 레이저 카메라 영상이 실시간으로 보인다
- [ ] 센서 토글 버튼이 켜지면 초록색이 된다
- [ ] 측위 그룹에서 하나를 켜면 다른 하나가 자동으로 꺼진다
- [ ] 조종 그룹에서 하나를 켜면 다른 하나가 자동으로 꺼진다
- [ ] 조종 노드를 켜면 게인 슬라이더가 나타난다
- [ ] 슬라이더를 움직이면 값이 즉시 반영된다 (터미널 로그의 `실시간 반영:` 확인)
- [ ] 프리셋 저장 후 값을 바꾸고 불러오면 되돌아온다
- [ ] 수치 패널·전류 바·그래프가 움직인다
- [ ] 방향키 버튼을 누른 채 탭을 닫으면 서버 로그에 `강제 정지(x) 발행` 이 찍힌다

---

## Self-Review

**Spec coverage:**

| 스펙 요구 | 태스크 |
|:---|:---|
| §4 아키텍처 (단일 노드 + stdlib HTTP) | Task 3 |
| §4.1 스레드 모델 (ThreadingHTTPServer) | Task 3 Step 3 |
| §5.1 영상 중계 재인코딩 0 | Task 4 |
| §5.1 QoS BEST_EFFORT/depth=1 | Task 4 Step 3 |
| §5.1 카메라 죽음 표시 | Task 4 Step 6, Task 6 Step 6 |
| §5.2 조종 명령 전달 (`/gui/key`) | Task 2 |
| §5.3 Watchdog | Task 5 |
| §5.4 센서 토글 분해 | Task 7 Step 3 |
| §5.4 video4 인터록 | Task 7 Step 5 |
| §5.4 gscam/usb_cam 제외 | Task 7 Step 1 (테스트로 강제) |
| §5.4 `_terminate` 재사용 | Task 7 Step 3 (import) |
| §5.4 카메라 기동 규칙 | Task 7 Step 6 (`_ensure_camera_once`) |
| §5.5 모드 전환 순서 (x → 대기 → 종료) | Task 7 Step 5 (`_stop_locked`) |
| §5.6 게인 튜닝 자동 목록 | Task 7 Step 6 (`list_tunable`) |
| §5.6 `read_only` 제외 | Task 7 Step 6 |
| §5.6 프리셋 YAML | Task 7 Step 6 |
| §5.7 모니터링 3종 | Task 6 |
| §6 백업 | Task 1 |
| §7 신규 파일 3개 + setup.py | Task 3 |
| §8 오류 처리 8항목 | Task 3~7 각 단계에 분산 |
| §9 검증 3항목 | Task 3/5/7 의 `_selftest` |

빠진 스펙 항목 없음.

**Type consistency:** `mjpeg_frame(bytes) -> bytes`, `FrameStore.get() -> (bytes|None, float)`, `KeyWatchdog.check(now) -> str|None`, `TopicCache.get(now) -> list|str|None`, `ProcManager.start/stop -> (bool, str)`, `group_siblings(str) -> list`, `list_tunable(str) -> (list|None, str)`, `set_param -> (bool, str)`, `preset_save/load -> (bool, str)` — 모든 태스크에서 일관되게 쓰였다.

**알려진 설계 판단:** `_call()`이 HTTP 스레드에서 서비스를 호출하며 `future.add_done_callback` + `Event`로 기다린다. rclpy가 메인 스레드에서 이미 spin 중이므로 HTTP 스레드에서 `spin_until_future_complete`를 부르면 executor 충돌이 난다. 이 방식이 그 문제를 피한다.
