# PKRC 통합 관제 웹 GUI — 설계

- **작성일**: 2026-07-30
- **대상 워크스페이스**: `/home/hero/hero_ws`
- **상태**: 승인 대기

## 1. 목적

수중로봇 PKRC의 센서·측위·조종 노드를 브라우저 한 화면에서 켜고 끄고, 레이저 카메라
영상을 실시간으로 보고, 조종 모드를 버튼으로 전환하고, 제어 게인을 실시간
튜닝한다. 수조 실험 중 노트북 한 대로 전부 처리하는 것이 목표다.

## 2. 환경 제약 (설계를 규정하는 사실)

조사로 확인한 사실이며, 선택이 아니라 전제다.

| 사실 | 근거 | 설계 결과 |
|:---|:---|:---|
| `DISPLAY` 비어 있음 (headless Jetson) | 셸 환경변수, `live_tuning.py:216-220`의 기존 경고 | rqt·PyQt 불가. **웹이 유일한 선택** |
| 웹 프레임워크 미설치 (flask/fastapi 없음) | `python3 -c "import flask"` 실패 | stdlib `http.server` 사용, 새 의존성 0 |
| `/image_raw/compressed`가 MJPG passthrough | `camera_node.py:34-38, 109-115` | JPEG 바이트 그대로 중계 → 재인코딩 0 |
| 실시간 튜닝 인프라 기존 존재 | `live_tuning.py` `_ROUTES` 약 50개 파라미터 | 새 메커니즘 불필요, `set_parameters`만 호출 |
| `hero_ws`가 git 저장소 아님 | `git rev-parse` 실패 | 제어 코드 수정 전 `.bak` 사본 필수 |

### 2.1 카메라 장치 배치

`v4l2-ctl --info`로 확인한 실제 매핑이다. 초기 가정(ArUco가 video0)은 **틀렸고**
아래가 정정된 사실이다.

| 장치 | 카메라 | 사용 노드 | 근거 |
|:---|:---|:---|:---|
| `/dev/video0` | exploreHD | `laser_camera_publisher` | `camera_node.py:21` |
| `/dev/video0` | exploreHD | exploreHD gscam (full_system) | `full_system.launch.py:108` |
| `/dev/video4` | stellarHD | `aruco_detector_6dof` | `aruco_detector_6dof.py:50` (`camera_device` 기본값 4) |
| `/dev/video4` | stellarHD | stellarHD usb_cam (full_system) | `full_system.launch.py:88` |

V4L2 장치는 배타적 open이다. 같은 장치를 두 노드가 열면 나중에 뜬 쪽이 실패한다.

## 3. 결정 사항

| 항목 | 결정 | 이유 |
|:---|:---|:---|
| 플랫폼 | 웹 (stdlib `http.server`) | headless, 의존성 0 |
| 영상 | 레이저 카메라만 (`/image_raw/compressed`) | 이미 JPEG → 무비용 |
| 조종 명령 전달 | `/gui/key` 토픽 구독 추가 | stdin raw 모드 우회, 터미널 경로 보존 |
| 조종 안전 | 생존 시그널 + watchdog 자동 정지 | 통신 끊김 시 추력 폭주 방지 |
| 게인 보존 | 프리셋 YAML 저장/불러오기 | 재시작 후에도 실험값 유지 |
| 바인드 | `0.0.0.0:8080` | 같은 망 노트북에서 접속 |
| video0 | **레이저 카메라 상시 고정** | 인터록 하나 제거 |
| 카메라 기동 | 이미 돌면 사용, 없으면 띄우고 소유 | 수동 실행과 충돌 없음 |
| 백업 | `.bak-20260730` 사본 2개 | git 없음, 기존 관행(`robust_original.py`) 계승 |
| 모니터링 | 수치 패널 + 전류 바 + 시계열 그래프 | 게인 튜닝은 숫자만으로 판단 불가 |

## 4. 아키텍처

파일 2개, 새 의존성 0개. rclpy 노드 하나가 곧 웹서버다. 별도 브리지 프로세스가 없다.

```
브라우저 (노트북, 같은 망)
   │  HTTP  0.0.0.0:8080
   ▼
gui_server 노드 (Jetson)
   ├─ GET  /          → gui.html
   ├─ GET  /stream    → multipart/x-mixed-replace MJPEG
   ├─ GET  /state     → JSON 텔레메트리 (브라우저 10Hz 폴링)
   ├─ POST /key       → /gui/key 발행 + heartbeat 갱신
   ├─ POST /param     → set_parameters
   ├─ POST /preset    → YAML 저장/불러오기
   └─ POST /node      → subprocess 로 launch 기동/종료
        │
        └─ ROS: /image_raw/compressed 구독 (BEST_EFFORT, depth=1)
                /teleop/yaw_debug, /teleop/depth_debug,
                /teleop/thruster_currents 구독
                /gui/key 발행
                파라미터 클라이언트, watchdog 타이머 (10Hz)
```

### 4.1 스레드 모델

`ThreadingHTTPServer`를 데몬 스레드로 띄우고, rclpy는 메인 스레드에서 spin한다.
공유 상태(최신 JPEG 프레임, 텔레메트리, heartbeat 시각)는 `threading.Lock` 하나로
보호한다.

MJPEG 스트림 응답은 스레드 하나를 붙잡은 채 계속 프레임을 밀어내므로
`ThreadingHTTPServer`가 필수다. 단일 스레드 서버면 스트림이 다른 모든 요청을
영구히 막는다.

## 5. 컴포넌트

### 5.1 영상 중계 (재인코딩 0)

`camera_node.py:109-115`가 내보내는 `msg.data`는 카메라 하드웨어가 만든 JPEG
바이트다. 서버는 그것을 multipart 경계와 함께 그대로 이어 붙인다.

```
--frame\r\nContent-Type: image/jpeg\r\nContent-Length: N\r\n\r\n<msg.data 그대로>\r\n
```

브라우저는 `<img src="/stream">` 한 줄로 재생한다. cv2도, JS 디코더도,
`web_video_server`도 필요 없다.

**QoS 함정 (반드시 지킬 것)**: `/image_raw/compressed`는
`BEST_EFFORT, KEEP_LAST, depth=1`로 퍼블리시된다(`camera_node.py:59-63`).
구독도 동일 QoS여야 한다. 기본 RELIABLE·depth=10으로 구독하면 **한 프레임도
받지 못한다**. 이 코드베이스는 같은 함정을
`keyboard_control_teleop.py:586-589`(압력)와 `:592-596`(DVL)에서 이미 주석으로
기록해뒀다.

**카메라 죽음 표시**: `laser_camera_publisher`는 카메라를 못 열면
`camera_node.py:57`에서 `RuntimeError`로 죽는다. 그러면 gui_server는 살아있고
영상만 안 나온다. `/state`에 마지막 프레임 수신 시각을 넣어 UI가 "카메라 노드
죽음"을 명시적으로 표시한다. 검은 화면만 보이면 원인을 알 수 없다.

### 5.2 조종 명령 전달

브라우저는 방향키 버튼을 누르는 동안 20Hz로 `POST /key`를 반복한다. 노드의
auto-repeat 전제(`key_timeout = 0.40`, `keyboard_control_teleop.py:537`)를 그대로
활용하므로 제어 로직 자체는 바꿀 필요가 없다. 떼면 전송이 멈추고 0.4초 뒤 기존
decay가 작동한다.

노드 측 변경은 두 파일에 각각 약 15줄이다.

```python
# __init__ 안
self._pending_gui_key = ''
self.create_subscription(String, '/gui/key', self._gui_key_cb, 10)

def _gui_key_cb(self, msg):
    self._pending_gui_key = msg.data

# get_key() 맨 앞
def get_key(self):
    if self._pending_gui_key:
        k, self._pending_gui_key = self._pending_gui_key, ''
        return k
    ... 기존 stdin 로직 그대로 ...
```

기존 stdin 경로는 손대지 않는다. 터미널 조작이 그대로 살아 있으므로 웹이 죽어도
기존 워크플로가 무사하다.

### 5.3 Watchdog (안전 필수 요소)

**이 설계에서 가장 중요한 부분이며 단순화 대상이 아니다.**

서버는 10Hz 타이머로 마지막 `/key` 수신 시각을 확인한다. 0.5초 넘게 조용하면서
직전에 이동 명령이 있었다면 `x`(전 축 정지 + 컨트롤러 리셋)를 자동 발행한다.

탭을 닫든, 와이파이가 끊기든, 노트북 배터리가 나가든 로봇이 추력을 물고 달아나지
않는다. 물에 잠긴 로봇이 명령 없이 계속 밀고 나가는 것은 회수 불가 상황이다.

### 5.4 센서 토글

`full_system.launch.py`를 통짜로 토글하지 않는다. 하위 launch가 전부 개별 패키지에
존재하고 빌드도 되어 있으므로(`install/` 확인) 개별 토글로 쪼갠다.

| 토글 | 실행 대상 | 그룹 |
|:---|:---|:---|
| 압력 (Bar10XT) | `bar10xt_ros2 bar10xt.launch.py` | — |
| DVL-A50 | `dvl_a50 dvl_a50.launch.py` | — |
| IMU (GV7) | `microstrain_inertial_driver microstrain_launch.py` | — |
| Lumen LED | `lumen_led lumen_led.launch.py` | — |
| Ping1D 소나 | `ping1d_sonar ping_sonar.launch.py` | — |
| Localization (UKFM) | `pkrc_controller localization.launch.py` | video4 |
| ArUco 단독 | `active_marker aruco_detector_6dof` | video4 |

**제외 대상**: exploreHD gscam, stellarHD usb_cam. 전자는 레이저 카메라와 video0을
다투고, 후자는 ArUco와 video4를 다툰다. UI에 두고 "누르면 화면이 죽는다"로
만드는 것은 함정이다.

**video0**: 레이저 카메라 상시 고정. 토글을 UI에서 아예 없앤다. `/image_raw/compressed`를
내는 것은 그 노드뿐이므로 웹 화면이 나오려면 반드시 떠 있어야 한다. 이 결정으로
상호배타 그룹 하나, 프로세스 전환 로직, UI 라디오 하나가 통째로 사라진다.

**video4 그룹**: `localization.launch.py`는 내부에 `aruco_detector_6dof`를
포함한다(`localization.launch.py:62`). 따라서 "Localization"과 "ArUco 단독"은
라디오(상호배타)다. 하나를 켜면 다른 하나를 먼저 내린다.

**프로세스 기동/종료**: `subprocess.Popen(..., start_new_session=True)`로 띄우고,
종료는 `live_tuning.py:158-207`의 `_descendants()` + `_terminate()`를 **재사용**한다.
`ros2 launch`는 래퍼이고 실제 자식이 별도 프로세스 그룹으로 빠져나가는데, 이
코드베이스는 이미 그 문제를 풀어놨다. 다시 쓸 이유가 없다.

**카메라 기동 규칙**: 시작 시 `/image_raw/compressed` 퍼블리셔가 있는지 먼저
확인한다. 있으면 구독만 하고 소유하지 않는다(gui_server 종료 시에도 살려둔다).
없으면 `ros2 launch`로 띄우고 소유한다(종료 시 정리). 수동·자동 어느 순서로
띄워도 동작한다.

### 5.5 조종 모드 전환

`keyboard_control_teleop`과 `keyboard_control_wall_align`은 같은 CAN 버스와 같은
VESC ID(`0x151`~`0x156`)를 만지므로 **절대 동시에 떠서는 안 된다**. 라디오
그룹(`없음` / `teleop` / `wall_align`)이다.

전환 순서가 중요하다.

1. 현재 노드에 `x` 발행 → 전 축 정지 + 컨트롤러 리셋
2. 정지 반영 대기 (약 0.3초)
3. 프로세스 종료
4. 새 노드 실행

정지 없이 죽이면 VESC가 마지막 전류 명령을 계속 유지한다.

### 5.6 게인 튜닝 + 프리셋

서버가 대상 노드의 `list_parameters`/`get_parameters`로 현재 값을 읽어 슬라이더를
자동 생성한다. 파라미터 목록을 GUI에 하드코딩하지 않는 이유는 두 가지다.
`live_tuning.py`의 `_ROUTES`가 이미 SSOT이고, 노드마다 선언 집합이
다르다(teleop 약 40개 vs wall_align 약 60개).

`_FIXED`(`live_tuning.py:30-32`)로 `read_only` 표시된 물성값·구독 토픽은 서술자를
읽어 자동 제외한다. 이미 그 목적으로 만들어진 표시다.

프리셋은 `~/.ros/pkrc_presets/<이름>.yaml`에 저장한다. 소스 트리가 아닌 이유는
실험값이 소스를 오염시키지 않아야 하고, 패키지를 재빌드해도 살아남아야 하기
때문이다. 저장은 현재 노드의 튜닝 가능 파라미터 전체 스냅샷, 불러오기는 한 번의
`set_parameters` 호출이다.

### 5.7 모니터링

이미 퍼블리시 중인 토픽을 구독해 `/state` JSON으로 합친다. 부가 부담이 거의 없다.

| 토픽 | 내용 | 근거 |
|:---|:---|:---|
| `/teleop/yaw_debug` | 12개: yaw 목표·현재·오차·rate, cmd, heave, pitch, yawing, DVL vx·vy, drift fb | `keyboard_control_teleop.py:1106-1119` |
| `/teleop/depth_debug` | 6개: 목표·현재 수심, 오차, heave cmd, 추정 속도, 압력 | `:1121-1128` |
| `/teleop/thruster_currents` | T1~T6 전류 | `:1100` |
| `/teleop/wall_debug` | 11개: 목표·실측 거리, 오차, surge cmd, 소나 신뢰도, 소나 생존, 스캔 누적각, 최소거리, 그 방위, 샘플수, raw 거리 | `keyboard_control_wall_align.py:1677-1689` |
| `/teleop/wall_mode` | `String` — 현재 상태 (IDLE/SCAN/BRAKE/TURN/APPROACH/HOLD) | `:788-789` |

소나 거리와 wall_align 상태는 별도 계산 없이 `/teleop/wall_debug`와
`/teleop/wall_mode`에서 그대로 읽는다. 두 토픽은 wall_align 노드가 떠 있을 때만
존재하므로, 없을 때 해당 패널은 비활성으로 렌더링한다.

브라우저가 10Hz로 폴링하고 세 가지를 렌더링한다.

1. **핵심 수치 패널** — yaw 목표/현재/오차, 수심 목표/현재, 소나 거리, DVL 속도, 현재 상태(SCAN/TURN/APPROACH/HOLD)
2. **T1~T6 전류 바** — 포화가 눈에 보이도록 `max_current` 한계선 표시
3. **시계열 스파크라인** — yaw 오차·수심 최근 30초. Canvas 직접 그리기, 차트 라이브러리 없음

게인 튜닝은 숫자만으로 진동·오버슈트를 판단할 수 없어서 그래프가 실질적으로
필요하다.

## 6. 백업 (구현 전 필수)

`hero_ws`는 git 저장소가 아니다. 복구 수단이 파일 사본밖에 없으므로 제어 코드 수정
전에 반드시 사본을 만든다. `keyboard_control_robust_original.py`가 이미 있는 것을
보면 이 관행이 프로젝트에 존재한다.

```
pkrc_control/
  keyboard_control_teleop.py                   ← 수정됨
  keyboard_control_teleop.py.bak-20260730      ← 원본
  keyboard_control_wall_align.py               ← 수정됨
  keyboard_control_wall_align.py.bak-20260730  ← 원본
```

`.bak-*` 확장자라 colcon과 `setup.py`의 `find_packages`가 무시하고 entry_point도
생기지 않는다. 복구는 `cp` 한 번이다.

## 7. 신규 파일

| 경로 | 내용 |
|:---|:---|
| `pkrc_control/pkrc_control/gui_server.py` | rclpy 노드 + HTTP 서버 |
| `pkrc_control/resource/gui.html` | 단일 페이지 UI (인라인 CSS/JS) |
| `pkrc_control/launch/gui.launch.py` | gui_server 기동 (host·port 인수) |

`setup.py` 변경: `gui_server` entry_point 추가, `resource/gui.html`과 `launch/`를
`data_files`에 추가(현재 `launch/` 디렉터리가 `pkrc_control`에 없으므로 신규 생성).

## 8. 오류 처리

| 상황 | 처리 |
|:---|:---|
| 브라우저 통신 끊김 | watchdog이 0.5초 후 `x` 자동 발행 |
| 카메라 노드 죽음 | `/state`에 프레임 수신 시각 → UI에 "카메라 죽음" 명시 |
| launch 기동 실패 | `Popen` 종료코드 확인 → UI에 실패 표시, 토글 원위치 |
| 파라미터 설정 실패 | `set_parameters` 결과 확인 → 슬라이더를 실제 값으로 되돌림 |
| 대상 노드 없는데 튜닝 시도 | 노드 미기동 시 슬라이더 비활성 |
| 프리셋 파일 손상 | 로드 시 예외 잡고 UI에 알림, 현재 값 유지 |
| 포트 8080 사용 중 | 기동 시 명확한 오류 로그 후 종료 |

## 9. 검증

물에 들어가기 전에 확인 가능한 것만 자동화한다. `demo()` 자체 검증 하나
(`python3 gui_server.py --selftest`)로 셋을 확인한다.

1. multipart MJPEG 프레이밍이 규격에 맞는지 (경계·헤더·길이)
2. watchdog이 0.5초 무응답에서 정확히 `x`를 발행하는지 (가짜 시계로)
3. video4 인터록이 같은 그룹의 이전 노드를 실제로 내리는지 (가짜 프로세스로)

나머지(실제 영상, 실제 추력, CAN 전송)는 하드웨어가 붙은 상태에서 눈으로 확인해야
한다. 프레임워크·픽스처는 쓰지 않는다.

## 10. 만들지 않는 것 (YAGNI)

| 항목 | 이유 |
|:---|:---|
| 인증·로그인 | 폐쇄망 전제 |
| WebSocket | 10Hz 폴링에 과함 |
| rosbridge / web_video_server | 각각 대체 가능, 의존성만 늘어남 |
| 다중 사용자 동시 제어 조정 | 한 사람이 조종하는 장비 |
| 녹화 기능 | `ros2 bag`이 이미 있고 `bagfiles/`도 존재 |
| stellarHD·exploreHD 웹 영상 | 장치 충돌 + raw는 재인코딩 비용 |
| 차트 라이브러리 | Canvas 직접 그리기로 충분 |

## 11. 알려진 한계

- **stellarHD 영상을 나중에 추가하려면** raw Image(1600x1200@60)를 JPEG로
  재인코딩해야 하므로 Jetson CPU 비용이 붙는다. 해상도·프레임 감축이 전제다.
- **exploreHD gscam은 웹 GUI와 함께 쓸 수 없다.** 레이저 카메라와 같은 장치다.
  둘 다 필요해지면 하드웨어 추가나 노드 통합이 필요하다.
- **프리셋은 노드별로 호환되지 않는다.** teleop과 wall_align의 파라미터 집합이
  다르므로 프리셋 파일에 대상 노드명을 기록하고 불일치 시 거부한다.
