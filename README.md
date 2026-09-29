# PKRC Underwater Scanning Robot

수중 벽면 검사용 ROV(PKRC)의 ROS 2 워크스페이스입니다. Jetson Orin 위에서 스러스터 제어, 센서 드라이버, 카메라·레이저 검사, 측위(UKF-M), 웹 관제 GUI를 실행합니다.

- **플랫폼**: NVIDIA Jetson Orin (headless), Ubuntu + ROS 2 Humble
- **추진**: VESC 6채널(CAN `can0`) — surge 2 · sway 2 · heave 2
- **센서**: Microstrain 3DM-GV7-INS(IMU), Water Linked DVL-A50, Keller 압력계 2종, Ping1D 소나
- **카메라**: exploreHD ×2(레이저·하방), stellarHD(ArUco 능동 마커)
- **운용**: 브라우저 웹 GUI 하나로 노드 on/off, 조종, 영상, 게인 튜닝, rosbag 기록

---

## 목차

1. [빠른 시작](#1-빠른-시작)
2. [워크스페이스 구성](#2-워크스페이스-구성)
3. [빌드](#3-빌드)
4. [하드웨어 준비](#4-하드웨어-준비)
5. [웹 GUI로 운용하기](#5-웹-gui로-운용하기)
6. [패키지별 사용법](#6-패키지별-사용법)
7. [캘리브레이션](#7-캘리브레이션)
8. [토픽 요약](#8-토픽-요약)
9. [알려진 문제·주의사항](#9-알려진-문제주의사항)

---

## 1. 빠른 시작

```bash
# 1) 빌드 (최초 1회, 코드 수정 후마다)
cd ~/hero_ws
colcon build --symlink-install
source install/setup.bash

# 2) CAN 버스 활성화 (부팅마다)
sudo ip link set can0 up type can bitrate 500000

# 3) 웹 GUI 실행
ros2 launch pkrc_control gui.launch.py
```

브라우저에서 `http://<Jetson IP>:8080` 에 접속한 뒤 **노드** 패널에서 필요한 센서(IMU·압력·DVL)와 조종 모드(teleop 또는 wall_align)를 켭니다.

---

## 2. 워크스페이스 구성

| 경로 | 종류 | 역할 |
|:---|:---|:---|
| `src/pkrc_control` | 자체 | **웹 GUI**, 키보드 텔레옵, 벽면 정렬, 스러스터 CAN 출력, 로깅 도구 |
| `src/pkrc_controller` | 자체 | 측위(UKF-M), 센서 묶음 launch, 자기장 캘리브레이션 |
| `src/laser_camera_publisher` | 자체 | exploreHD 카메라 발행(MJPG 패스스루) + 카메라·레이저 평면 캘리브레이션 |
| `src/active_marker` | 자체 | stellarHD 카메라 + LED ArUco 마커 6DoF 검출 |
| `src/pressure_sensor/bar10xt_ros2` | 자체 | Keller Bar10XT 압력·수온·수심 (I2C) |
| `src/keller_ld_sensor` | 자체 | Keller LD 시리즈 압력계 (I2C) |
| `src/ping1d_sonar` | 포크 | Blue Robotics Ping1D 소나 |
| `src/lumen_led` | 자체 | Blue Robotics Lumen LED (Jetson 하드웨어 PWM) |
| `src/acoustic_modem` | 자체 | 수중 음향 모뎀 UART 수신 |
| `src/alpha_grabber` | 스크립트 | Reach Alpha 2 그리퍼 텔레옵 (ROS 패키지 아님) |
| `src/microstrain_inertial` | 외부 | Microstrain IMU 드라이버 (GV7-INS 설정 추가) |
| `src/dvl_a50`, `src/dvl_msgs` | 외부 | Water Linked DVL-A50 드라이버·메시지 ([paagutie/dvl-a50](https://github.com/paagutie/dvl-a50), [paagutie/dvl_msgs](https://github.com/paagutie/dvl_msgs)) — 로컬 수정 포함 |
| `src/nmea_msgs` | 외부 | NMEA 메시지 ([ros-drivers/nmea_msgs](https://github.com/ros-drivers/nmea_msgs)) |
| `src/reach_robotics_sdk` | 외부 | Reach Robotics SDK·`bplprotocol` ([Reach-Robotics/reach_robotics_sdk](https://github.com/Reach-Robotics/reach_robotics_sdk)) — 로컬 수정 포함 |
| `src/plot_tools` | 데이터 | 측위 비교 CSV·그래프 결과물 (스크립트 없음) |
| `tools/` | 스크립트 | Jetson pinmux/PWM7 진단·설정 스크립트 |
| `camera_laser_cal/` | 레거시 | 구버전 캘리브레이션 스크립트 (현행은 `src/laser_camera_publisher/calibration/`) |
| `docs/` | 문서 | 장비 매뉴얼 PDF(git 제외), 설계 문서 |

외부 패키지 4개(`dvl_a50`, `dvl_msgs`, `nmea_msgs`, `reach_robotics_sdk`)는 이 저장소에 코드째 포함되어 있어 clone만으로 빌드됩니다.

---

## 3. 빌드

```bash
cd ~/hero_ws
colcon build --symlink-install
source install/setup.bash
```

`pkrc_control`의 `package.xml`에는 런타임 의존성이 선언되어 있지 않습니다. 새 머신에서는 아래 파이썬 패키지를 직접 설치해야 합니다.

```bash
sudo apt install python3-can python3-yaml python3-smbus2 libgpiod-dev gpiod
```

- 카메라 노드는 OpenCV의 GStreamer 백엔드와 `v4l2-ctl`(`v4l-utils`)을 사용합니다.
- `ping1d_sonar`는 내부의 `ping-python`(brping) 라이브러리를 사용합니다. 설치 방법은 `src/ping1d_sonar/README.md`를 참고하세요.
- `full_system.launch.py`는 `usb_cam`, `gscam` 패키지가 설치되어 있어야 합니다.

---

## 4. 하드웨어 준비

### 4.1 CAN (스러스터)

```bash
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0      # 상태 확인
```

| VESC | CAN ID (extended) |
|:---|:---|
| surge_left / surge_right | `0x151` / `0x152` |
| sway_left / sway_right | `0x153` / `0x154` |
| heave_up / heave_down | `0x155` / `0x156` |

명령은 전류[A]를 mA 정수로 바꾼 4바이트 빅엔디언 값이며, 노드에서 ±5.0 A로 하드 클램프합니다.

### 4.2 장치 경로

| 장치 | 경로·주소 | 비고 |
|:---|:---|:---|
| IMU (GV7-INS) | `/dev/ttyACM0`, 115200 | udev 고정 규칙 없음 |
| DVL-A50 | TCP `192.168.0.220` (launch 기본값) | `dvl_address:=` 로 변경 |
| Bar10XT / Keller LD | I2C bus 1, 주소 `0x40` | |
| Ping1D 소나 | `/dev/ping`, 115200 | `/etc/udev/rules.d/99-ping1d.rules` |
| 음향 모뎀 | `/dev/ttyUSB0`, 9600 | 소스에 하드코딩 |
| Alpha 그리퍼 | `/dev/ttyUSB1`, 115200 (RS-485) | 번호가 바뀔 수 있음 |
| Lumen LED | J37 GPIO07 → PWM7 (`pwmchip3/pwm0`) | root 또는 사전 pinmux 필요 |
| 하방 카메라 | `/dev/v4l/by-path/...usb-0:2.1.1:1.0-video-index0` | USB 허브 포트 2.1.1 |
| 레이저 카메라 | `/dev/v4l/by-path/...usb-0:2.1.2:1.0-video-index0` | USB 허브 포트 2.1.2 |
| stellarHD | `/dev/v4l/by-path/...usb-0:2.1.3:1.0-video-index0` | USB 허브 포트 2.1.3 |

> **카메라는 `by-path`로 지정합니다.** exploreHD 두 대가 같은 USB 시리얼(SN00009)을 쓰기 때문에 `by-id`로는 구분이 안 되고, `/dev/videoN` 번호는 USB 연결 순서에 따라 바뀝니다. 허브를 다른 포트로 옮기면 경로 상수만 고치면 됩니다.

---

## 5. 웹 GUI로 운용하기

```bash
ros2 launch pkrc_control gui.launch.py                   # 0.0.0.0:8080
ros2 launch pkrc_control gui.launch.py port:=9000
ros2 launch pkrc_control gui.launch.py host:=127.0.0.1   # SSH 포트포워딩 전용
```

`gui_server` 노드 하나가 HTTP 서버를 내장하고 있어서 별도 브리지가 필요 없습니다. 브라우저에서 할 수 있는 일은 다음과 같습니다.

- **노드 on/off**: 버튼을 누르면 서버가 `ros2 launch`/`ros2 run` 프로세스를 직접 띄우고 끕니다. 터미널에서 따로 띄운 노드도 찾아서 정리합니다.
- **조종**: 화면 키패드와 물리 키보드가 같은 키 매핑을 씁니다.
- **영상**: laser / stellar / downward 중 한 번에 하나만 MJPEG로 스트리밍합니다(대역폭 보호). 레이저 영상에는 레이저 선 검출 오버레이를 켤 수 있습니다.
- **게인 튜닝·프리셋**: 조종 노드의 파라미터를 실시간으로 바꾸고 `~/.ros/pkrc_presets`에 저장하거나 불러옵니다.
- **LED 밝기**, **rosbag 기록**(`ros2 bag record -a` → `~/hero_ws/rosbags/<타임스탬프>`)

### 5.1 노드 목록과 상호배타 그룹

| 버튼 | 실행 내용 | 그룹 |
|:---|:---|:---|
| `teleop` | `keyboard_control_teleop` (수동 조종) | **control** |
| `wall_align` | `keyboard_control_wall_align` (수동 + 벽면 정렬) | **control** |
| `localization` | `pkrc_controller localization.launch.py` (UKF-M + ArUco) | **video4** |
| `aruco` | ArUco 검출 단독 | **video4** |
| `pressure` / `dvl` / `imu` / `led` / `rosbag` | 각 드라이버 | 독립 |

같은 그룹의 노드는 동시에 켤 수 없습니다. control 그룹은 같은 CAN 버스와 VESC를 쓰고, video4 그룹은 같은 stellarHD 카메라를 씁니다. 조종 노드를 끌 때 GUI는 먼저 정지 키를 보내고 0.3초 기다린 다음 프로세스를 종료합니다. VESC에 마지막 전류가 남지 않게 하려는 것입니다.

### 5.2 조종 키

| 키 | 동작 |
|:---|:---|
| `↑` `↓` | surge 전진·후진 (누르는 동안) |
| `←` `→` | sway 좌·우 (누르는 동안) |
| `A` `D` | yaw 회전 (누르는 동안) |
| `W` `S` | 목표 수심 ±10 cm (1회) |
| `R` | 목표 방위 리셋 |
| `T` | 목표 수심 리셋 |
| `C` | 벽면 정렬 시작/취소 (wall_align 모드) |
| `X` | 정지 + 리셋 |

이동 키가 0.5초 이상 들어오지 않으면 서버가 자동으로 정지 명령을 보냅니다(watchdog). 이 자동 정지는 목표 방위·수심을 유지한 채 추력과 적분기만 정리합니다.

### 5.3 벽면 정렬 (wall_align)

`C`를 누르면 소나(`/sensor/sonar/ping1d/data`)를 이용해 다음 순서로 진행합니다.

`SCAN`(제자리 360° 회전하며 최소 거리 방위 탐색) → `BRAKE` → `TURN`(그 방위로 회전) → `APPROACH`(`target_distance` = 1.0 m까지 접근) → `HOLD`

정렬 중에 다른 조작 키를 누르면 즉시 중단됩니다. 소나의 `scan_length`는 wall_align의 `sonar_max_range`(5.0 m)와 맞춰야 합니다. 너무 짧으면 수조 벽이 범위 밖으로 나가 "Sonar lost"가 됩니다.

---

## 6. 패키지별 사용법

GUI 없이 각 노드를 직접 실행할 때 참고하세요.

### 6.1 pkrc_control — 조종

```bash
ros2 run pkrc_control keyboard_control_teleop       # 수동 조종 (기본: rqt_reconfigure 자동 실행)
ros2 run pkrc_control keyboard_control_teleop --ros-args -p tuning_gui:=false
ros2 run pkrc_control keyboard_control_wall_align   # 수동 + 벽면 정렬
ros2 run pkrc_control thruster_test                 # ROS 없이 스러스터 개별 테스트
ros2 run pkrc_control teleop_logger                 # IMU·키·스러스터 CSV 로깅
ros2 run pkrc_control wall_following_logger         # wall_align 디버그 CSV 로깅
```

- 제어 루프는 50 Hz이며, heading hold(각도→각속도 캐스케이드 PID), 수심 PID, DVL 기반 드리프트 보정을 포함합니다.
- 전류 한계는 `max_current_surge/sway`(3.0 A), `max_current_heave`(5.0 A)입니다. `-p`로 넘길 때는 **반드시 소수점을 붙이세요**(`3.0`). 정수 `3`을 넘기면 노드가 `InvalidParameterTypeException`으로 바로 죽습니다.
- 파라미터 전체 목록과 설계 의도는 [`src/pkrc_control/PARAMETERS.md`](src/pkrc_control/PARAMETERS.md)에 있습니다.
- `thruster_test`는 프로펠러가 실제로 돕니다. 로봇을 물 밖에 고정하고, teleop/wall_align을 끈 뒤 실행하세요.

### 6.2 pkrc_controller — 센서 묶음·측위

```bash
# 제어용 최소 센서 (압력 Bar10XT + DVL + IMU)
ros2 launch pkrc_controller control_sensors.launch.py [dvl_address:=192.168.0.220]

# 전체 센서 + 카메라 2대 + LED + 소나 + ArUco
ros2 launch pkrc_controller full_system.launch.py [cameras:=false] [use_rviz:=true]

# 측위 (UKF-M + ArUco)
ros2 launch pkrc_controller localization.launch.py
ros2 run pkrc_controller ukfm_data_logger          # UKF-M 결과 CSV 기록
```

- 입력: `/imu/data`, `/dvl/data`, `/bar10xt/pressure`(수심 = (절대압 − 101325 Pa) / ρg), `/aruco/pose_array`
- 출력: `/ukfm/odom`, `/ukfm/odom_validated`, `/ukfm/path`, `/ukfm/wall_distance`
- IMU가 한 번이라도 들어와야 추정을 시작합니다.
- 자기장 캘리브레이션: `python3 src/pkrc_controller/scripts/mag_calibration.py` (결과 `mag_cal_result.json`)

### 6.3 센서 드라이버

```bash
ros2 launch bar10xt_ros2 bar10xt.launch.py [bus:=1 rate:=5.0]      # /bar10xt/{pressure,temperature,depth}
ros2 run keller_ld_sensor keller_ld_node                           # /pressure_bar, /pressure(mbar), /temperature
ros2 launch microstrain_inertial_driver microstrain_launch.py      # GV7-INS 설정(gv7_ins.yml) 기본 적용
ros2 launch dvl_a50 dvl_a50.launch.py ip_address:=192.168.0.220
ros2 launch ping1d_sonar ping_sonar.launch.py [scan_length:=5000]  # /sensor/sonar/ping1d/*
ros2 run acoustic_modem receive_node                               # /uart_data
```

- **Bar10XT**: 이 개체는 압력 MSB가 고장 나 있습니다. 드라이버가 LSB 롤오버를 세서 상대 수심을 복원하지만, **절대 수심은 복원할 수 없고** 초당 약 2 m보다 빠르게 오르내리면 롤오버를 놓칩니다. 기동 시에는 수면에서 영점을 잡으세요(`auto_zero`). 근본적인 해결책은 센서 교체입니다.
- **GV7-INS**: GNSS가 없는 기종입니다. `filter_auto_heading_alignment_selector`는 비트필드이며 `0`(NONE)이어야 합니다. GNSS 비트(1·2)를 넣으면 드라이버가 기동에 실패합니다. 자력계 보조는 수중 금속·스러스터 전류 간섭 때문에 꺼 두었습니다.

### 6.4 Lumen LED

```bash
src/lumen_led/run_lumen.sh                      # sudo로 pinmux 설정 후 노드 실행 (권장)
ros2 launch lumen_led lumen_led.launch.py       # pinmux가 이미 설정된 경우
ros2 run lumen_led lumen_ctrl on|off|0.0~1.0    # 밝기 명령
```

pinmux 설정에는 root가 필요합니다. 부팅 때 자동으로 설정하려면 `src/lumen_led/systemd/lumen-pinmux.service`를 등록하세요. 노드는 기동 후 2초 동안 off 펄스(1100 µs)를 유지해 LED를 arm한 다음 밝기를 적용합니다.

### 6.5 카메라

```bash
ros2 launch laser_camera_publisher laser_camera.launch.py     # 레이저 → /image_raw/compressed
ros2 launch laser_camera_publisher downward_camera.launch.py  # 하방 → /downward/image_raw/compressed
ros2 run active_marker aruco_detector_6dof                    # stellarHD 직접 열어 검출 + 영상 재발행
ros2 run active_marker stellar_camera_publisher               # 검출 없이 stellarHD 영상만 발행
```

- exploreHD는 JPEG를 디코딩하지 않고 그대로 발행합니다(약 30 Hz). 노출·감마는 실행 중에 바꿀 수 있습니다.
  `ros2 param set /laser_camera_publisher exposure 10`
- OpenCV V4L2 백엔드로는 20 fps에서 막히기 때문에 GStreamer 백엔드를 씁니다. `cap.set()`이 장치까지 전달되지 않아서 노출은 `v4l2-ctl`로 설정합니다.
- **ArUco 검출기**: `DICT_4X4_1000`, 마커 크기 0.12 m(외곽 테두리 기준), 기본 ID 0–29(5×6 격자, 1.5 m 간격)입니다. 노출을 최소로 낮춰 LED 블롭 후보를 먼저 찾은 뒤 그 주변만 확대해서 검출합니다. 카메라 내부 파라미터는 **수중** 캘리브레이션 값이라, 공기 중에서 재면 Z가 약 1.33배 크게 나옵니다.
- `aruco_detector_6dof` 출력: `/aruco/pose_6dof`(가장 가까운 마커), `/aruco/pose_array`(전체 마커, `frame_id`에 `id:quality` 인코딩)

### 6.6 Alpha 그리퍼 (ROS 아님)

```bash
python3 src/alpha_grabber/tx_blink_test.py all   # 어떤 ttyUSB가 팔인지 TX LED로 확인
python3 src/alpha_grabber/grabber_teleop.py      # ↑↓ 집게, ←→ 회전
```

---

## 7. 캘리브레이션

현행 스크립트는 `src/laser_camera_publisher/calibration/`에 있습니다. 루트의 `camera_laser_cal/`은 구버전입니다.

### 7.1 카메라 내부 파라미터 (fisheye)

```bash
ros2 launch laser_camera_publisher laser_camera.launch.py   # 영상 발행
cd src/laser_camera_publisher/calibration
python3 calibrate_camera.py
```

체커보드는 내부 코너 4×5, 한 칸 70 mm입니다. `SPACE`로 캡처하고 15장 이상 모이면 `c`로 계산합니다(`d` 마지막 삭제, `q` 종료). 결과는 같은 폴더의 `camera_intrinsics.yaml`에 저장됩니다.

### 7.2 레이저 평면

```bash
pkill -f laser_camera_publisher     # 스크립트가 카메라를 직접 열므로 먼저 종료
cd src/laser_camera_publisher/calibration
python3 calibrate_laser.py [--device <by-path> --exposure 50 --angle 0]
```

Tkinter 창에서 체커보드(5×4)를 여러 위치·각도에 두고 레이저 선이 보드를 지나도록 한 뒤 `SPACE`로 캡처합니다. 10회 이상 캡처하면 `c`로 평면을 적합합니다. 그 밖의 키는 `b` 근/원 칸 크기 전환, `t` ROI 전환, `m` 검출 방식 전환입니다. 결과는 `green_laser_plane.yaml`(법선·d·RMSE·검출 파라미터)에 저장됩니다.

레이저가 과노출되어 하얗게 포화되면 `legacy`(초록색 분리) 방식은 검출에 실패합니다. 이럴 때는 `contrast`(국소 밝기 대비) 방식을 쓰세요. GUI의 레이저 오버레이도 이 스크립트와 같은 검출식을 씁니다.

---

## 8. 토픽 요약

| 토픽 | 타입 | 발행 |
|:---|:---|:---|
| `/imu/data` | `sensor_msgs/Imu` | microstrain |
| `/dvl/data` | `dvl_msgs/DVL` | dvl_a50 |
| `/bar10xt/pressure` · `/depth` · `/temperature` | `FluidPressure` · `Float64` · `Temperature` | bar10xt_ros2 |
| `/pressure` | `std_msgs/Float64` (mbar) | keller_ld_sensor |
| `/sensor/sonar/ping1d/data` · `/confidence` · `/range` | `Float32` · `Float32` · `Range` | ping1d_sonar |
| `/image_raw/compressed` | `CompressedImage` | 레이저 카메라 |
| `/downward/image_raw/compressed` | `CompressedImage` | 하방 카메라 |
| `/stellarHD/image_raw/compressed`, `/stellarHD/camera_info` | `CompressedImage`, `CameraInfo` | active_marker |
| `/aruco/pose_6dof` · `/aruco/pose_array` | `PoseStamped` · `PoseArray` | aruco_detector_6dof |
| `/gui/key` | `std_msgs/String` | gui_server → 조종 노드 |
| `/teleop/*` (force, thruster_currents, yaw/depth/wall_debug, wall_mode) | `Float64MultiArray` 등 | 조종 노드 |
| `/ukfm/*` | `Odometry`, `Path` | pkrc_controller |
| `lumen/brightness` · `lumen/on_off` · `lumen/state` | `Float32` · `Bool` · `Float32` | lumen_led |

QoS 주의: 압력(Bar10XT)과 DVL은 BEST_EFFORT, IMU와 측위는 RELIABLE, Ping1D는 RELIABLE입니다. 구독자와 QoS가 맞지 않으면 데이터가 들어오지 않습니다.

---

## 9. 알려진 문제·주의사항

아래는 코드를 읽으며 확인한 내용입니다. 실제 운용에서 재현 여부는 확인하지 않았습니다.

| 항목 | 내용 |
|:---|:---|
| `pkrc_controller/README.md` | `wall_following`, `depth_controller` 실행 예시가 있지만 현재 코드에는 해당 실행 파일이 없습니다(구버전 문서). |
| `keyboard_control_robust_original` | 레거시 노드입니다. `/pressure`(Float64)를 구독하며 GUI에서는 켤 수 없습니다. |
| `calibrate_camera.py` | 결과를 `~/ros2_ws/src/laser_ros/config/`로 복사하려는 코드가 남아 있습니다(다른 워크스페이스 경로, 무시해도 됨). |
| 캘리브레이션 결과 사용처 | `green_laser_plane.yaml`, `camera_intrinsics.yaml`을 실행 중에 읽는 ROS 노드는 없습니다. 현재는 오프라인 분석용입니다. |
| `src/dual_cam_launch.py` | 패키지에 설치되지 않는 단독 launch 파일입니다(`usb_cam`+`gscam` 구버전 구성). |
| 백업 파일 | `*.bak-20260730`, `aruco_detector_6dof copy*.py` 등은 빌드에 포함되지 않는 옛 사본입니다. |
| `tools/wake_sensor.py` | 이름과 달리 Keller LD 드라이버이며, Python 2식 `print() % ()` 구문 때문에 Python 3에서 오류가 납니다. |
| `tools/setup_pwm7_pinmux.py` 등 | `/dev/mem`에 접근하므로 `sudo`가 필요합니다. `find_pinmux_reg.py`는 GPIO 41을 5초 동안 실제로 구동합니다. |

Claude Code 작업 환경 설정 메모는 [`docs/claude-harness.md`](docs/claude-harness.md)에 있습니다.
