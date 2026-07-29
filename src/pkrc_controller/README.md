# PKRC Controller - 실제 로봇용 Wall Following & Depth Controller

시뮬레이션 코드를 실제 PKRC 수중로봇에 적용한 컨트롤러 패키지입니다.

## 하드웨어 구성

| 장치 | 역할 | 인터페이스 |
|------|------|-----------|
| DVL-A50 | 벽면 거리 측정 (altitude) | TCP/IP (192.168.1.99:16171) |
| MS5837 | 깊이/압력 측정 | I2C |
| VESC x6 | 스러스터 제어 | CAN (can0) |

### VESC 배치
- `vesc_1` (0x151): 전방 좌측 (surge)
- `vesc_2` (0x152): 전방 우측 (surge)
- `vesc_3` (0x153): 측면 좌측 (sway)
- `vesc_4` (0x154): 측면 우측 (sway)
- `vesc_5` (0x155): 수직 상 (heave)
- `vesc_6` (0x156): 수직 하 (heave)

## 의존성

```bash
# ROS2 패키지
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs

# Python 패키지
pip install python-can

# DVL 메시지 및 드라이버 (별도 설치 필요)
cd ~/hero_ws/src
git clone https://github.com/paagutie/dvl_msgs.git
git clone https://github.com/paagutie/dvl-a50.git
```

## 빌드

```bash
cd ~/hero_ws
colcon build --packages-select dvl_msgs dvl_a50 pkrc_controller
source install/setup.bash
```

## 사전 준비

### 1. CAN 버스 활성화
```bash
sudo ip link set can0 up type can bitrate 500000
# 또는 부팅 시 자동 설정: /etc/network/interfaces.d/can0
```

### 2. 센서 노드 실행
```bash
# 터미널 1: DVL-A50
ros2 run dvl_a50 dvl_a50 --ros-args -p dvl_address:='192.168.1.99'

# 터미널 2: 압력센서
ros2 run pressure_sensor pressure_sensor_node
```

## 실행 방법

### Wall Following (벽 따라가기 + 깊이 유지)
```bash
# 기본 실행
ros2 run pkrc_controller wall_following

# 파라미터와 함께
ros2 launch pkrc_controller wall_following.launch.py target_distance:=1.5 target_depth:=2.0
```

### Depth Controller Only (깊이 유지만)
```bash
ros2 run pkrc_controller depth_controller

# standalone 모드 (heave만 제어)
ros2 launch pkrc_controller depth_controller.launch.py standalone_mode:=true target_depth:=2.0
```

### Teleop Depth (키보드로 깊이 조절)
```bash
# 다른 터미널에서 실행
ros2 run pkrc_controller teleop_depth
# W: 깊게, S: 얕게, Q: 종료
```

### Full System (전체 시스템)
```bash
ros2 launch pkrc_controller full_system.launch.py
```

## 토픽

### 구독 (Subscribed)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/dvl/data` | `dvl_msgs/DVL` | DVL altitude (벽 거리) |
| `/pressure` | `std_msgs/Float64` | 압력 (mbar) |
| `/pkrc/depth/target` | `std_msgs/Float64` | 목표 깊이 |
| `/joy` | `sensor_msgs/Joy` | 조이스틱 (depth_controller) |

### 발행 (Published)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/pkrc/depth/current` | `std_msgs/Float64` | 현재 깊이 |
| `/pkrc/wall/distance` | `std_msgs/Float64` | 벽까지 거리 |

## 파라미터

### Wall Following
```yaml
target_distance: 1.0    # 벽과의 목표 거리 (m)
approach_speed: 0.4     # 접근 속도
follow_speed: 0.3       # 따라가기 속도
kp, ki, kd: 1.5, 0.1, 0.2  # 벽 거리 PID
yaw_gain: 0.1           # 회전 보정
```

### Depth Control
```yaml
target_depth: 1.0       # 목표 깊이 (m)
depth_kp, ki, kd: 0.8, 0.35, 1.2  # 깊이 PID
depth_offset: 0.0       # 깊이 보정
```

### Thruster Control
```yaml
max_current: 5.0        # 최대 전류 (A)
current_scale: 3.0      # PWM -> 전류 스케일
can_channel: 'can0'     # CAN 채널
```

## 런타임 파라미터 변경

```bash
# 목표 깊이 변경
ros2 param set /wall_following_controller target_depth 2.0

# 벽 거리 변경
ros2 param set /wall_following_controller target_distance 1.5

# 컨트롤러 비활성화
ros2 param set /wall_following_controller enabled false
```

## 시뮬레이션 vs 실제 차이점

| 항목 | 시뮬레이션 | 실제 |
|------|-----------|------|
| DVL | `/{vehicle}/altitude` (Range) | `/dvl/data` (dvl_msgs/DVL) |
| Depth | `/{vehicle}/pressure` (FluidPressure) | `/pressure` (Float64, mbar) |
| Thruster | `/{vehicle}/setpoint/pwm` (Float64MultiArray) | CAN bus (VESC current) |
| 스러스터 수 | 6개 PWM | 6개 VESC (전류 제어) |

## 문제 해결

### CAN 통신 오류
```bash
# CAN 상태 확인
ip -details link show can0

# CAN 재시작
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

### DVL 연결 실패
```bash
# DVL IP 확인
ping 192.168.1.99

# DVL 파라미터 변경
ros2 run dvl_a50 dvl_a50 --ros-args -p dvl_address:='새IP주소'
```

### 깊이 센서 오프셋
```bash
# 수면에서 깊이 보정
ros2 param set /wall_following_controller depth_offset -0.5
```
