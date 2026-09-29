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
import math
import os
import signal
import socket
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
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, FluidPressure, Imu, Range
from std_msgs.msg import String, Float32, Float64MultiArray

# DVL 메시지는 별도 패키지라 빌드돼 있지 않을 수 있다. 조종 노드들도 같은
# 방어를 한다(keyboard_control_teleop.py:60) — 없다고 GUI 전체가 죽으면 안 된다.
try:
    from dvl_msgs.msg import DVL
except ImportError:
    DVL = None

# 프로세스 종료는 이 패키지가 이미 풀어놓은 문제다 — ros2 launch 는 래퍼이고
# 실제 자식이 별도 프로세스 그룹으로 빠져나가므로 단순 kill 이 닿지 않는다.
from pkrc_control.live_tuning import _terminate
from pkrc_control import laser_overlay

# MJPEG multipart 경계 문자열. 브라우저가 프레임 구분에 쓴다.
BOUNDARY = 'pkrcframe'

# 브라우저로 밀어낼 최대 프레임률. 카메라는 약 27fps 로 발행하지만
# 프레임이 약 192KB 라 그대로 흘리면 42Mbps 가 되고, 링크가 포화되면
# 영상이 밀리는 것은 물론 브라우저의 조종 fetch 까지 늦어진다.
# 12fps ≈ 18Mbps — 관제 화면으로 충분히 부드럽고 여유가 생긴다.
# 화질이 아니라 프레임 수만 줄이므로 레이저 관측에는 영향이 없다.
STREAM_MAX_FPS = 12.0

# 누르고 있는 동안만 유효한 이동 키. 이 키를 보낸 뒤 브라우저가 조용해지면
# 통신이 끊긴 것으로 보고 강제 정지시킨다.
# w/s 는 제외 — 목표 수심을 한 번 바꾸는 1회성 키이고(그 뒤는 depth hold 가
# 유지), 홀드 반복 전송이 없어 watchdog 이 정상 조작을 단절로 오판한다.
# r/t/c/x/q 도 같은 이유로 제외.
MOTION_KEYS = frozenset({'UP', 'DOWN', 'LEFT', 'RIGHT', 'a', 'd'})

# 이 시간 동안 이동 키가 갱신되지 않으면 정지시킨다 [초].
KEY_TIMEOUT = 0.5

# 전 축 정지 + 컨트롤러 리셋 키 (두 조종 노드 공통).
# 사용자가 직접 누르는 비상 정지다 — 목표 수심·방위까지 현재값으로 리셋한다.
STOP_KEY = 'x'

# watchdog 이 통신 단절에 발행하는 정지 키. x 와 나눈 이유:
# '사용자가 정지를 눌렀다' 와 '브라우저가 0.5초 조용하다' 는 다른 사건인데
# 같은 x 로 표현하면, x 의 목표 리셋(target_depth = current_depth)이 단절에도
# 적용된다. 그래서 GUI 에서 목표 수심을 잡아두고 yaw 를 한 번 돌린 뒤 손을
# 떼면 0.5초 후 목표가 현재 수심으로 밀렸다. z 는 추력과 적분기만 정리하고
# 목표는 보존한다 — 통신이 복구되면 원래 잡아둔 수심으로 돌아간다.
WATCHDOG_STOP_KEY = 'z'

# POST /key 로 허용하는 키 전체 — 두 조종 노드가 실제로 이해하는 키만 통과시킨다.
# z 는 서버가 발행하는 키라 브라우저 KEYMAP 에는 없지만, publish 경로가
# 같은 검증을 지나므로 여기 포함해야 한다.
VALID_KEYS = frozenset({
    'UP', 'DOWN', 'LEFT', 'RIGHT',
    'w', 's', 'a', 'd', 'r', 't', 'x', 'z', 'q', 'c',
})

# 레이저 카메라 — 토글이 아니다. /image_raw/compressed 를 내는 유일한
# 노드이므로 시작 시 확보한다. exploreHD(/dev/video0) 를 쓰기 때문에
# full_system 의 gscam 과는 동시 실행이 불가능하다(V4L2 배타 open).
CAMERA_CMD = ['ros2', 'launch', 'laser_camera_publisher',
              'laser_camera.launch.py']
CAMERA_TOPIC = '/image_raw/compressed'

# 하방 카메라 — 레이저와 같은 노드를 device/topic 만 바꿔 띄운다
# (downward_camera.launch.py). 레이저와 마찬가지로 GUI 가 확보한다.
DOWNWARD_CMD = ['ros2', 'launch', 'laser_camera_publisher',
                'downward_camera.launch.py']
DOWNWARD_TOPIC = '/downward/image_raw/compressed'

# 화면에 띄우는 카메라. /stream?cam=<키> 로 각각 받아간다.
#
# stellarHD 는 전용 카메라 노드가 없다 — aruco_detector_6dof 가 /dev/video4 를
# 직접 열고(use_direct_camera 기본 True) 프레임을 JPEG 로 되쏜다
# (aruco_detector_6dof.py:270-274). 그래서 별도 프로세스를 띄우지 않고
# 그 토픽을 그대로 구독한다 — 띄웠다간 같은 V4L2 장치를 다퉈 둘 다 죽는다.
#
# 단 그 되쏘기는 기본으로 꺼져 있다(publish_raw_image 기본 False,
# aruco_detector_6dof.py:105). 매 프레임 JPEG 인코딩이 젯슨에 비싸서
# 보는 사람이 있을 때만 켜라는 설계다. GUI 가 볼 때만 켜고 끈다
# (_set_stellar_raw). 실측: 켜면 22.5Hz / 41KB → 7.6Mbps.
CAMERAS = {
    'laser': {
        'label': '레이저 (exploreHD)',
        'topic': CAMERA_TOPIC,
        'qos': 'best_effort',       # laser_camera_publisher 가 그렇게 낸다
    },
    'stellar': {
        'label': 'stellarHD (ArUco)',
        'topic': '/stellarHD/image_raw/compressed',
        'qos': 'reliable',          # aruco_detector 가 기본 QoS 로 낸다(실측)
    },
    'downward': {
        'label': 'Downward Camera',
        'topic': DOWNWARD_TOPIC,
        'qos': 'best_effort',       # 같은 camera_node 라 레이저와 동일
    },
}

# ── 레이저 오버레이 ────────────────────────────────────────────────────
# 검출식은 calibration/calibrate_laser.py 와 같다(laser_overlay.py 참조).
# 기본은 꺼짐 — 꺼져 있으면 아래 _stream_mjpeg 이 JPEG 를 손대지 않으므로
# 이 기능이 붙기 전과 완전히 같은 무손실 passthrough 로 동작한다.
#
# 켜도 영향은 GUI 화면뿐이다. 오버레이는 스트리밍 스레드의 사본에만 그려
# HTTP 응답으로 나가고, ROS 토픽으로 되돌아가지 않는다 — rosbag 에 남는
# 것은 카메라가 발행한 1280x720 원본이다(ros2 bag record -a 는 토픽을
# 직접 구독하므로 이 서버를 아예 거치지 않는다).
#
# 검출 해상도가 왜 노브인가: 젯슨 8코어 실측(디코드+검출+인코드 왕복,
# 12fps 상한 기준 코어 점유율)
#     legacy   @640   26.2 ms   31.5%
#     contrast @640   37.8 ms   45.3%
#     legacy   @1280  47.4 ms   56.9%
#     contrast @1280  90.5 ms  108.6%  ← 한 코어를 넘고 12fps 를 못 맞춘다
# 검출만 축소본에서 하고 그리기는 원본 해상도라, 화면에 나가는 영상은
# 어느 설정에서든 1280x720 이다.
LASER_DET_WIDTH_DEFAULT = 640
LASER_METHODS = ('legacy', 'contrast')

# stellarHD 프레임을 되쏘게 만들 대상 노드와 파라미터.
# 두 측위 모드(localization / aruco 단독) 모두 같은 실행파일이라 노드
# 이름이 같다.
STELLAR_NODE = 'aruco_detector_6dof'
STELLAR_RAW_PARAM = 'publish_raw_image'

# 브라우저의 '이 카메라를 보는 중' 선언이 이 시간 동안 갱신되지 않으면
# 아무도 안 보는 것으로 본다 [초].
#
# 브라우저는 /state 를 10Hz 로 폴링하면서 이 선언을 함께 보낸다. 탭을
# 닫거나 브라우저가 죽으면 폴링이 멈추므로 자연히 만료된다. 2초면
# 일시적인 네트워크 지연으로 영상이 껌뻑이지 않으면서, 탭을 떠난 뒤
# 되쏘기가 오래 켜져 있지도 않다.
ACTIVE_CAM_TIMEOUT = 2.0

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
    'rosbag': {
        'label': 'rosbag 기록',
        # 전체 토픽 기록. 출력 경로(-o)는 build_cmd 가 기록 시작 시각으로
        # 붙인다 — NODE_SPECS 는 정적 테이블이라 여기 둘 수 없다.
        'cmd': ['ros2', 'bag', 'record', '-a'],
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
        # exposure_time=1 (노드 기본 2): 카메라 최소 노출. LED 마커 블룸이
        # 줄어 깜빡임 디코드율이 오른다 — 2026-08-04 무손실 녹화 실측.
        # camera_device: by-path 로 stellarHD 고정 — /dev/videoN 번호는
        # USB 열거 순서에 밀린다 (localization.launch.py 동일).
        'cmd': ['ros2', 'run', 'active_marker', 'aruco_detector_6dof',
                '--ros-args', '-p', 'exposure_time:=1',
                '-p', ('camera_device:=/dev/v4l/by-path/'
                       'platform-3610000.usb-usb-0:2.1.3:1.0-video-index0')],
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

# 노드가 "제대로 켜졌는가" 의 증거가 되는 토픽.
#
# 프로세스 생존(ProcManager.status)만으로는 부족하다 — 장치가 안 붙어 있거나
# 드라이버 초기화가 실패해도 노드 프로세스는 살아서 로그만 뱉는다. 그 노드가
# 실제로 데이터를 내고 있는지가 진짜 헬스다.
#
# qos 는 반드시 퍼블리셔와 맞춰야 한다. 틀리면 노드가 멀쩡해도 한 건도
# 못 받아 UI 가 빨강으로 오진한다 — 이 워크스페이스에서 반복된 함정이다.
#
# stale 은 퍼블리시 주기에 맞춘다. 하나로 통일하면 느린 토픽(LED state 1Hz)이
# 정상인데도 깜빡인다.
HEALTH_TOPICS = {
    'pressure': {'topic': '/bar10xt/pressure', 'type': 'FluidPressure',
                 'qos': 'sensor', 'stale': 2.0},
    'dvl': {'topic': '/dvl/data', 'type': 'DVL',
            'qos': 'sensor', 'stale': 2.0},
    'imu': {'topic': '/imu/data', 'type': 'Imu',
            'qos': 'default', 'stale': 2.0},
    # LED 는 state 발행이 1Hz 라 여유를 둔다.
    'led': {'topic': 'lumen/state', 'type': 'Float32',
            'qos': 'default', 'stale': 3.0},
    # 측위·ArUco 는 마커를 놓치면 정상적으로 발행이 멈춘다. 그때 빨강이
    # 되는 것은 오진이 아니라 "지금 측위가 안 되고 있다" 는 유용한 신호다.
    'localization': {'topic': '/ukfm/odom', 'type': 'Odometry',
                     'qos': 'default', 'stale': 2.0},
    'aruco': {'topic': '/aruco/pose_6dof', 'type': 'PoseStamped',
              'qos': 'default', 'stale': 2.0},
}

# control 그룹 토글의 기본 전류 한계 [A]. 축 특성이 달라 둘로 나눈다 —
# heave 는 부력을 이겨야 하므로 수평축보다 큰 전류가 필요하다.
# 이 값은 노드 자체 기본값(keyboard_control_teleop.py:417-419)과 같은
# 전 출력이다. 물 밖 벤치에서 시험할 때는 GUI 에서 낮춰서 켤 것.
DEFAULT_MAX_CURRENT_HORIZ = 3.0   # surge / sway
DEFAULT_MAX_CURRENT_HEAVE = 5.0   # heave (상하)
# GUI 가 허용하는 전류 한계 범위 [A] — 이 밖의 값(오타 등)은 거부한다.
MAX_CURRENT_RANGE = (0.1, 5.0)

# 조종 노드 키 → ROS 노드 이름 (파라미터 서비스 호출 대상).
# 이름은 각 파일의 super().__init__() 인수와 정확히 일치해야 한다
# (keyboard_control_teleop.py:351, keyboard_control_wall_align.py:408).
# 틀리면 파라미터 서비스가 존재하지 않아 게인 튜닝이 전부 실패한다.
CONTROL_NODE_NAMES = {
    'teleop': 'keyboard_control_teleop',
    'wall_align': 'keyboard_control_wall_align',
}

# NODE_SPECS 키 → 그 노드가 실제로 만드는 실행파일 (패키지, 실행파일) 목록.
# GUI 종료 시 이 표로 잔존 프로세스를 찾아 정리한다. _procs (이번 세션이
# 직접 띄운 Popen) 만 보면, GUI 를 재시작한 순간 이전 세션이 띄운 노드가
# 전부 고아가 되어 Ctrl+C 를 눌러도 아무것도 안 꺼진다.
#
# 각 값은 launch 파일을 직접 읽어 확인했다 (bar10xt.launch.py:26,
# dvl_a50.launch.py:15, microstrain_launch.py:45, lumen_led.launch.py:9,
# localization.launch.py:39/73).
# rosbag 은 ros2 bag record 라 실행파일 이름이 없다 — Popen 경로로만 정리된다.
NODE_EXECUTABLES = {
    'pressure': [('bar10xt_ros2', 'bar10xt_node')],
    'dvl': [('dvl_a50', 'dvl_a50_sensor')],
    'imu': [('microstrain_inertial_driver',
             'microstrain_inertial_driver_node')],
    'led': [('lumen_led', 'lumen_node')],
    'localization': [
        ('active_marker', 'aruco_detector_6dof'),
        ('pkrc_controller', 'ukfm_localization'),
    ],
    'aruco': [('active_marker', 'aruco_detector_6dof')],
    'teleop': [('pkrc_control', 'keyboard_control_teleop')],
    'wall_align': [('pkrc_control', 'keyboard_control_wall_align')],
}

# 프리셋 저장 위치. 소스 트리가 아닌 이유: 실험값이 소스를 오염시키지 않고,
# 패키지를 재빌드해도 살아남아야 한다.
PRESET_DIR = os.path.expanduser('~/.ros/pkrc_presets')

# rosbag 저장 위치. 소스 트리 밖 — 재빌드에도 살아남고 소스를 오염시키지
# 않는다. 하위 디렉터리 이름이 기록 시작 시각이다.
BAG_DIR = os.path.expanduser('~/hero_ws/rosbags')


def _is_number(v) -> bool:
    """JSON 으로 들어온 값이 실제 숫자인지. bool 은 int 의 서브클래스라
    isinstance(True, int) 가 참이므로 따로 제외한다 — 안 그러면
    {"value": true} 가 1.0 으로 조용히 통과해버린다."""
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _pose_dict(pose, frame_id: str = '') -> dict:
    """geometry_msgs/Pose → JSON 으로 보낼 수 있는 평범한 dict.

    ROS 메시지를 캐시에 그대로 넣으면 HTTP 스레드가 rclpy 객체를 만지게
    되고 json.dumps 도 실패한다. 콜백(rclpy 스레드)에서 바로 풀어둔다.

    yaw 만 뽑는 데 scipy 를 끌어오지 않는다 — 쿼터니언에서 yaw 는 atan2
    한 줄이고, 이 노드는 자세를 계산하는 곳이 아니라 표시하는 곳이다.

    ArUco 는 header.frame_id 에 'marker_<ID>' 를 넣는다
    (aruco_detector_6dof.py:948). 여러 마커를 평균한 pose 이고 그중 가장
    큰 마커의 ID 이므로 대표값으로만 보여준다.
    """
    p, q = pose.position, pose.orientation
    yaw = math.degrees(math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    out = {'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw}
    if frame_id.startswith('marker_'):
        out['marker'] = frame_id[len('marker_'):]
    return out


def build_cmd(key: str, horiz: float = None, heave: float = None) -> list:
    """NODE_SPECS[key]['cmd'] 에 control 그룹 전류 한계 오버라이드를 얹은
    argv. NODE_SPECS 는 mutate 하지 않는다 — 새 리스트를 반환한다.

    horiz = surge/sway 공통, heave = 상하. 축 특성이 달라 따로 받는다.
    """
    spec = NODE_SPECS[key]
    cmd = spec['cmd']
    if spec['group'] == 'control':
        h = DEFAULT_MAX_CURRENT_HORIZ if horiz is None else horiz
        v = DEFAULT_MAX_CURRENT_HEAVE if heave is None else heave
        # 반드시 소수점을 붙여야 한다. 노드가 이 파라미터를 DOUBLE 로
        # 선언하므로 '3' 을 주면 ROS 가 INTEGER 로 해석해
        # InvalidParameterTypeException 으로 노드가 즉시 죽는다.
        # JS 의 JSON.stringify(3.0) 은 '3' 이 되므로 브라우저에서 켤 때만
        # 이 문제가 났다 — curl 로 '3.0' 을 쓰면 재현되지 않았다.
        cmd = cmd + [
            '-p', 'max_current_surge:=%.4f' % float(h),
            '-p', 'max_current_sway:=%.4f' % float(h),
            '-p', 'max_current_heave:=%.4f' % float(v),
        ]
    if key == 'rosbag':
        # 기록 시작 시각이 곧 bag 이름 — 언제 찍은 기록인지 경로만 봐도 안다.
        os.makedirs(BAG_DIR, exist_ok=True)
        cmd = cmd + ['-o', os.path.join(BAG_DIR,
                                        time.strftime('%Y%m%d_%H%M%S'))]
    return cmd


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
        """정지가 필요하면 WATCHDOG_STOP_KEY, 아니면 None.

        사용자가 누르는 x 가 아니라 z 를 쓴다 — x 는 목표 수심·방위까지
        현재값으로 리셋하므로, 통신이 잠깐 끊겼다고 조종 목표가 바뀐다.

        한 번 발동하면 disarm 되어 새 이동 키가 올 때까지 재발동하지
        않는다 — 10Hz 로 계속 쏘면 로그가 폭주하고 조종 복귀를 방해한다.
        """
        if not self._armed:
            return None
        if now - self._last_motion > self.timeout:
            self._armed = False
            return WATCHDOG_STOP_KEY
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
        # 수신율 추정용. 직전 도착 간격의 지수이동평균만 들고 있는다 —
        # 관제 화면은 "대략 몇 Hz" 면 충분해서 타임스탬프 링버퍼를 둘
        # 이유가 없고, 상태가 둘뿐이라 락 범위도 그대로다.
        self._ema_dt = 0.0

    # EMA 계수. 낮을수록 부드럽고 반응이 느리다. 0.2 면 약 5샘플이면
    # 새 주기에 수렴한다 — 10Hz 토픽에서 0.5초.
    _EMA_ALPHA = 0.2

    def put(self, value, now: float = None):
        with self._lock:
            t = time.time() if now is None else now
            if self._value is not None:
                dt = t - self._stamp
                # 같은 시각에 두 번 들어오면 나눗셈이 폭발한다.
                if dt > 1e-6:
                    self._ema_dt = (dt if self._ema_dt <= 0.0 else
                                    self._ema_dt + self._EMA_ALPHA
                                    * (dt - self._ema_dt))
            self._value = value
            self._stamp = t

    def get(self, now: float = None):
        with self._lock:
            if self._value is None:
                return None
            t = time.time() if now is None else now
            if t - self._stamp > self.stale_sec:
                return None
            return self._value

    def health(self, now: float = None):
        """(살아있나, 마지막 수신 이후 초, 추정 Hz).

        get() 과 달리 값이 stale 해도 age 를 돌려준다 — UI 가 '꺼짐' 과
        '켜졌는데 데이터가 3초째 없음' 을 구분해 보여줘야 하기 때문이다.
        한 번도 못 받았으면 age 는 None.
        """
        with self._lock:
            if self._value is None:
                return False, None, None
            t = time.time() if now is None else now
            age = t - self._stamp
            hz = (1.0 / self._ema_dt) if self._ema_dt > 1e-6 else None
            return age <= self.stale_sec, age, hz


class ProcManager:
    """launch/run 프로세스를 띄우고 내린다.

    상호배타 그룹을 강제하는 것이 이 클래스의 핵심 책임이다. 같은 V4L2
    장치나 같은 CAN 버스를 두 노드가 잡으면 조용히 실패하거나 로봇이
    오작동하므로, 조합 자체를 만들 수 없게 한다.
    """

    def __init__(self, logger, on_before_stop=None, live_nodes=None):
        self._logger = logger
        self._procs = {}          # key → Popen
        self._lock = threading.Lock()
        # control 그룹을 내리기 전에 정지 키를 보내기 위한 훅
        self._on_before_stop = on_before_stop
        # 내가 띄우지 않은 노드(터미널의 ros2 run 등)도 알아야 한다.
        # 모르면 끄지도 못하고 인터록으로 교체하지도 못해, 같은 CAN 버스를
        # 쓰는 조종 노드가 방치된다. () -> set(노드 이름) 콜백.
        self._live_nodes = live_nodes

    def _alive(self, key: str) -> bool:
        p = self._procs.get(key)
        return p is not None and p.poll() is None

    def _external(self, key: str) -> bool:
        """내가 띄우지 않았지만 실제로 돌고 있는 노드인가.

        ROS 노드 등록(get_node_names)은 프로세스가 죽은 뒤에도 DDS 디스커버리
        캐시에 몇 초 남는다. 그것만 믿으면 모드를 전환한 직후 옛 노드가
        여전히 '켜짐' 으로 보여 두 조종 버튼이 동시에 켜지고, 게인 패널이
        엉뚱한 노드를 조회한다(실측 3초간). 그래서 실제 프로세스 존재를
        먼저 확인하고, 이름 등록은 보조로만 쓴다.
        """
        if self._alive(key) or self._live_nodes is None:
            return False
        name = CONTROL_NODE_NAMES.get(key)
        if name is None:
            return False          # 조종 노드만 이름으로 조회할 수 있다
        # 프로세스가 실제로 있는지가 먼저다 — 없으면 등록 잔상이다.
        if not self._proc_exists(name):
            return False
        try:
            return name in self._live_nodes()
        except Exception:
            return False

    @classmethod
    def _proc_exists(cls, node_name: str) -> bool:
        """해당 조종 노드의 실행파일 프로세스가 실제로 있는지.
        pgrep -f 는 부분일치라 형제 노드를 오검출하므로 쓰지 않는다."""
        return bool(cls._pids_of(node_name))

    def _present(self, key: str) -> bool:
        """어떤 경로로든 실제로 돌고 있는가 (표시·인터록·정지 판정용)."""
        return self._alive(key) or self._external(key)

    def _kill_external(self, key: str):
        """외부에서 띄운 조종 노드를 프로세스 이름으로 찾아 정리한다.
        ProcManager 는 Popen 핸들이 없으므로 pkill 에 해당하는 일을 직접 한다.
        같은 CAN 버스를 두 노드가 잡으면 로봇이 오작동하므로 방치할 수 없다."""
        name = CONTROL_NODE_NAMES.get(key)
        if name is None:
            return
        if self._on_before_stop is not None:
            self._on_before_stop()      # 먼저 전 축 정지
            time.sleep(0.3)
        # pkill -f 를 쓰면 안 된다: 전체 명령줄 부분일치라 teleop 패턴이
        # wall_align 프로세스까지 잡는 것을 실측했다(모드 전환이 안 되던
        # 원인). 실행파일 경로가 정확히 일치하는 PID 만 골라 죽인다.
        killed = 0
        for pid in self._pids_of(name):
            try:
                os.kill(pid, signal.SIGTERM)
                killed += 1
            except OSError:
                pass
        if killed:
            self._logger.info(
                f'외부에서 띄운 {name} 종료 {killed}개 '
                f'(GUI 가 소유하지 않은 노드)')

    @staticmethod
    def _pids_of(node_name: str, package: str = 'pkrc_control') -> list:
        """해당 노드의 PID 목록. /proc/<pid>/cmdline 을 직접 읽어 실행파일
        경로가 정확히 일치하는 토큰만 인정한다 — 부분일치로 형제 노드를
        죽이거나 오검출하지 않기 위함.

        package 기본값이 pkrc_control 인 것은 조종 노드 조회(_proc_exists,
        _kill_external)가 전부 이 패키지라서다. 종료 정리는 센서 패키지도
        훑어야 하므로 NODE_EXECUTABLES 가 패키지를 함께 넘긴다."""
        exe = 'lib/' + package + '/' + node_name
        out = []
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            try:
                with open('/proc/' + entry + '/cmdline', 'rb') as f:
                    argv = f.read().split(b'\x00')
            except OSError:
                continue
            for a in argv:
                if a.decode('utf-8', 'replace').endswith(exe):
                    out.append(int(entry))
                    break
        return out

    def status(self) -> dict:
        with self._lock:
            # 죽은 프로세스 정리
            for k in [k for k in self._procs if not self._alive(k)]:
                self._procs.pop(k, None)
            # 외부 기동 노드도 '켜짐' 으로 보고한다 — 실제로 돌고 있는데
            # 버튼이 꺼져 보이면 눌러서 중복 기동을 시도하게 된다.
            return {k: self._present(k) for k in NODE_SPECS}

    def start(self, key: str, horiz: float = None, heave: float = None):
        spec = NODE_SPECS.get(key)
        if spec is None:
            return False, f'알 수 없는 노드: {key}'

        with self._lock:
            if self._present(key):
                return True, f'{spec["label"]} 이미 실행 중'

            # 상호배타: 같은 그룹의 형제를 먼저 내린다
            for sib in group_siblings(key):
                if self._external(sib):
                    self._logger.info(
                        f'{NODE_SPECS[sib]["label"]} (외부 기동) 종료 — '
                        f'{spec["label"]} 과 같은 그룹({spec["group"]})')
                    self._kill_external(sib)
                    continue
                if self._alive(sib):
                    self._logger.info(
                        f'{NODE_SPECS[sib]["label"]} 종료 — '
                        f'{spec["label"]} 과 같은 그룹'
                        f'({spec["group"]}) 이라 동시 실행 불가')
                    self._stop_locked(sib)

            cmd = build_cmd(key, horiz, heave)

            try:
                # stdout/stderr 를 버리면 노드가 죽은 이유를 알 수 없다.
                # /tmp 에 노드별 로그를 남겨 사인을 확인할 수 있게 한다.
                logpath = f'/tmp/pkrc_node_{key}.log'
                try:
                    logf = open(logpath, 'w')
                except OSError:
                    logf = subprocess.DEVNULL
                p = subprocess.Popen(
                    cmd,
                    stdout=logf, stderr=subprocess.STDOUT,
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
            # 실행 명령 전체를 남긴다 — 파라미터 오버라이드(예: aruco 의
            # exposure_time:=1)가 실제로 적용됐는지 이 로그로 확인한다.
            self._logger.info(
                f'{spec["label"]} 시작 (pid {p.pid}): {" ".join(cmd)}')
            return True, f'{spec["label"]} 시작'

    def _stop_locked(self, key: str):
        """락을 이미 쥔 상태에서 호출. control 그룹은 먼저 정지시킨다.

        control 그룹은 이 0.3초 sleep 동안 self._lock 을 계속 쥔다 — 그래서
        같은 시간 동안 다른 HTTP 스레드의 /state·/node 호출이 block 된다.
        일부러다: 이 sleep 은 STOP 이 VESC 에 실제로 정착할 시간을 버는
        것이고, 그 사이에 다른 start() 가 끼어들어 종료 중인 노드와
        경합하면 안 되기 때문이다. UI 지연보다 정지 안전이 우선이라 락을
        풀지 않는다.
        """
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
                if self._external(key):
                    # 내가 띄운 게 아니어도 끈다 — CAN 버스를 공유하는
                    # 조종 노드를 방치하면 다음 기동과 충돌한다.
                    self._kill_external(key)
                    return True, f'{spec["label"]} 종료 (외부 기동)'
                return True, f'{spec["label"]} 실행 중 아님'
            self._stop_locked(key)
            self._logger.info(f'{spec["label"]} 종료')
            return True, f'{spec["label"]} 종료'

    def stop_all(self):
        """GUI 종료 시 전체 정리.

        _procs 만 내리면 부족하다: GUI 를 재시작하면 이전 세션이 띄운 노드는
        Popen 핸들이 없어 영원히 고아가 되고, Ctrl+C 를 눌러도 센서와 조종
        노드가 그대로 살아 CAN·V4L2 를 계속 점유한다. 그래서 Popen 을 먼저
        내린 뒤, NODE_EXECUTABLES 로 잔존 프로세스를 이름으로 한 번 더 훑는다.
        """
        with self._lock:
            # control 그룹이 있으면 _stop_locked 가 정지 키를 먼저 보낸다.
            for key in list(self._procs):
                self._stop_locked(key)
            self._kill_leftovers_locked()

    def _kill_leftovers_locked(self):
        """Popen 으로 못 잡는 잔존 노드를 실행파일 이름으로 정리한다.

        조종 노드가 남아 있으면 VESC 가 마지막 전류 명령을 유지하므로,
        죽이기 전에 정지 키를 보내고 정착할 시간을 준다 — _stop_locked 가
        control 그룹에 하는 것과 같은 이유다.
        """
        targets = []      # (pid, 표시이름)
        control_left = False
        for key, execs in NODE_EXECUTABLES.items():
            for package, exe in execs:
                for pid in self._pids_of(exe, package):
                    targets.append((pid, exe))
                    if NODE_SPECS.get(key, {}).get('group') == 'control':
                        control_left = True
        if not targets:
            return

        if control_left and self._on_before_stop is not None:
            self._on_before_stop()
            time.sleep(0.3)

        for sig in (signal.SIGTERM, signal.SIGKILL):
            alive = []
            for pid, exe in targets:
                try:
                    os.kill(pid, sig)
                    alive.append((pid, exe))
                except (ProcessLookupError, PermissionError):
                    pass
            if not alive:
                return
            self._logger.info(
                '잔존 노드 정리 (%s): %s'
                % ('SIGTERM' if sig == signal.SIGTERM else 'SIGKILL',
                   ', '.join(sorted({e for _, e in alive}))))
            # SIGTERM 후 종료를 기다린다. rclpy 노드는 보통 1초 안에 내려간다.
            deadline = time.time() + 3.0
            while time.time() < deadline:
                if not any(os.path.exists('/proc/%d' % p) for p, _ in alive):
                    return
                time.sleep(0.1)
            targets = [(p, e) for p, e in alive
                       if os.path.exists('/proc/%d' % p)]
            if not targets:
                return


class _Handler(BaseHTTPRequestHandler):
    """HTTP 요청 처리. self.server.node 로 GuiServer 에 접근한다."""

    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        """기본 stderr 접근 로그를 끈다 — 10Hz 폴링이라 로그가 폭주한다.

        단 /params 와 /node 는 남긴다. 브라우저가 게인 패널을 못 여는 문제를
        추적할 때 '브라우저가 실제로 요청을 보냈는가' 를 알 수 없으면
        서버·브라우저 어느 쪽 문제인지 구분할 수 없다. 이 두 경로는
        폴링이 아니라 사용자 조작 시에만 불리므로 로그가 폭주하지 않는다.
        """
        try:
            path = self.path
        except AttributeError:
            return
        if path.startswith('/params') or path.startswith('/node'):
            self.server.node.get_logger().info(f'HTTP {path}')

    def _send(self, code, ctype, body: bytes, no_cache=False):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        if no_cache:
            # gui.html 을 브라우저가 캐시하면 서버 코드를 고쳐도 옛 화면이
            # 그대로 뜬다 — 이 세션에서 '수정했는데 안 된다' 의 실제 원인.
            self.send_header('Cache-Control',
                             'no-store, no-cache, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
        self.end_headers()
        self.wfile.write(body)

    @staticmethod
    def _overlay_opts(query: dict):
        """/stream 쿼리에서 레이저 오버레이 설정을 뽑는다.

        laser 인수가 없거나 아는 방식이 아니면 None — 오버레이 없이 간다.
        브라우저가 보낸 값을 그대로 믿지 않는다. 이 세션에서 정수/실수
        타입 하나가 노드를 즉사시킨 적이 있어(커밋 8670896) 숫자는 전부
        범위까지 확인하고 통과시킨다.
        """
        method = (query.get('laser') or [''])[0]
        if method not in LASER_METHODS:
            return None

        def num(key, default, lo, hi, cast):
            try:
                v = cast((query.get(key) or [''])[0])
            except (TypeError, ValueError):
                return default
            return v if lo <= v <= hi else default

        return {
            'method': method,
            # 0 은 '전부 통과' 라 의미가 없고, 상한은 legacy 신호 스케일(~200)
            # 을 넘지 않게 잡는다.
            'sig_thr': num('thr', None, 0.1, 250.0, float),
            # 160 밑으로는 라인이 뭉개져 검출 자체가 무의미해진다.
            'det_width': num('det', LASER_DET_WIDTH_DEFAULT, 160, 4096, int),
            'ransac': (query.get('ransac') or ['0'])[0] == '1',
        }

    def _stream_mjpeg(self, node, cam: str = 'laser', overlay: dict = None):
        """multipart/x-mixed-replace 로 프레임을 계속 밀어낸다.

        overlay 가 주어지면 프레임마다 레이저 검출을 그려 보낸다. None 이면
        JPEG 바이트를 손대지 않는다 — 그때는 이 기능이 붙기 전과 완전히
        같은 무손실 passthrough 다.

        이 응답은 스레드 하나를 계속 붙잡는다 — 그래서 서버가
        ThreadingHTTPServer 여야 한다. Content-Length 를 줄 수 없으므로
        HTTP/1.0 으로 응답해 연결 종료로 끝을 알린다.

        지연 대책: write() 는 프레임(약 192KB)을 다 보낼 때까지 블로킹한다.
        그 사이 카메라는 계속 새 프레임을 덮어쓰므로, 전송이 끝난 뒤 그냥
        다음 루프로 가면 브라우저는 항상 "전송이 끝난 시점"의 영상을 본다.
        링크가 느릴수록 이 뒤처짐이 쌓여 눈에 보이는 딜레이가 된다.
        그래서 (1) 전송 직후 최신 프레임으로 건너뛰고(밀린 것은 버린다),
        (2) 목표 fps 로 상한을 둬 대역폭이 링크를 포화시키지 않게 한다.
        포화되면 브라우저 쪽에서 조종 fetch 도 함께 늦어진다.
        """
        self.protocol_version = 'HTTP/1.0'
        self.send_response(200)
        self.send_header(
            'Content-Type',
            f'multipart/x-mixed-replace; boundary={BOUNDARY}')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()

        # 소켓 송신 버퍼가 크면 write() 가 커널에만 쌓아두고 즉시 반환해,
        # 실제로는 오래된 프레임이 버퍼에 줄 서 있게 된다(딜레이의 원인).
        # 버퍼가 작아야 write() 가 링크 속도를 그대로 반영해 블로킹하고,
        # 그래야 아래 skip-to-latest 가 밀린 프레임을 버린다.
        #
        # 512KB 는 레이저 프레임(~40KB) 12장 ≈ 1초치가 커널에 줄 서는
        # 크기였다 — 느린 링크(WiFi·SSH 터널)에서 딜레이가 계속 쌓이는
        # 원인(실측). 64KB 면 1~2장: 링크가 느리면 전송 fps 가 떨어질 뿐
        # 화면은 항상 최신에 가깝다.
        try:
            self.connection.setsockopt(
                socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)
        except OSError:
            pass

        # stellarHD 는 보는 사람이 있을 때만 되쏘게 한다. 이 스레드가
        # 살아있는 동안이 곧 '보는 중' 이므로 여기서 증감시킨다.
        node.viewer_enter(cam)
        interval = 1.0 / STREAM_MAX_FPS
        last_stamp = 0.0
        next_send = 0.0
        # 프레임이 없어도 연결 생존을 확인해야 하는 시각.
        #
        # 이 루프는 write() 가 실패해야만 끊김을 안다. 그런데 프레임이 안
        # 오면 continue 로 돌아가 write() 를 아예 호출하지 않아서, 클라이언트가
        # 이미 나갔는데도 영원히 돌고 finally 에 닿지 못한다 — 그러면
        # viewer_exit 이 안 불려 시청자 수가 줄지 않고, 되쏘기가 켜진 채로
        # 남는다(실측: 탭을 떠나도 viewers 가 2, 3 으로 쌓였다).
        #
        # 레이저는 항상 프레임이 있어 드러나지 않지만, stellar 는 되쏘기가
        # 꺼지는 순간 프레임이 멈추므로 정확히 이 함정에 빠진다. 그래서
        # 프레임이 없을 때는 주기적으로 경계만 보내 생존을 확인한다.
        # multipart 경계는 브라우저가 무시하고, 연결이 끊겼으면 여기서
        # 예외가 나 정상 종료 경로를 탄다.
        next_probe = time.time() + 1.0
        try:
            while True:
                now = time.time()
                if now < next_send:
                    time.sleep(min(0.005, next_send - now))
                    continue
                data, stamp = node.get_frame(cam)
                if data is None or stamp == last_stamp:
                    if now >= next_probe:
                        self.wfile.write(f'--{BOUNDARY}\r\n'.encode())
                        self.wfile.flush()
                        next_probe = now + 1.0
                    time.sleep(0.005)      # 아직 새 프레임이 없다
                    continue
                next_probe = now + 1.0     # 실제 프레임이 갔으면 미룬다
                if overlay is not None:
                    # 검출 실패나 디코딩 오류로 화면이 멈추면 안 된다.
                    # render() 는 디코딩 실패 시 원본을 그대로 돌려주지만,
                    # 그 밖의 예외까지 스트림을 죽이게 두지는 않는다.
                    try:
                        data = laser_overlay.render(data, **overlay)
                    except Exception as e:      # noqa: BLE001
                        node.get_logger().warn(
                            f'레이저 오버레이 실패 — 원본으로 계속: {e}',
                            throttle_duration_sec=5.0)
                self.wfile.write(mjpeg_frame(data))
                self.wfile.flush()
                last_stamp = stamp
                # 전송에 걸린 시간을 반영해 다음 전송 시각을 잡는다.
                # 전송이 interval 보다 오래 걸렸으면 즉시 다음 프레임으로
                # 가되, 그때 get_frame() 이 최신을 주므로 밀린 프레임은
                # 자동으로 버려진다.
                next_send = max(time.time(), now + interval)
        except (BrokenPipeError, ConnectionResetError, OSError):
            # 브라우저가 탭을 닫거나 새로고침한 정상 종료
            pass
        finally:
            # 예외로 빠져나가도 반드시 줄어야 한다 — 안 그러면 아무도 안
            # 보는데 검출기가 계속 JPEG 를 인코딩한다.
            node.viewer_exit(cam)

    def do_GET(self):
        node = self.server.node
        if self.path in ('/', '/index.html'):
            try:
                with open(node.html_path, 'rb') as f:
                    self._send(200, 'text/html; charset=utf-8', f.read(),
                               no_cache=True)
            except OSError as e:
                self._send(500, 'text/plain; charset=utf-8',
                           f'gui.html 을 읽을 수 없음: {e}'.encode())
        elif self.path.startswith('/stream'):
            # /stream?cam=stellar — 인수가 없으면 레이저(기존 동작 유지).
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            cam = (q.get('cam') or ['laser'])[0]
            if cam not in CAMERAS:
                self._send(404, 'text/plain; charset=utf-8',
                           f'알 수 없는 카메라: {cam}'.encode())
                return
            # /stream?cam=laser&laser=legacy&thr=12&det=640&ransac=1
            # laser 인수가 없거나 아는 방식이 아니면 오버레이 없이 간다 —
            # 오타로 CPU 를 먹는 경로가 조용히 켜지지 않게 한다.
            self._stream_mjpeg(node, cam, self._overlay_opts(q))
        elif self.path.startswith('/state'):
            # /state?cam=stellar — 브라우저가 폴링할 때마다 '지금 보고 있는
            # 카메라' 를 함께 알린다. 별도 요청을 만들지 않는 이유는 이미
            # 10Hz 로 오는 요청에 얹으면 하트비트가 공짜로 생기기 때문이다.
            # 탭을 닫으면 폴링이 멈추고 선언이 만료돼 되쏘기가 꺼진다.
            from urllib.parse import parse_qs, urlparse
            cam = (parse_qs(urlparse(self.path).query).get('cam') or [''])[0]
            if cam in CAMERAS:
                node.set_active_cam(cam)
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
        """요청 본문을 JSON 으로 파싱. 실패하면 None.

        Content-Length 가 없는 요청(chunked transfer 등)도 처리한다 — 전에는
        헤더가 없으면 즉시 None 을 돌려 400 을 냈고, 브라우저는 msg 없는
        응답을 받아 alert(undefined) 를 띄웠다.
        """
        try:
            raw = self.headers.get('Content-Length')
            if raw is not None:
                n = int(raw)
                if n <= 0:
                    return None
                return json.loads(self.rfile.read(n))
            # chunked 또는 길이 미상 — 한 줄씩 모아 읽는다.
            if self.headers.get('Transfer-Encoding', '').lower() == 'chunked':
                chunks = []
                while True:
                    size = int(self.rfile.readline().strip() or b'0', 16)
                    if size == 0:
                        self.rfile.readline()      # 트레일러 CRLF
                        break
                    chunks.append(self.rfile.read(size))
                    self.rfile.readline()
                return json.loads(b''.join(chunks))
            return None
        except (ValueError, TypeError, OSError):
            return None

    def do_POST(self):
        node = self.server.node
        body = self._read_json()
        if body is None:
            self._send(400, 'application/json',
                       json.dumps({'ok': False,
                                   'msg': 'JSON 본문을 읽을 수 없습니다',
                                   'error': 'JSON 본문이 필요합니다'}).encode())
            return

        if self.path == '/key':
            key = body.get('key', '')
            if not key:
                self._send(400, 'application/json',
                           json.dumps({'ok': False,
                                       'msg': 'key 가 비었습니다',
                                       'error': 'key 가 비었습니다'}).encode())
                return
            if key not in VALID_KEYS:
                self._send(400, 'application/json',
                           json.dumps({'ok': False,
                                       'msg': f'알 수 없는 키: {key!r}',
                                       'error': f'알 수 없는 키: {key!r}'}).encode())
                return
            node.publish_key(key)
            self._send(200, 'application/json',
                       json.dumps({'ok': True}).encode())

        elif self.path == '/node':
            key = body.get('key', '')
            on = bool(body.get('on'))
            # 전류 한계는 축별로 둘. horiz = surge/sway, heave = 상하.
            # 두 필드를 같은 규칙(숫자 + 범위)으로 검증한다.
            caps = {}
            for field in ('max_current_horiz', 'max_current_heave'):
                v = body.get(field)
                if v is None:
                    caps[field] = None
                    continue
                # 타입이 틀리면(문자열 등) 범위 비교(<=) 자체가 예외를
                # 던지므로 범위 검사보다 먼저 확인한다.
                if not _is_number(v):
                    self._send(400, 'application/json', json.dumps({
                        'ok': False,
                        'msg': f'{field} 는 숫자여야 합니다',
                        'nodes': node.procs.status(),
                    }).encode())
                    return
                lo, hi = MAX_CURRENT_RANGE
                if not (lo <= v <= hi):
                    self._send(400, 'application/json', json.dumps({
                        'ok': False,
                        'msg': f'{field} 는 {lo}~{hi}A 범위여야 합니다',
                        'nodes': node.procs.status(),
                    }).encode())
                    return
                caps[field] = v
            ok, msg = (node.procs.start(key, caps['max_current_horiz'],
                                        caps['max_current_heave']) if on
                       else node.procs.stop(key))
            self._send(200, 'application/json', json.dumps({
                'ok': ok, 'msg': msg, 'nodes': node.procs.status(),
            }).encode())

        elif self.path == '/cam':
            # 브라우저가 지금 보고 있는 카메라를 알린다.
            #
            # 스트림 연결이 끊기는 것만으로 판단할 수 없다: img.src 를 바꿔도
            # 브라우저가 이전 multipart 연결을 즉시 끊지 않는다(실측 — 탭을
            # 바꿔도 viewers 가 1 로 유지됐다). multipart/x-mixed-replace 는
            # '끝나지 않는 응답' 이라 브라우저가 그대로 붙들고 있을 수 있다.
            # 그래서 화면이 무엇을 보는지는 화면이 직접 말하게 한다.
            cam = body.get('cam', '')
            if cam not in CAMERAS:
                self._send(400, 'application/json', json.dumps({
                    'ok': False, 'msg': f'알 수 없는 카메라: {cam}'}).encode())
                return
            node.set_active_cam(cam)
            self._send(200, 'application/json',
                       json.dumps({'ok': True}).encode())

        elif self.path == '/clienterr':
            # 브라우저 JS 오류를 서버 로그로 남긴다 — 원격 브라우저의
            # 콘솔을 볼 수 없어 화면 문제를 진단하지 못하는 일을 막는다.
            node.get_logger().error(
                f'브라우저 오류: {str(body.get("msg", ""))[:300]}')
            self._send(200, 'application/json',
                       json.dumps({'ok': True}).encode())

        elif self.path == '/led':
            # Lumen LED 밝기. 노드가 자체적으로 0~1 클램프를 하지만
            # (lumen_node.py:141) 범위 밖 값을 그대로 흘리면 사용자가
            # 왜 안 밝아지는지 알 수 없으므로 여기서 거절한다.
            value = body.get('value')
            if not _is_number(value):
                self._send(400, 'application/json', json.dumps({
                    'ok': False, 'msg': 'value 는 숫자여야 합니다',
                }).encode())
                return
            if not (0.0 <= value <= 1.0):
                self._send(400, 'application/json', json.dumps({
                    'ok': False, 'msg': 'LED 밝기는 0.0~1.0 범위여야 합니다',
                }).encode())
                return
            node.set_led(float(value))
            self._send(200, 'application/json',
                       json.dumps({'ok': True}).encode())

        elif self.path == '/param':
            value = body.get('value', 0.0)
            # set_param() 내부가 float(value)/int(...) 를 예외 처리 없이
            # 호출한다 — 문자열 등 숫자가 아닌 값은 여기서 미리 걸러
            # 500 대신 400 을 낸다(POST /node 의 max_current 와 같은 문제).
            if not _is_number(value):
                self._send(400, 'application/json', json.dumps({
                    'ok': False, 'msg': 'value 는 숫자여야 합니다',
                }).encode())
                return
            ok, msg = node.set_param(
                body.get('node', ''), body.get('name', ''), value)
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
        # 카메라가 둘이라 FrameStore 도 둘이다. 키는 CAMERAS 와 같다.
        self.frames = {k: FrameStore() for k in CAMERAS}
        for key, spec in CAMERAS.items():
            # QoS 를 퍼블리셔와 정확히 맞춰야 한다. laser_camera_publisher 는
            # BEST_EFFORT / KEEP_LAST / depth=1 로 퍼블리시하므로
            # (camera_node.py:59-63) 기본 RELIABLE 로 구독하면 한 프레임도
            # 받지 못한다. 반대로 stellarHD 는 aruco_detector 가 RELIABLE 로
            # 내므로(실측) BEST_EFFORT 로 받으면 안 된다. 같은 함정이 이
            # 패키지의 압력·DVL 구독에도 있다.
            if spec['qos'] == 'best_effort':
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1)
            else:
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1)
            self.create_subscription(
                CompressedImage, spec['topic'],
                self._image_cb(key), qos)

        # stellarHD 되쏘기는 보는 사람이 있을 때만 켠다. 시청자 수는 HTTP
        # 스레드들이 증감시키므로 락으로 보호한다 — KeyWatchdog 과 달리
        # 여러 필드를 함께 갱신해서 GIL 만으로는 부족하다.
        self._stellar_lock = threading.Lock()
        # 시청자를 정수가 아니라 스레드 집합으로 센다 — viewer_exit 누락으로
        # 카운트가 영원히 어긋나는 문제를 구조적으로 없애기 위함이다.
        self._stellar_threads = set()
        # 브라우저가 '지금 이걸 보고 있다' 고 선언한 카메라와 그 시각.
        # 되쏘기 on/off 의 실제 판단 근거다(_stellar_wanted 참조).
        self._active_cam = 'laser'
        self._active_cam_at = 0.0
        self._stellar_raw_on = False
        # 1Hz 로 '아무도 안 보는데 켜져 있나' 를 정리한다.
        self.create_timer(1.0, self._stellar_tick)

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

        # ── Lumen LED 밝기 ─────────────────────────────────────────────
        # 게인 튜닝(파라미터 서비스) 경로를 쓸 수 없다 — LED 노드는
        # lumen/brightness 토픽으로 Float32 0.0~1.0 을 받는다.
        # lumen/state 로 현재 값을 되돌려주므로 그것을 읽어 슬라이더에
        # 실제 밝기를 표시한다(노드가 꺼져 있으면 stale → None).
        # 구독 자체는 아래 헬스 테이블이 만든다 — 같은 토픽을 두 번 받지
        # 않도록 그 캐시를 밝기 표시에도 그대로 쓴다.
        self.led_pub = self.create_publisher(Float32, 'lumen/brightness', 10)

        # ── 센서 헬스 + 측위 실시간 값 ─────────────────────────────────
        self._setup_health()

        # ── 노드 프로세스 관리 ─────────────────────────────────────────
        self.procs = ProcManager(self.get_logger(),
                                 on_before_stop=self._emit_stop,
                                 live_nodes=lambda: set(self.get_node_names()))
        # 파라미터 서비스 클라이언트 캐시 (노드명 → {서비스명: client})
        self._param_clients = {}
        # get_max_current() 결과 캐시 (노드명 → (조회 시각, 결과)), 2초 TTL
        self._max_current_cache = {}
        os.makedirs(PRESET_DIR, exist_ok=True)

        # 레이저 카메라 확보 — 이미 돌면 그걸 쓰고, 없으면 띄우고 소유한다.
        self._owns_camera = False
        self._camera_procs = []
        self.create_timer(1.0, self._ensure_camera_once)

        self._httpd = ThreadingHTTPServer((host, port), _Handler)
        self._httpd.node = self
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

        self.get_logger().info(
            f'웹 GUI 서버 시작 — http://{host}:{port} '
            f'(같은 망의 브라우저에서 접속)')

    def _set_bool_param(self, node_name: str, name: str, value: bool):
        """bool 파라미터 하나를 설정한다.

        set_param() 을 쓸 수 없다 — 그쪽은 list_tunable() 을 거치는데,
        게인 슬라이더용이라 숫자형(DOUBLE/INTEGER)만 남기고 걸러낸다.
        publish_raw_image 는 bool 이라 그 목록에 없어서 '튜닝 대상이
        아닙니다' 로 거절된다(실측). 여기서는 타입 조회 없이 바로 BOOL 로
        설정한다 — 대상 파라미터가 bool 인 것이 이미 확정이기 때문이다.
        """
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = bool(value)
        sc = self._client(node_name, SetParameters, 'set_parameters')
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name=name, value=pv)]
        res = self._call(sc, req)
        if res is None:
            return False, f'{node_name} 에 연결할 수 없습니다'
        if not res.results or not res.results[0].successful:
            reason = (res.results[0].reason if res.results else '알 수 없음')
            return False, f'{name} 설정 거절됨: {reason}'
        return True, f'{name} = {value}'

    def set_active_cam(self, cam: str):
        """브라우저가 지금 보고 있다고 선언한 카메라를 기록한다.

        스트림 연결 유무보다 이쪽을 신뢰한다 — 브라우저가 img.src 를 바꿔도
        이전 multipart 연결을 즉시 끊지 않아서, 연결만 보면 탭을 떠난 것을
        영영 모른다(실측). /state 폴링과 같은 주기로 갱신되므로, 탭을 닫거나
        브라우저가 죽으면 갱신이 멈추고 _stellar_tick 이 정리한다.
        """
        with self._stellar_lock:
            self._active_cam = cam
            self._active_cam_at = time.time()
        if cam == 'stellar':
            self._set_stellar_raw(True)

    def _stellar_wanted(self) -> bool:
        """지금 stellar 되쏘기가 필요한가.

        '브라우저가 stellar 를 보고 있다고 최근에 말했는가' 하나만 본다.
        선언이 끊기면(탭 전환·탭 닫힘·브라우저 종료) 곧 False 가 된다.
        """
        with self._stellar_lock:
            if self._active_cam != 'stellar':
                return False
            return (time.time() - self._active_cam_at) < ACTIVE_CAM_TIMEOUT

    def viewer_enter(self, cam: str):
        """스트림 시청 시작. stellar 되쏘기를 켠다.

        GUI 화면은 /cam 으로 직접 선언하므로 이 경로에 의존하지 않는다.
        여기서 켜는 것은 curl 처럼 선언 없이 /stream?cam=stellar 로 바로
        붙는 경우를 위한 폴백이다 — 그 경우 _active_cam 이 갱신되지 않아
        _stellar_tick 이 곧 끄지만, 최소한 첫 프레임은 나온다.

        시청자를 정수로 세지 않고 '살아있는 스레드 객체' 집합으로 관리한다.
        증감 방식은 exit 이 한 번이라도 누락되면 카운트가 영원히 어긋난 채
        회복되지 않는다(실측: 탭 전환을 반복하니 viewers 가 2, 3 으로
        쌓였고 되쏘기가 꺼지지 않았다). 스레드 객체를 들고 있으면
        is_alive() 로 실제 생존을 다시 확인할 수 있어 유령이 남지 않는다.
        """
        if cam != 'stellar':
            return
        with self._stellar_lock:
            self._stellar_threads.add(threading.current_thread())
            first = len(self._stellar_threads) == 1
        if first:
            self._set_stellar_raw(True)

    def viewer_exit(self, cam: str):
        """스트림 시청 종료. 실제로 끄는 것은 _stellar_tick 이 한다 —
        새로고침처럼 끊고 바로 다시 붙는 경우에 껐다 켜기를 반복하지
        않기 위해 1초 유예를 둔다."""
        if cam != 'stellar':
            return
        with self._stellar_lock:
            self._stellar_threads.discard(threading.current_thread())

    @property
    def _stellar_viewers(self) -> int:
        """실제로 살아있는 stellar 시청 스레드 수.

        죽은 스레드는 여기서 걸러낸다 — viewer_exit 이 누락돼도(스레드가
        예외 없이 사라지는 경우) 다음 조회에서 자동으로 정리된다.
        """
        with self._stellar_lock:
            dead = {t for t in self._stellar_threads if not t.is_alive()}
            self._stellar_threads -= dead
            return len(self._stellar_threads)

    def _set_stellar_raw(self, on: bool):
        """aruco_detector 의 프레임 되쏘기를 켜고 끈다.

        기본이 꺼짐인 이유는 매 프레임 JPEG 인코딩이 젯슨에 비싸기
        때문이다(aruco_detector_6dof.py:105, 650-655). 검출기의 본업은
        60fps 마커 검출이므로, 보는 사람이 있을 때만 켠다.

        노드가 없으면 조용히 실패한다 — 측위를 끄면 당연히 없고, 그때
        에러를 띄우면 사용자가 고칠 수 없는 경고만 보게 된다.
        """
        # 락은 상태 확인·갱신에만 쓰고, 서비스 호출 중에는 절대 쥐지 않는다.
        # 파라미터 설정은 ROS 왕복이라 최대 2초 걸리는데, 그동안 락을 쥐면
        # viewer_enter/exit 과 _stellar_tick 이 전부 멈춰 관제 화면이 밀린다.
        with self._stellar_lock:
            if self._stellar_raw_on == on:
                return
            # 같은 전이를 두 스레드가 동시에 시도하지 않도록 먼저 찜한다.
            self._stellar_raw_on = on
        ok, msg = self._set_bool_param(
            STELLAR_NODE, STELLAR_RAW_PARAM, on)
        if ok:
            self.get_logger().info(
                f'stellarHD 영상 되쏘기 {"켬" if on else "끔"}')
            return
        # 실패했으면 찜해둔 상태를 되돌린다 — 안 그러면 실제로는 꺼져 있는데
        # 켜진 줄 알고 다시는 켜려 하지 않는다.
        with self._stellar_lock:
            self._stellar_raw_on = not on
        if on:
            # 끄기 실패는 조용히 넘긴다(노드가 이미 죽었으면 끌 것도 없다).
            self.get_logger().warn(f'stellarHD 영상을 켤 수 없음: {msg}')

    def _restore_stellar_raw_subprocess(self):
        """종료 시 되쏘기를 끈다. ROS 컨텍스트가 이미 죽어 있으므로
        `ros2 param set` 을 별도 프로세스로 부른다.

        실패해도 조용히 넘어간다 — 검출기가 이미 죽었으면 끌 대상이 없고,
        종료 중에 사용자가 고칠 수 없는 에러를 띄울 이유도 없다.
        """
        try:
            r = subprocess.run(
                ['ros2', 'param', 'set', '/' + STELLAR_NODE,
                 STELLAR_RAW_PARAM, 'false'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                timeout=10)
            ok = (r.returncode == 0
                  and b'successful' in r.stdout.lower())
            # 로거도 컨텍스트에 묶여 있어 종료 시점엔 안전하지 않다.
            print(f'[gui_server] stellarHD 되쏘기 끔: '
                  f'{"성공" if ok else "실패(검출기가 이미 없을 수 있음)"}',
                  flush=True)
        except (OSError, subprocess.SubprocessError):
            pass

    def _stellar_tick(self):
        """보는 사람이 없어지면 되쏘기를 끈다.

        브라우저가 탭을 닫으면 스트림 스레드가 죽으면서 시청자 수가 준다.
        그때 파라미터를 되돌려 놓지 않으면 아무도 안 보는데 검출기가 계속
        JPEG 를 인코딩한다.
        """
        # 판단 근거는 '브라우저가 stellar 를 보고 있다고 최근에 말했는가' 다.
        # 스트림 스레드 수는 보조로만 쓴다 — 브라우저가 이전 연결을 안 끊어
        # 스레드가 남아 있어도, 화면이 레이저를 보고 있다면 꺼야 한다.
        #
        # 끄기는 반드시 별도 스레드에서 한다. 이 타이머 콜백은 메인 spin
        # 스레드에서 돌기 때문에, 여기서 _call() 로 서비스 응답을 기다리면
        # 그 응답을 처리할 executor 를 자기가 붙잡고 있어 항상 타임아웃한다
        # — 실패로 오인해 1초마다 2초씩 재시도하며 executor 를 영원히
        # 블로킹하고, 그동안 모든 카메라 스트림이 0.5fps 로 떨어졌다(실측).
        # _set_stellar_raw 는 락으로 상태를 먼저 찜하므로 스레드가 겹쳐도
        # 같은 전이를 두 번 호출하지 않는다.
        if self._stellar_raw_on and not self._stellar_wanted():
            threading.Thread(target=self._set_stellar_raw,
                             args=(False,), daemon=True).start()

    def _setup_health(self):
        """HEALTH_TOPICS 를 실제 구독으로 만든다.

        타입 이름을 문자열로 둔 이유: 테이블이 '무엇을 감시하는가' 만
        말하게 하고, 클래스 해석과 QoS 선택은 여기 한 곳에서 처리하기
        위함이다. 새 센서를 붙일 때 테이블 한 줄이면 끝난다.
        """
        types = {
            'FluidPressure': FluidPressure, 'Imu': Imu, 'Range': Range,
            'Float32': Float32, 'Odometry': Odometry,
            'PoseStamped': PoseStamped, 'DVL': DVL,
        }
        # 기본 QoS 는 RELIABLE depth 10 — 이 워크스페이스의 조종 노드들이
        # imu 를 받는 방식과 같다.
        qos_map = {'sensor': qos_profile_sensor_data, 'default': 10}

        self.health = {}
        for key, spec in HEALTH_TOPICS.items():
            msg_type = types.get(spec['type'])
            cache = TopicCache(stale_sec=spec['stale'])
            self.health[key] = cache
            if msg_type is None:
                # dvl_msgs 미빌드 등 — 캐시는 만들되 구독하지 않는다.
                # UI 는 '데이터 없음'(빨강)으로 그린다.
                self.get_logger().warn(
                    f'{spec["topic"]} 헬스 감시 불가 — '
                    f'{spec["type"]} 메시지를 import 할 수 없습니다')
                continue
            self.create_subscription(
                msg_type, spec['topic'],
                self._health_cb(key, spec['type']), qos_map[spec['qos']])

        # LED 밝기 표시는 헬스 캐시를 그대로 쓴다(같은 lumen/state 토픽).
        self.tc_led = self.health['led']

    def _health_cb(self, key: str, type_name: str):
        """헬스 캐시에 넣을 값을 만드는 콜백을 생성한다.

        캐시에는 ROS 메시지가 아니라 평범한 dict/float 만 넣는다 —
        HTTP 스레드가 rclpy 객체를 만지지 않게 하고, /state 가 그대로
        JSON 직렬화할 수 있게 하기 위함이다.
        """
        cache = self.health[key]
        if type_name == 'Odometry':
            return lambda m: cache.put(_pose_dict(m.pose.pose))
        if type_name == 'PoseStamped':
            return lambda m: cache.put(_pose_dict(m.pose, m.header.frame_id))
        if type_name == 'Float32':
            return lambda m: cache.put(float(m.data))
        # 나머지 센서는 값이 아니라 '왔다' 는 사실만 필요하다.
        return lambda m: cache.put(True)

    def _image_cb(self, key: str):
        """카메라가 만든 JPEG 바이트를 그대로 저장하는 콜백을 만든다.
        디코딩하지 않는다 — 퍼블리셔가 이미 JPEG 로 준다."""
        store = self.frames[key]
        return lambda msg: store.put(bytes(msg.data), time.time())

    def get_frame(self, key: str = 'laser'):
        return self.frames[key].get()

    def frame_age(self, key: str = 'laser') -> float:
        return self.frames[key].age()

    def set_led(self, value: float):
        """LED 밝기를 lumen/brightness 로 발행한다 (0.0~1.0)."""
        msg = Float32()
        msg.data = float(value)
        self.led_pub.publish(msg)

    def publish_key(self, key: str):
        """키를 /gui/key 로 발행하고 watchdog 을 갱신한다."""
        msg = String()
        msg.data = key
        self.key_pub.publish(msg)
        self.watchdog.touch(key, time.time())

    def _emit_stop(self):
        """조종 노드를 내리기 전 전 축 정지 + 컨트롤러 리셋.

        예외를 삼키는 이유: 이 훅은 shutdown() 의 정리 경로에서도 불리는데,
        그 시점의 rclpy 컨텍스트는 이미 무효라 publish 가 "context is
        invalid" 로 죽는다(파일 하단 shutdown() 주석의 실측과 같은 원인).
        여기서 예외가 새면 stop_all() 이 중간에 끊겨 뒤따르는 노드들이
        정리되지 않은 채 남는다 — 정지 명령 실패보다 그쪽이 더 위험하다.
        어차피 곧 SIGTERM 으로 노드를 내리므로 추력은 멈춘다.
        """
        try:
            msg = String()
            msg.data = STOP_KEY
            self.key_pub.publish(msg)
            self.get_logger().info('조종 노드 종료 전 정지 명령 발행')
        except Exception as e:
            # 로거도 컨텍스트에 의존하므로 실패할 수 있다.
            try:
                self.get_logger().warn(f'정지 명령 발행 실패 (종료 중): {e}')
            except Exception:
                pass

    def _ensure_camera_once(self):
        """카메라 퍼블리셔들을 한 번만 확보한다. 타이머는 즉시 자기를 끈다."""
        for t in list(self.timers):
            if t.callback == self._ensure_camera_once:
                self.destroy_timer(t)

        for label, cmd, topic in (('레이저', CAMERA_CMD, CAMERA_TOPIC),
                                  ('하방', DOWNWARD_CMD, DOWNWARD_TOPIC)):
            # 이미 퍼블리셔가 있으면 남이 띄운 것 — 소유하지 않는다.
            if self.count_publishers(topic) > 0:
                self.get_logger().info(
                    f'{topic} 퍼블리셔가 이미 있음 — 그것을 사용하고 '
                    f'gui_server 종료 시에도 살려둡니다')
                continue
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    stdin=subprocess.DEVNULL, start_new_session=True)
                self._camera_procs.append(proc)
                self._owns_camera = True
                self.get_logger().info(
                    f'{label} 카메라 시작 (pid {proc.pid}) — '
                    f'gui_server 종료 시 함께 정리합니다')
            except OSError as e:
                self.get_logger().error(
                    f'{label} 카메라 실행 실패: {e} — 영상이 나오지 않습니다')

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
        이미 spin 중이다).

        서비스 준비 대기는 0.25초만 한다. 2초를 기다리면 /params 응답이
        2초 블로킹되고, 그 사이 사용자가 버튼을 다시 눌러 노드가 재기동되면
        브라우저는 이미 사라진 노드의 실패 응답을 받는다 — 재시도가 영원히
        헛도는 원인이었다(실측). 서비스가 아직 없으면 즉시 실패로 돌려주고,
        브라우저가 400ms 후 다시 물어보게 하는 편이 훨씬 빠르게 붙는다.
        """
        if not client.wait_for_service(timeout_sec=0.25):
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

        2초 캐시: /state 가 10Hz 로 폴링되므로 캐시가 없으면 파라미터
        서비스를 10Hz 로 호출하게 된다.
        """
        cache = self._max_current_cache.get(node_name)
        if cache is not None and time.time() - cache[0] < 2.0:
            return cache[1]

        names = ['max_current_surge', 'max_current_sway',
                 'max_current_heave']
        gc = self._client(node_name, GetParameters, 'get_parameters')
        req = GetParameters.Request()
        req.names = names
        res = self._call(gc, req, timeout=0.5)
        if res is None or len(res.values) != len(names):
            result = None
        else:
            ms, mw, mh = (v.double_value for v in res.values)
            result = [ms, ms, mw, mw, mh, mh]

        self._max_current_cache[node_name] = (time.time(), result)
        return result

    # ─── 프리셋 ─────────────────────────────────────────────────────
    def preset_path(self, name: str) -> str:
        """경로 성분이 섞인 이름은 거부한다.

        예전엔 '/' 를 '_' 로 조용히 바꿔치기했는데, 그러면 사용자가
        '../../etc/evil' 을 입력해도 실제로는 'evil.yaml' 에 저장되면서
        메시지는 원래 입력 그대로를 보여줘 오해를 준다(경로 탈출 자체는
        안 됐지만 사용자가 의도한 이름과 다른 곳에 조용히 저장됨). 그래서
        디렉터리 구분자가 섞이면 아예 거부해 사용자가 알아채게 한다.
        """
        safe = name.strip()
        if not safe or safe.startswith('.') or '/' in safe or '\\' in safe:
            raise ValueError('프리셋 이름에 경로 구분자나 점(.)으로 '
                             '시작하는 이름은 쓸 수 없습니다')
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
        # 메시지엔 실제로 저장된 파일명(정제된 이름)을 쓴다 — 원본 입력을
        # 그대로 보여주면 사용자가 의도한 이름과 실제 저장 위치가 달라 보여도
        # 눈치채지 못한다.
        return True, f'{os.path.basename(path)} 저장 ({len(params)}개 파라미터)'

    def preset_load(self, name: str, node_name: str):
        try:
            path = self.preset_path(name)
            with open(path) as f:
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
        return True, f'{os.path.basename(path)} 적용 ({ok}개 파라미터)'

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

    def sensor_health(self, nodes: dict) -> dict:
        """센서별 {state, age, hz} — state 는 'off' | 'nodata' | 'ok'.

        색 판정을 브라우저가 아니라 여기서 끝내는 이유: '초록인가' 는
        도메인 규칙이고, 판정에 필요한 stale 임계값은 서버에만 있다
        (HEALTH_TOPICS). 서버가 결론만 주면 규칙이 두 곳으로 흩어지지 않고
        JS 는 색만 칠하면 된다.

        nodes 는 ProcManager.status() — 프로세스가 도는가.
        health 캐시는 그 노드가 실제로 데이터를 내는가. 이 둘이 다르기
        때문에 이 기능이 필요하다. 프로세스는 살아 있는데 장치가 안 붙어
        있으면 노드는 로그만 뱉으며 멀쩡히 떠 있다.
        """
        out = {}
        for key, cache in self.health.items():
            alive, age, hz = cache.health()
            # 데이터가 오면 프로세스 장부와 무관하게 초록이다. 센서는
            # ProcManager 가 외부 기동을 감지하지 못하므로(_external 은
            # CONTROL_NODE_NAMES, 즉 조종 노드만 본다) 터미널이나
            # full_system.launch.py 로 띄운 센서는 nodes[key] 가 False 다.
            # 그때 회색으로 그리면 "데이터는 오는데 왜 꺼져 보이지" 하며
            # 버튼을 눌러 중복 기동을 시도하게 된다.
            if alive:
                state = 'ok'
            elif nodes.get(key):
                # 프로세스는 도는데 데이터가 없다 — 이 기능이 존재하는
                # 이유인 상태다. 장치 미연결·드라이버 초기화 실패·QoS
                # 불일치가 전부 여기로 온다.
                state = 'nodata'
            else:
                state = 'off'
            out[key] = {
                'state': state,
                'age': None if age is None else round(age, 1),
                'hz': None if hz is None else round(hz, 1),
            }
        return out

    def state_snapshot(self) -> dict:
        """브라우저에 보낼 현재 상태 전체."""
        age = self.frame_age()
        nodes = self.procs.status()
        # 활성 조종 노드가 있으면 그 노드의 실제 전류 한계를 읽는다 —
        # max_current_* 는 런타임에 오버라이드될 수 있어 UI 의 하드코딩
        # 기본값만으로는 포화가 숨겨질 수 있다 (예: 1.0A 캡 운용 시).
        #
        # ProcManager 의 status() 만으로 게이트를 걸면 gui_server 가 직접
        # 띄우지 않은 조종 노드(예: 터미널에서 ros2 run 으로 띄운 경우)를
        # 놓친다 — 그래서 노드 이름이 실제로 존재하는지도 함께 본다.
        # get_node_names() 는 로컬 캐시 조회라 서비스 왕복보다 훨씬 싸다.
        # 어느 조종 노드가 살아있는지 브라우저에 알려준다 — 게인 패널이
        # 이 이름으로 파라미터를 조회한다.
        # nodes 는 ProcManager.status() 결과이고, 그 _present() 가 이미
        # '내가 띄운 것 + 외부 기동' 을 프로세스 실체로 판정한다. 여기서
        # get_node_names() 를 다시 보면 DDS 등록 잔상 때문에 모드를 전환한
        # 뒤에도 옛 노드 이름이 몇 초간 남아, 게인 패널이 죽은 노드를
        # 조회해 실패한다(실측). 그래서 nodes 만 신뢰한다.
        # control_node 는 프로세스 존재만으로 즉시 정한다. get_max_current 는
        # 파라미터 서비스를 쓰므로 노드가 막 떴을 때 최대 2초 블로킹되는데,
        # 그것 때문에 control_node 보고가 늦어지면 브라우저가 옛 상태(null)를
        # 보고 재시도를 포기한다(실측한 버그). 그래서 순서를 분리한다.
        control_node = None
        for key, node_name in CONTROL_NODE_NAMES.items():
            if nodes.get(key):
                control_node = node_name
                break
        # 전류 한계는 부가 정보다 — 실패하면 None 이고 JS 가 기본값으로 폴백한다.
        max_current = (self.get_max_current(control_node)
                       if control_node else None)
        sensors = self.sensor_health(nodes)
        return {
            # 센서별 3색 상태. 색 판정을 서버에서 끝내는 이유는
            # sensor_health() 주석 참조.
            'sensors': sensors,
            # 측위·ArUco 실시간 값. 값이 stale 하면 None 이라 UI 가
            # 옛 좌표를 살아있는 것처럼 보여주지 않는다.
            'pose': {
                'ukfm': self.health['localization'].get(),
                'aruco': self.health['aruco'].get(),
            },
            'camera': {
                # 1초 넘게 프레임이 없으면 카메라 노드가 죽은 것으로 본다.
                # 검은 화면만 보여주면 원인을 알 수 없으므로 명시한다.
                'alive': age < 1.0,
                'age': None if age == float('inf') else round(age, 2),
            },
            # 카메라별 생존. 'camera' 는 레이저 전용으로 남겨둔다 —
            # 헤더 문구 등 기존 JS 가 그대로 쓰고 있다.
            'cameras': {
                k: {'alive': self.frame_age(k) < 1.0,
                    'age': (None if self.frame_age(k) == float('inf')
                            else round(self.frame_age(k), 2))}
                for k in CAMERAS
            },
            # stellarHD 되쏘기 상태. 이 파라미터는 GUI 가 남의 노드를
            # 건드리는 유일한 곳이라, 지금 켜져 있는지·왜 켜져 있는지를
            # 밖에서 볼 수 있어야 한다. 안 그러면 '아무도 안 보는데 켜져
            # 있다' 를 진단할 방법이 없다.
            'stellar_raw': {
                'on': self._stellar_raw_on,
                'viewers': self._stellar_viewers,
            },
            'yaw': self.tc_yaw.get(),
            'depth': self.tc_depth.get(),
            'thrusters': self.tc_thrust.get(),
            'wall': self.tc_wall.get(),
            'wall_mode': self.tc_wall_mode.get(),
            'nodes': nodes,
            'max_current': max_current,
            'control_node': control_node,
            # LED 노드가 보고하는 실제 밝기. 노드가 없으면 None →
            # 브라우저가 슬라이더를 비활성으로 그린다.
            'led': self.tc_led.get(),
        }

    def shutdown(self):
        # 내가 켠 stellarHD 되쏘기는 내가 끈다 — 안 그러면 GUI 를 내린 뒤에도
        # 검출기가 아무도 안 보는 프레임을 계속 JPEG 로 인코딩한다.
        #
        # 여기서 ROS 서비스를 쓸 수 없다. rclpy 는 SIGINT 를 받으면
        # KeyboardInterrupt 를 던지기 *전에* 컨텍스트를 무효화해서, 이
        # 시점의 서비스 호출은 "rcl node's context is invalid" 로 죽는다
        # (실측). 직접 spin 을 돌려도 마찬가지다 — 컨텍스트 자체가 없다.
        # 그래서 ROS 를 안 거치는 subprocess 로 되돌린다. 이 파일이 노드
        # 기동·종료에 이미 subprocess 를 쓰는 것과 같은 이유다.
        #
        # 각 단계를 따로 감싸는 이유: 앞 단계가 던지면 뒤 단계가 통째로
        # 건너뛰어진다. 실제로 노드 정리(stop_all)가 가장 중요한데 그게
        # 마지막에 가까우므로, 앞의 정리가 실패해도 반드시 도달해야 한다.
        def _quiet(what, fn):
            try:
                fn()
            except Exception as e:
                try:
                    self.get_logger().warn(f'{what} 실패 (종료 계속): {e}')
                except Exception:
                    pass

        if self._stellar_raw_on:
            _quiet('stellarHD 되쏘기 복구', self._restore_stellar_raw_subprocess)
        _quiet('HTTP 서버 종료', self._httpd.server_close)
        _quiet('노드 정리', self.procs.stop_all)
        # 내가 띄운 카메라만 정리한다 — 남이 띄운 것은 건드리지 않는다.
        for proc in getattr(self, '_camera_procs', []):
            _quiet('카메라 정리', lambda p=proc: _terminate(p))


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
    # watchdog 은 x 가 아니라 z 를 쏴야 한다 — x 면 목표 수심이 리셋된다.
    assert wd.check(now=100.6) == WATCHDOG_STOP_KEY, 'watchdog 미발동'
    assert WATCHDOG_STOP_KEY != STOP_KEY, 'watchdog 키가 수동 정지키와 같음'
    assert WATCHDOG_STOP_KEY in VALID_KEYS, 'watchdog 키가 VALID_KEYS 에 없음'
    assert wd.check(now=101.0) is None, 'watchdog 중복 발동'
    wd.touch('r', now=200.0)
    assert wd.check(now=201.0) is None, '비이동 키에 발동'
    print('selftest: watchdog OK')

    # 인터록: 같은 그룹은 서로 형제, 다른 그룹·무그룹은 형제 없음
    assert group_siblings('localization') == ['aruco'], 'video4 인터록 오류'
    assert group_siblings('teleop') == ['wall_align'], 'control 인터록 오류'
    assert group_siblings('pressure') == [], '무그룹에 형제가 있음'
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

    # build_cmd: control 그룹만 전류 한계 오버라이드가 붙는지, NODE_SPECS
    # 원본은 mutate 되지 않는지.
    before = list(NODE_SPECS['teleop']['cmd'])
    cmd = build_cmd('teleop', horiz=1.0, heave=2.0)
    assert 'max_current_surge:=1.0' in ' '.join(cmd), 'teleop 전류 오버라이드 누락'
    assert 'max_current_sway:=1.0' in ' '.join(cmd), 'teleop 전류 오버라이드 누락'
    assert 'max_current_heave:=2.0' in ' '.join(cmd), 'teleop 전류 오버라이드 누락'
    assert NODE_SPECS['teleop']['cmd'] == before, 'NODE_SPECS 가 mutate 됨'
    cmd_default = build_cmd('teleop')
    assert f'max_current_surge:={DEFAULT_MAX_CURRENT_HORIZ}' in ' '.join(cmd_default), \
        '기본 수평 전류 한계 누락'
    assert f'max_current_heave:={DEFAULT_MAX_CURRENT_HEAVE}' in ' '.join(cmd_default), \
        '기본 상하 전류 한계 누락'
    cmd_sensor = build_cmd('pressure', horiz=1.0, heave=2.0)
    assert 'max_current' not in ' '.join(cmd_sensor), \
        '비-control 노드에 전류 오버라이드가 붙음'
    print('selftest: build_cmd OK')

    # preset_path: 경로 구분자가 섞인 이름은 거부, 정상 이름은 PRESET_DIR
    # 아래로만 간다. preset_path 는 self 를 쓰지 않는 순수 로직이라
    # unbound 로 바로 호출해도 안전하다.
    p = GuiServer.preset_path(None, 'test1')
    assert p == os.path.join(PRESET_DIR, 'test1.yaml'), 'preset_path 정상 케이스 오류'
    for bad in ('../../etc/evil', 'a/b', 'a\\b', '.hidden', ''):
        try:
            GuiServer.preset_path(None, bad)
            raise AssertionError(f'{bad!r} 가 거부되지 않음')
        except ValueError:
            pass
    print('selftest: preset_path OK')

    # TopicCache: stale 판정과 수신율 추정. now 를 주입해 시계에 의존하지
    # 않게 한다(sleep 하는 테스트는 느리고 불안정하다).
    tc = TopicCache(stale_sec=1.0)
    assert tc.health(now=0.0) == (False, None, None), '빈 캐시가 살아있다고 보고'
    tc.put('v', now=100.0)
    alive, age, hz = tc.health(now=100.5)
    assert alive and abs(age - 0.5) < 1e-9, 'stale 이내인데 죽었다고 판정'
    assert hz is None, '샘플 1개로 Hz 를 추정함'
    assert tc.get(now=100.5) == 'v', 'stale 이내 값이 안 나옴'
    assert tc.get(now=102.0) is None, 'stale 인데 값이 나옴'
    # stale 해도 age 는 돌려줘야 한다 — '꺼짐' 과 '켜졌는데 무데이터' 를
    # 구분하는 근거가 이 age 다.
    alive, age, hz = tc.health(now=102.0)
    assert not alive and abs(age - 2.0) < 1e-9, 'stale 후 age 가 사라짐'
    # 10Hz 로 넣으면 EMA 가 10Hz 근처로 수렴해야 한다.
    tc2 = TopicCache(stale_sec=1.0)
    for i in range(40):
        tc2.put(i, now=200.0 + i * 0.1)
    _, _, hz = tc2.health(now=200.0 + 39 * 0.1)
    assert hz is not None and abs(hz - 10.0) < 0.5, f'Hz 추정 오류: {hz}'
    # 같은 시각 중복 입력이 0 나눗셈을 일으키지 않아야 한다.
    tc2.put('dup', now=200.0 + 39 * 0.1)
    print('selftest: TopicCache OK')

    # sensor_health 3색 판정: 프로세스 생존과 데이터 도착의 4개 조합.
    # 이 표가 화면 색의 유일한 근거라 네 칸을 전부 고정한다.
    class _FakeCache:
        def __init__(self, alive): self._alive = alive
        def health(self, now=None):
            return (True, 0.1, 10.0) if self._alive else (False, None, None)

    fake = GuiServer.__new__(GuiServer)      # __init__ 없이 메서드만 쓴다
    fake.health = {'imu': _FakeCache(True), 'dvl': _FakeCache(False)}
    # 데이터가 오면 프로세스 장부와 무관하게 ok — 외부 기동 센서를
    # 회색으로 그리지 않기 위한 규칙이다.
    got = fake.sensor_health({'imu': True, 'dvl': True})
    assert got['imu']['state'] == 'ok', '데이터가 오는데 ok 가 아님'
    assert got['dvl']['state'] == 'nodata', '프로세스만 살아있는데 nodata 가 아님'
    got = fake.sensor_health({'imu': False, 'dvl': False})
    assert got['imu']['state'] == 'ok', '외부 기동 센서가 ok 가 아님'
    assert got['dvl']['state'] == 'off', '프로세스도 데이터도 없는데 off 가 아님'
    assert got['imu']['hz'] == 10.0, 'hz 가 전달되지 않음'
    print('selftest: sensor_health OK')

    # _pose_dict: 쿼터니언 → yaw, marker_<ID> 파싱. ROS 메시지가 아니라
    # 같은 필드를 가진 가짜 객체로 검증한다(rclpy 없이 돌아야 한다).
    class _V:
        def __init__(self, **kw): self.__dict__.update(kw)

    # z 축 90° 회전: q = (0, 0, sin45, cos45)
    r2 = 2 ** -0.5
    pose = _V(position=_V(x=1.0, y=2.0, z=3.0),
              orientation=_V(x=0.0, y=0.0, z=r2, w=r2))
    d = _pose_dict(pose, 'marker_7')
    assert abs(d['yaw'] - 90.0) < 1e-6, f'yaw 변환 오류: {d["yaw"]}'
    assert (d['x'], d['y'], d['z']) == (1.0, 2.0, 3.0), '위치가 어긋남'
    assert d['marker'] == '7', 'marker ID 파싱 실패'
    # UKFM 은 frame_id 를 주지 않는다 — marker 키가 없어야 한다.
    assert 'marker' not in _pose_dict(pose), 'frame_id 없이 marker 가 생김'
    # 회전이 없으면 yaw 0
    ident = _V(position=_V(x=0.0, y=0.0, z=0.0),
               orientation=_V(x=0.0, y=0.0, z=0.0, w=1.0))
    assert abs(_pose_dict(ident)['yaw']) < 1e-9, '단위 쿼터니언 yaw 가 0 이 아님'
    print('selftest: _pose_dict OK')

    # 헬스 테이블과 노드 테이블의 키가 어긋나면 sensor_health 가 항상
    # nodes.get()==None 을 받아 외부 기동 센서를 영원히 off 로 그린다.
    unknown = set(HEALTH_TOPICS) - set(NODE_SPECS)
    assert not unknown, f'HEALTH_TOPICS 에 NODE_SPECS 없는 키: {unknown}'
    print('selftest: HEALTH_TOPICS 키 OK')

    # 시청자 집계: viewer_exit 이 누락돼도 죽은 스레드는 자동 정리돼야 한다.
    # 정수 증감 방식은 exit 한 번만 놓쳐도 영원히 어긋난 채 회복되지 않아
    # 되쏘기가 켜진 상태로 남았다(실측). 그래서 스레드 집합으로 바꿨다.
    fake = GuiServer.__new__(GuiServer)
    fake._stellar_lock = threading.Lock()
    fake._stellar_threads = set()
    fake._stellar_raw_on = False
    fake._set_stellar_raw = lambda on: None     # ROS 호출은 건너뛴다

    assert fake._stellar_viewers == 0, '초기 시청자가 0 이 아님'
    done = threading.Event()

    def _watcher(exit_properly: bool):
        fake.viewer_enter('stellar')
        done.wait(2.0)
        if exit_properly:
            fake.viewer_exit('stellar')

    # 1) 정상 종료: exit 을 부르고 끝난다
    t1 = threading.Thread(target=_watcher, args=(True,))
    # 2) 비정상: exit 을 못 부르고 스레드가 사라진다(실제로 났던 상황)
    t2 = threading.Thread(target=_watcher, args=(False,))
    t1.start(); t2.start()
    time.sleep(0.2)
    assert fake._stellar_viewers == 2, \
        f'시청 중 집계 오류: {fake._stellar_viewers}'
    done.set()
    t1.join(3.0); t2.join(3.0)
    # exit 을 안 부른 t2 도 죽었으므로 0 이어야 한다 — 유령이 남으면 안 된다.
    assert fake._stellar_viewers == 0, \
        f'죽은 스레드가 유령으로 남음: {fake._stellar_viewers}'
    # stellar 가 아닌 카메라는 집계 대상이 아니다
    fake.viewer_enter('laser')
    assert fake._stellar_viewers == 0, 'laser 가 stellar 집계에 들어감'
    print('selftest: 시청자 집계 OK')

    # _is_number: /node 의 max_current, /param 의 value 가 공유하는 검사.
    # 문자열·bool·리스트는 거부하고 int/float 만 통과해야 한다.
    assert _is_number(1.0) and _is_number(99) and _is_number(0), \
        '정상 숫자가 거부됨'
    assert not _is_number('99'), '숫자 문자열이 통과됨'
    assert not _is_number(True) and not _is_number(False), \
        'bool 이 숫자로 통과됨 (isinstance(bool, int) 함정)'
    assert not _is_number(None) and not _is_number([1.0]), \
        'None/리스트가 통과됨'
    print('selftest: _is_number OK')


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
    # SIGTERM 은 기본 동작이 '즉시 종료' 라 finally 가 돌지 않는다. 그러면
    # stellarHD 되쏘기를 끄지 못해 아무도 안 보는데 검출기가 계속 JPEG 를
    # 인코딩한다. 예외로 바꿔 아래 finally 를 반드시 타게 한다.
    #
    # SIGHUP 도 같이 잡는 이유(실측): 터미널 창을 그냥 닫으면 ros2 launch 는
    # SIGHUP 을 받고 죽지만, 자식인 이 노드에게는 SIGTERM 을 전달하지 못한다.
    # 그래서 SIGHUP 을 안 잡으면 gui_server 와 그것이 띄운 조종 노드가
    # 통째로 살아남아 CAN 을 계속 점유한다 — Ctrl+C 는 정상 정리되는데
    # 창 닫기만 노드가 남는 이유였다.
    def _on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGHUP, _on_term)
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
