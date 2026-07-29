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

from pkrc_control.gui_server import (
    mjpeg_frame, BOUNDARY, FrameStore, KeyWatchdog, MOTION_KEYS,
    TopicCache, NODE_SPECS, group_siblings, CONTROL_NODE_NAMES)


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
        {'UP', 'DOWN', 'LEFT', 'RIGHT', 'a', 'd'})


def test_watchdog_ignores_depth_keys():
    """w/s 는 1회성 수심 변경 키라 감시 대상이 아니다 — 홀드 반복이 없다."""
    wd = KeyWatchdog(timeout=0.5)
    wd.touch('w', now=100.0)
    assert wd.check(now=101.0) is None

    wd.touch('s', now=100.0)
    assert wd.check(now=101.0) is None


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
    test_watchdog_ignores_depth_keys()
    test_topic_cache_stale()
    test_topic_cache_overwrites()
    test_node_specs_shape()
    test_excluded_cameras_absent()
    test_group_siblings_video4()
    test_group_siblings_control()
    test_group_siblings_none_for_ungrouped()
    test_control_group_has_exactly_two()
    test_control_node_names_match_source()
    print('test_gui_server: 18 passed')
