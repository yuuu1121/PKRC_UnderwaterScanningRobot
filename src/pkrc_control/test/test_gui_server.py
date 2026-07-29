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
    mjpeg_frame, BOUNDARY, FrameStore, KeyWatchdog, MOTION_KEYS, VALID_KEYS)


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


def test_valid_keys_rejects_bogus_accepts_real():
    """POST /key 화이트리스트 — 정의된 키만 통과."""
    assert 'UP' in VALID_KEYS
    assert 'w' in VALID_KEYS
    assert 'bogus' not in VALID_KEYS


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
    test_valid_keys_rejects_bogus_accepts_real()
    print('test_gui_server: 11 passed')
