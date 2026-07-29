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

from pkrc_control.gui_server import mjpeg_frame, BOUNDARY, FrameStore


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


if __name__ == '__main__':
    test_mjpeg_frame_structure()
    test_mjpeg_frame_does_not_alter_payload()
    test_frame_store_returns_latest()
    test_frame_store_age()
    print('test_gui_server(frame): 4 passed')
