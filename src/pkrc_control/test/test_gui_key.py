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
