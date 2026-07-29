#!/usr/bin/env python3
"""개별 스러스터 대화형 키보드 테스트 — 숫자로 스러스터를 고르고
a/s/d 로 전류를 실시간 조절한다. ROS 불필요, python-can 만 사용.

키 매핑
  1~6 : 스러스터 선택 (전환 시 0 A 부터 시작)
  a   : 전류 -0.1 A
  s   : 정지 (0 A)
  d   : 전류 +0.1 A
  q   : 종료 (전 채널 정지 후 빠져나감, Ctrl-C 동일)

⚠ 프로펠러가 실제로 회전한다 — 기체를 물 밖에서 고정하고 실행할 것.
⚠ teleop 노드가 떠 있으면 명령이 충돌한다 — 반드시 종료 후 실행할 것.
"""

import select
import sys
import termios
import time
import tty

import can

# keyboard_control_robust_update.py 의 vesc_ids 와 동일 매핑
THRUSTERS = {
    '1': (0x151, 'surge_left'),
    '2': (0x152, 'surge_right'),
    '3': (0x153, 'sway_left (front)'),
    '4': (0x154, 'sway_right (rear)'),
    '5': (0x155, 'heave_up'),
    '6': (0x156, 'heave_down'),
}
MAX_TEST_CURRENT = 2.0   # 진단용 상한 — teleop 수평 스러스터 한계와 동일
CURRENT_STEP = 0.1
TX_PERIOD = 0.02         # 50 Hz 재전송 — teleop 과 동일 (VESC 워치독 대비)


def send_current(bus, can_id, current):
    """keyboard_control_robust_update.py send_current 와 동일 인코딩."""
    current = max(-5.0, min(5.0, current))
    scaled = int(current * 1000)
    if scaled < 0:
        scaled &= 0xFFFFFFFF
    data = [(scaled >> 24) & 0xFF, (scaled >> 16) & 0xFF,
            (scaled >> 8) & 0xFF, scaled & 0xFF]
    bus.send(can.Message(arbitration_id=can_id, data=data,
                         is_extended_id=True, dlc=4))


def main():
    print(__doc__)
    print('스러스터 매핑:')
    for k, (cid, name) in THRUSTERS.items():
        print(f'  {k}: {hex(cid)}  {name}')
    print()

    try:
        bus = can.interface.Bus(channel='can0', interface='socketcan')
    except Exception as e:
        sys.exit(f'CAN 초기화 실패: {e}\n  → ip link show can0 로 상태 확인')

    settings = termios.tcgetattr(sys.stdin)
    selected = None      # (키, can_id, 이름)
    current = 0.0
    tx_errors = 0
    # cbreak: raw 와 달리 Ctrl-C(SIGINT) 가 살아 있어 비상 정지 경로 유지
    tty.setcbreak(sys.stdin.fileno())
    try:
        next_tx = time.monotonic()
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.005)
            if rlist:
                key = sys.stdin.read(1)
                if key in ('q', '\x03'):
                    break
                elif key in THRUSTERS:
                    selected = (key,) + THRUSTERS[key]
                    current = 0.0   # 전환 시 이전 전류가 새 모터에 튀는 것 방지
                elif key == 'a':
                    current = max(current - CURRENT_STEP, -MAX_TEST_CURRENT)
                elif key == 's':
                    current = 0.0
                elif key == 'd':
                    current = min(current + CURRENT_STEP, MAX_TEST_CURRENT)
                else:
                    continue
                if selected:
                    print(f'\rT{selected[0]} {selected[2]} '
                          f'({hex(selected[1])}): {current:+.1f} A   ',
                          end='', flush=True)
                else:
                    print('\r스러스터 미선택 — 1~6 을 먼저 누르세요   ',
                          end='', flush=True)
            if time.monotonic() >= next_tx:
                # teleop 과 동일하게 전 채널 송신 — 선택 외에는 0 A
                for cid, _ in THRUSTERS.values():
                    cur = current if selected and cid == selected[1] else 0.0
                    try:
                        send_current(bus, cid, cur)
                    except can.CanError:
                        tx_errors += 1
                next_tx += TX_PERIOD
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\n종료 — 전 채널 정지 명령 전송')
        for _ in range(10):
            for cid, _ in THRUSTERS.values():
                try:
                    send_current(bus, cid, 0.0)
                except can.CanError:
                    pass
            time.sleep(0.02)
        bus.shutdown()
        if tx_errors:
            print(f'⚠ 송신 실패 {tx_errors} 프레임 — CAN 버스 상태 확인 필요')


if __name__ == '__main__':
    main()
