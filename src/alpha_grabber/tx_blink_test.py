#!/usr/bin/env python3
"""
TX LED 점검용 연속 송신기.
FUS-1T Combo-ISO 의 TX+/TX- (또는 RX) LED 가 깜빡이는지 눈으로 보면서
'어느 ttyUSB 포트가 실제로 팔/FUS 에 연결됐는지' 가린다.

사용:
  python3 tx_blink_test.py                # 기본 ttyUSB0 에 연속 송신
  python3 tx_blink_test.py /dev/ttyUSB1   # 특정 포트로 송신
  python3 tx_blink_test.py all            # 0~3 을 차례로 2초씩 송신(어느 포트인지 찾기)
Ctrl-C 로 종료.
"""
import sys, time, glob
import serial
from bplprotocol import BPLProtocol, PacketID

# 안전: 속도 0 패킷이라 팔이 실제로 움직이진 않음. LED 점멸만 본다.
PKT = BPLProtocol.encode_packet(0x02, PacketID.VELOCITY, BPLProtocol.encode_floats([0.0]))

def spam(port, seconds=None):
    print(f"[{port}] 연속 송신 시작 — FUS 의 TX LED 를 봐라", flush=True)
    sp = serial.Serial(port, 115200, parity=serial.PARITY_NONE,
                       stopbits=serial.STOPBITS_ONE, timeout=0)
    t0 = time.time()
    try:
        while seconds is None or time.time() - t0 < seconds:
            sp.write(PKT)
            sp.flush()
            time.sleep(0.02)   # 50 Hz → LED 가 또렷하게 깜빡임
    finally:
        sp.close()

arg = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
if arg == "all":
    for p in sorted(glob.glob("/dev/ttyUSB*")):
        try:
            spam(p, seconds=2.0)
        except Exception as e:
            print(f"  {p}: {e}")
        print("  -> 이 2초 동안 FUS LED 깜빡였으면 그 포트가 정답\n")
else:
    spam(arg)
