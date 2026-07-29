#!/usr/bin/env python3
"""
Reach Alpha 2 (RA-2130) 방향키 텔레오프 — 속도(VELOCITY) 방식
==============================================================
  ↑ / ↓ : 회전 (device 0x02)
  → / ← : 집게 열기 / 닫기 (device 0x01)
  + / - : 속도 키우기 / 줄이기
  q     : 종료

누르고 있으면 그 방향으로 계속 이동, **키에서 손 떼면 즉시 정지**.
매 루프마다 입력 버퍼를 통째로 비워서 명령이 쌓이지 않음 (폭주/잔여이동 없음).

실행:  python3 grabber_teleop.py
"""
import sys, os, tty, termios, select, time, logging
logging.disable(logging.CRITICAL)
import serial
from bplprotocol import BPLProtocol, PacketID

# FTDI 어댑터가 여러 개라 ttyUSB 번호는 재부팅/재연결 시 바뀔 수 있음.
# 팔의 RS485 A,B는 FUS-1T(FT232H, Product ID 6014)에 연결됨 → 그 by-id 경로 사용.
# (FT231X DP05I17R는 FUS가 아니라 다른 어댑터라 사용하면 안 됨)
PORT = "/dev/ttyUSB1"
JAW = 0x01   # 집게 (mm/s)
ROT = 0x02   # 회전 (rad/s)

rot_vel = 0.5     # rad/s
jaw_vel = 3.0     # mm/s
IDLE_STOP = 0.15  # 이 시간(초) 동안 키 없으면 정지 = 손 뗀 것으로 간주

sp = serial.Serial(PORT, baudrate=115200, parity=serial.PARITY_NONE,
                   stopbits=serial.STOPBITS_ONE, timeout=0)

def vel(dev, v):
    sp.write(BPLProtocol.encode_packet(dev, PacketID.VELOCITY,
                                       BPLProtocol.encode_floats([float(v)])))

def stop_all():
    vel(JAW, 0.0); vel(ROT, 0.0)

def parse_last_key(buf):
    """버퍼에서 마지막 유효 키 하나만 추출 (백로그 무시). 반환: (dev, vel) 또는 'quit' 또는 None"""
    if b'q' in buf or b'\x03' in buf:
        return 'quit'
    last = None
    i = 0
    while i < len(buf):
        if buf[i:i+3] == b'\x1b[A': last=(ROT, +rot_vel); i+=3
        elif buf[i:i+3] == b'\x1b[B': last=(ROT, -rot_vel); i+=3
        elif buf[i:i+3] == b'\x1b[C': last=(JAW, +jaw_vel); i+=3
        elif buf[i:i+3] == b'\x1b[D': last=(JAW, -jaw_vel); i+=3
        elif buf[i:i+1] in (b'+', b'='): last='faster'; i+=1
        elif buf[i:i+1] in (b'-', b'_'): last='slower'; i+=1
        else: i+=1
    return last

HELP = ("\r\n=== Alpha 2 방향키 제어 (속도 방식) ===\r\n"
        "  ↑/↓ 회전   →/← 집게 열기/닫기\r\n"
        "  +/- 속도   q 종료\r\n"
        "  (누르면 이동, 떼면 즉시 정지)\r\n")

def status(msg=""):
    return ""   # 표시 안 함

fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
active = None      # 현재 움직이는 device
last_key_t = 0.0
try:
    tty.setraw(fd)
    sys.stdout.write(HELP); sys.stdout.write(status()); sys.stdout.flush()
    while True:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            buf = os.read(fd, 4096)          # 버퍼 통째로 비움 → 누적 없음
            k = parse_last_key(buf)
            if k == 'quit':
                break
            elif k == 'faster':
                rot_vel=min(2.0,rot_vel+0.1); jaw_vel=min(10.0,jaw_vel+1.0)
                sys.stdout.write(status("속도 ↑")); sys.stdout.flush()
            elif k == 'slower':
                rot_vel=max(0.1,rot_vel-0.1); jaw_vel=max(0.5,jaw_vel-1.0)
                sys.stdout.write(status("속도 ↓")); sys.stdout.flush()
            elif k:
                dev, v = k
                if active is not None and active != dev:
                    vel(active, 0.0)         # 다른 관절로 바뀌면 이전 관절 정지
                vel(dev, v)
                active = dev
                last_key_t = time.time()
                name = {JAW:"집게", ROT:"회전"}[dev]
                arrow = {(JAW,+jaw_vel):"열기▷",(JAW,-jaw_vel):"닫기◁",
                         (ROT,+rot_vel):"회전+",(ROT,-rot_vel):"회전-"}.get((dev,v),"")
                sys.stdout.write(status(f"{name} {arrow}")); sys.stdout.flush()
        # 키가 한동안 없으면 = 손 뗌 → 정지
        if active is not None and (time.time() - last_key_t) > IDLE_STOP:
            vel(active, 0.0)
            active = None
            sys.stdout.write(status("정지")); sys.stdout.flush()
finally:
    stop_all()                                # 종료 시 반드시 정지
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    sp.close()
    print("\r\n정지 후 종료. 안녕히!")
