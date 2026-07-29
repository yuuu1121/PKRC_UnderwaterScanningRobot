#!/usr/bin/env python3
"""Check and set pinmux for SOC_GPIO19_PG6 (PWM mode)."""
import mmap, struct, os, sys

PINMUX_BASE = 0x02430000
PAGE_SIZE = 4096

fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
mm = mmap.mmap(fd, PAGE_SIZE, mmap.MAP_SHARED,
               mmap.PROT_READ | mmap.PROT_WRITE,
               offset=PINMUX_BASE)

print("Scanning pinmux registers (non-zero)...")
for offset in range(0, 0x200, 4):
    val = struct.unpack('<I', mm[offset:offset+4])[0]
    if val != 0:
        print(f"  0x{PINMUX_BASE + offset:08X} [0x{offset:03X}]: 0x{val:08X}")

mm.close()
os.close(fd)
