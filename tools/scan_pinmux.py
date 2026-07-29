#!/usr/bin/env python3
"""Scan Tegra234 pinmux registers to find SOC_GPIO19_PG6."""
import mmap, struct, os

PINMUX_BASE = 0x02430000
PINMUX_SIZE = 0x19100

fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
mm = mmap.mmap(fd, PINMUX_SIZE, mmap.MAP_SHARED,
               mmap.PROT_READ | mmap.PROT_WRITE,
               offset=PINMUX_BASE)

# Scan for registers matching current SOC_GPIO19_PG6 config:
# function=rsvd1 (1), tristate=1, input_en=1, gpio-mode=0
# Bits: [1:0]=01, bit4=1, bit6=1

print("Registers matching func=rsvd1, tristate=1, input_en=1:")
found = []
for offset in range(0, PINMUX_SIZE, 4):
    val = struct.unpack('<I', mm[offset:offset+4])[0]
    if val == 0xFFFFFFFF or val == 0:
        continue
    func = val & 0x3
    tristate = (val >> 4) & 1
    input_en = (val >> 6) & 1
    if func == 1 and tristate == 1 and input_en == 1:
        found.append((offset, val))
        print(f"  0x{PINMUX_BASE + offset:08X} [0x{offset:05X}]: 0x{val:08X}")

print(f"\nTotal matches: {len(found)}")

# Also print the full register map of non-trivial values
print("\n--- All non-trivial registers ---")
count = 0
for offset in range(0, PINMUX_SIZE, 4):
    val = struct.unpack('<I', mm[offset:offset+4])[0]
    if val != 0xFFFFFFFF and val != 0:
        count += 1
        if count <= 200:
            func = val & 0x3
            tristate = (val >> 4) & 1
            input_en = (val >> 6) & 1
            park = (val >> 5) & 1
            schmitt = (val >> 7) & 1
            print(f"  0x{offset:05X}: 0x{val:08X}  func={func} tri={tristate} in={input_en} park={park}")

print(f"\nTotal non-trivial registers: {count}")
mm.close()
os.close(fd)
