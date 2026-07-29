#!/usr/bin/env python3
"""Find SOC_GPIO19_PG6 pinmux register by toggling GPIO and detecting changes."""
import mmap, struct, os, subprocess, time

PINMUX_BASE = 0x02430000
PINMUX_SIZE = 0x19100

def read_all_regs(mm):
    regs = {}
    for offset in range(0, PINMUX_SIZE, 4):
        val = struct.unpack('<I', mm[offset:offset+4])[0]
        if val != 0xFFFFFFFF and val != 0:
            regs[offset] = val
    return regs

fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
mm = mmap.mmap(fd, PINMUX_SIZE, mmap.MAP_SHARED,
               mmap.PROT_READ | mmap.PROT_WRITE,
               offset=PINMUX_BASE)

print("Step 1: Reading registers BEFORE GPIO request...")
before = read_all_regs(mm)

print("Step 2: Requesting GPIO line 41 (PG.06) as output...")
# Use gpioset in background to hold the line
proc = subprocess.Popen(['gpioset', '-m', 'time', '-s', '5', 'gpiochip0', '41=1'],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
time.sleep(1)

print("Step 3: Reading registers AFTER GPIO request...")
after = read_all_regs(mm)

# Find differences
print("\nChanged registers:")
all_offsets = set(list(before.keys()) + list(after.keys()))
for offset in sorted(all_offsets):
    bval = before.get(offset, 0)
    aval = after.get(offset, 0)
    if bval != aval:
        func_b = bval & 0x3
        func_a = aval & 0x3
        tri_b = (bval >> 4) & 1
        tri_a = (aval >> 4) & 1
        inp_b = (bval >> 6) & 1
        inp_a = (aval >> 6) & 1
        print(f"  0x{offset:05X}: 0x{bval:08X} -> 0x{aval:08X}")
        print(f"    func: {func_b}->{func_a}, tri: {tri_b}->{tri_a}, inp: {inp_b}->{inp_a}")

proc.terminate()
proc.wait()

mm.close()
os.close(fd)
