#!/usr/bin/env python3
"""Configure SOC_GPIO19_PG6 (Pin 32) pinmux for PWM7 output.

Register: 0x02434080 (PADCTL base 0x02430000 + offset 0x4080)
Target: function=GP(0), tristate=0, input_en=0 → PWM7 output
"""
import mmap, struct, os, sys

PINMUX_BASE = 0x02430000
PINMUX_SIZE = 0x19100
PG6_OFFSET = 0x4080  # SOC_GPIO19_PG6

def read_reg(mm, offset):
    return struct.unpack('<I', mm[offset:offset+4])[0]

def write_reg(mm, offset, val):
    mm[offset:offset+4] = struct.pack('<I', val)

def print_reg(label, val):
    print(f"{label}: 0x{val:08X}")
    print(f"  func[1:0]   = {val & 0x3}  (0=GP/PWM, 1=RSVD1)")
    print(f"  tristate[4] = {(val >> 4) & 1}  (0=driven, 1=hi-Z)")
    print(f"  input_en[6] = {(val >> 6) & 1}  (0=output, 1=input)")
    print(f"  sfio[10]    = {(val >> 10) & 1}  (0=GPIO, 1=SFIO)")

fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
mm = mmap.mmap(fd, PINMUX_SIZE, mmap.MAP_SHARED,
               mmap.PROT_READ | mmap.PROT_WRITE,
               offset=PINMUX_BASE)

old_val = read_reg(mm, PG6_OFFSET)
print_reg("BEFORE", old_val)

# Set for PWM7 output:
# - func = 0 (GP, which includes PWM7)
# - tristate = 0 (drive the pin)
# - input_en = 0 (output mode)
# - sfio = 1 (SFIO mode: route PWM7 to pin)
new_val = old_val
new_val &= ~0x3      # Clear func bits [1:0] → 0 = GP
new_val &= ~(1 << 4) # Clear tristate → driven
new_val &= ~(1 << 6) # Clear input_en → output
new_val |= (1 << 10) # Set sfio → SFIO mode (PWM routed to pin)

write_reg(mm, PG6_OFFSET, new_val)

verify = read_reg(mm, PG6_OFFSET)
print()
print_reg("AFTER", verify)

if verify == new_val:
    print("\nPinmux configured successfully for PWM7!")
else:
    print(f"\nWARNING: Write may have failed. Expected 0x{new_val:08X}, got 0x{verify:08X}")

mm.close()
os.close(fd)
