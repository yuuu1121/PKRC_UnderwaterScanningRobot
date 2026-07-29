import smbus

bus = smbus.SMBus(1)

for addr in range(0x03, 0x80):
    try:
        bus.read_byte(addr)
        print("found:", hex(addr))
    except:
        pass
