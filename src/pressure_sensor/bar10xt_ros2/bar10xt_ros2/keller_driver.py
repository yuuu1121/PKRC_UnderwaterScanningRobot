"""Keller LD/Bar10XT driver using the correct write-cmd -> read (repeated start)
I2C protocol. The pip `kellerLD` library reads calibration with
read_i2c_block_data(), which injects an extra 0x00 command byte and corrupts
the scaling coefficients (pMax read as 0 -> pressure stuck at the offset, e.g. 1.0).
"""
import time
import struct

from smbus2 import SMBus, i2c_msg


class KellerBar10XT:
    ADDR = 0x40
    CMD_MEASURE = 0xAC
    P_MODE_OFFSETS = (1.01325, 1.0, 0.0)  # PR, PA, PAA

    def __init__(self, bus=1):
        self._bus = SMBus(bus)
        self.pMin = None
        self.pMax = None
        self.offset = 1.0
        self._pressure = None
        self._temperature = None

    def _cmd(self, reg, n=3):
        self._bus.i2c_rdwr(i2c_msg.write(self.ADDR, [reg]))
        time.sleep(0.01)
        msg = i2c_msg.read(self.ADDR, n)
        self._bus.i2c_rdwr(msg)
        return list(msg)

    def _word(self, reg):
        d = self._cmd(reg)
        return d[1] << 8 | d[2]

    def init(self):
        mode_id = self._word(0x12) & 0b11
        self.offset = self.P_MODE_OFFSETS[mode_id]
        pmin_raw = self._word(0x13) << 16 | self._word(0x14)
        pmax_raw = self._word(0x15) << 16 | self._word(0x16)
        self.pMin = struct.unpack('f', struct.pack('I', pmin_raw))[0]
        self.pMax = struct.unpack('f', struct.pack('I', pmax_raw))[0]
        return True

    def read(self):
        d = self._cmd(self.CMD_MEASURE, 5)
        p_raw = d[1] << 8 | d[2]
        t_raw = d[3] << 8 | d[4]
        self._pressure = (p_raw - 16384) * (self.pMax - self.pMin) / 32768 + self.pMin + self.offset
        self._temperature = ((t_raw >> 4) - 24) * 0.05 - 50
        return self._pressure, self._temperature

    def pressure(self):
        return self._pressure

    def temperature(self):
        return self._temperature

    def close(self):
        try:
            self._bus.close()
        except Exception:
            pass
