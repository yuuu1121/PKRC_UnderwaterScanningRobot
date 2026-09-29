"""Keller LD/Bar10XT driver using the correct write-cmd -> read (repeated start)
I2C protocol. The pip `kellerLD` library reads calibration with
read_i2c_block_data(), which injects an extra 0x00 command byte and corrupts
the scaling coefficients (pMax read as 0 -> pressure stuck at the offset, e.g. 1.0).

The status byte (first byte of every reply) must be checked before trusting the
data bytes. Bit 5 set (0x60) means the A/D conversion is not finished, and the
sensor then replies with all-zero data. Feeding p_raw=0 into the linear scaling
yields -4 bar / -51 m, which is what surfaced as "depth spikes negative".
Measured on bus 1: 0x60 replies were all-zero 100% of the time, 0x40 replies
never were. Longer waits do NOT eliminate it (20 ms was worse than 10 ms) --
it is sporadic phase drift against the sensor's own conversion cycle, so the
fix is a retry, not a longer sleep.

THIS UNIT'S PRESSURE MSB IS STUCK. Measured over a real 3 m dive (508 samples,
bagfiles/pressure1): the pressure MSB read 0x40 on every single sample while the
LSB cycled 0..255, pinning p_raw to 0x4000..0x40FF and depth to -0.136..+0.660 m.
Per the datasheet (Communication_Protocol_4LD-9LD, section 4.2) a healthy unit
spans 16384..49152, i.e. the MSB should move freely. So the scaling code is
correct and the sensor is at fault -- REPLACING IT IS THE REAL FIX.

`simplified: unwrap only` -- until then, _unwrap() reconstructs the missing MSB
by counting LSB rollovers, the way a 3-digit odometer is read past 999. This
recovers RELATIVE depth changes; it cannot recover absolute depth, and it fails
silently if a rollover is missed. At the sensor's 5 Hz that happens above
~2 m/s of vertical speed (256 counts = 0.799 m per sample interval), so the
class tracks `wrap_suspect` for callers to surface. Replaying the 3 m dive bag
through this unwrap reached only 1.95 m, consistent with rollovers having been
missed on the fast descent -- treat unwrapped depth as approximate.
"""
import time
import struct

from smbus2 import SMBus, i2c_msg


class KellerBar10XT:
    ADDR = 0x40
    CMD_MEASURE = 0xAC
    P_MODE_OFFSETS = (1.01325, 1.0, 0.0)  # PR, PA, PAA
    STATUS_BUSY = 0x20   # bit5: conversion in progress -> data bytes are garbage
    READ_RETRIES = 3     # measured busy rate ~3%; 3 tries -> ~1e-5 give-up rate

    # Unwrap constants. On the 3 m dive bag consecutive-sample deltas were
    # <=38 counts when normal and >=218 when wrapping -- a 6x gap, so half the
    # modulus is a safe split with no risk of mistaking noise for a rollover.
    WRAP_MOD = 256          # LSB-only span: the MSB never increments
    WRAP_THRESHOLD = 128    # WRAP_MOD // 2
    WRAP_SUSPECT_FRAC = 0.75  # delta this close to the threshold -> near-miss

    def __init__(self, bus=1, unwrap=True):
        self._bus = SMBus(bus)
        self.pMin = None
        self.pMax = None
        self.offset = 1.0
        self._pressure = None
        self._temperature = None

        self.unwrap = unwrap
        self._prev_raw = None    # previous wrapped reading
        self._wrap_offset = 0    # accumulated counts from rollovers
        self.wrap_count = 0      # rollovers seen since start
        self.wrap_suspect = 0    # deltas large enough that one may have been missed

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
        for _ in range(self.READ_RETRIES):
            d = self._cmd(self.CMD_MEASURE, 5)
            if not d[0] & self.STATUS_BUSY:
                break
        else:
            # Raise rather than return the last good sample: a stuck sensor must
            # stay visible to the callers' staleness checks, not be papered over.
            raise IOError(
                "Bar10XT busy (status=0x%02x) after %d retries" % (d[0], self.READ_RETRIES))

        p_raw = d[1] << 8 | d[2]
        t_raw = d[3] << 8 | d[4]
        if self.unwrap:
            p_raw = self._unwrap(p_raw)
        self._pressure = (p_raw - 16384) * (self.pMax - self.pMin) / 32768 + self.pMin + self.offset
        self._temperature = ((t_raw >> 4) - 24) * 0.05 - 50
        return self._pressure, self._temperature

    def _unwrap(self, p_raw):
        """Undo the LSB rollover, odometer-style, and return the true count.

        The first reading anchors the sequence, so absolute depth is only as
        right as the surface is at startup -- zero the node at the surface.
        """
        if self._prev_raw is None:
            self._prev_raw = p_raw
            return p_raw

        diff = p_raw - self._prev_raw
        if diff > self.WRAP_THRESHOLD:
            self._wrap_offset -= self.WRAP_MOD
            self.wrap_count += 1
        elif diff < -self.WRAP_THRESHOLD:
            self._wrap_offset += self.WRAP_MOD
            self.wrap_count += 1
        elif abs(diff) > self.WRAP_THRESHOLD * self.WRAP_SUSPECT_FRAC:
            # Close to the fold: at this rate of change a rollover could have
            # been skipped between samples, which would bias every later
            # reading by a whole 0.8 m. Surface it instead of hiding it.
            self.wrap_suspect += 1

        self._prev_raw = p_raw
        return p_raw + self._wrap_offset

    def reset_unwrap(self):
        """Re-anchor at the current reading (call at the surface)."""
        self._prev_raw = None
        self._wrap_offset = 0
        self.wrap_count = 0
        self.wrap_suspect = 0

    def pressure(self):
        return self._pressure

    def temperature(self):
        return self._temperature

    def close(self):
        try:
            self._bus.close()
        except Exception:
            pass
