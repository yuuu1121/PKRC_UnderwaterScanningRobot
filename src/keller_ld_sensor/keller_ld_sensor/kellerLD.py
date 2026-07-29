"""Keller LD I2C pressure/temperature transmitter driver.

Python 3 port of https://github.com/bluerobotics/KellerLD-python (MIT License).
Uses smbus2 for I2C access.
"""
import os
import struct
import time

from smbus2 import SMBus, i2c_msg


class KellerLD(object):

    _SLAVE_ADDRESS = 0x40
    _REQUEST_MEASUREMENT = 0xAC
    _DEBUG = False
    _P_MODES = (
        "PR Mode, Vented Gauge",    # Zero when front pressure == rear pressure
        "PA Mode, Sealed Gauge",    # Zero at 1.0 bar
        "PAA Mode, Absolute Gauge"  # Zero at vacuum
    )
    _P_MODE_OFFSETS = (1.01325, 1.0, 0.0)

    def __init__(self, bus=1):
        self._bus = None
        self._pressure = None
        self._temperature = None
        self.pMin = None
        self.pMax = None
        self.pMode = None
        self.pModeOffset = 0.0
        self.year = 0
        self.month = 0
        self.day = 0

        try:
            self._bus = SMBus(bus)
        except Exception:
            print("Bus %d is not available." % bus)
            print("Available busses are listed as /dev/i2c*")
            if os.uname()[1] == 'raspberrypi':
                print("Enable the i2c interface using raspi-config!")

    def _read_reg(self, reg):
        """Keller memory read per datasheet: pure write(reg) with STOP,
        short delay, then pure read of 3 bytes (STATUS + MSB + LSB).

        SMBus combined write-read (repeated start) is not handled correctly
        by the LD series — the register pointer ends up one step ahead, so
        we must terminate the register-select with an explicit STOP before
        issuing a pure read.
        """
        self._bus.i2c_rdwr(i2c_msg.write(self._SLAVE_ADDRESS, [reg]))
        time.sleep(0.002)
        r = i2c_msg.read(self._SLAVE_ADDRESS, 3)
        self._bus.i2c_rdwr(r)
        return list(r)

    def init(self):
        if self._bus is None:
            print("No bus!")
            return False

        # 0x12: pressure mode + calibration date
        # status byte layout: 0b01BMoEXX (B=busy, Mo=mode, E=checksum, X=don't care)
        data = self._read_reg(0x12)
        scaling0 = data[1] << 8 | data[2]
        self.debug(("0x12:", scaling0, data))

        pModeID = scaling0 & 0b11
        self.pMode = self._P_MODES[pModeID]
        self.set_reference_pressure(self._P_MODE_OFFSETS[pModeID])

        self.year = scaling0 >> 11
        self.month = (scaling0 & 0b0000011110000000) >> 7
        self.day = (scaling0 & 0b0000000001111100) >> 2
        self.debug(("calibration date", self.year, self.month, self.day))

        # 0x13/0x14: pMin (MSW/LSW of IEEE-754 float, big-endian)
        data = self._read_reg(0x13)
        MSWord = data[1] << 8 | data[2]
        self.debug(("0x13:", MSWord, data))

        data = self._read_reg(0x14)
        LSWord = data[1] << 8 | data[2]
        self.debug(("0x14:", LSWord, data))

        pMinRaw = (MSWord << 16) | LSWord

        # 0x15/0x16: pMax
        data = self._read_reg(0x15)
        MSWord = data[1] << 8 | data[2]
        self.debug(("0x15:", MSWord, data))

        data = self._read_reg(0x16)
        LSWord = data[1] << 8 | data[2]
        self.debug(("0x16:", LSWord, data))

        pMaxRaw = (MSWord << 16) | LSWord

        # Keller stores calibration floats big-endian per datasheet.
        self.pMin = struct.unpack('>f', struct.pack('>I', pMinRaw))[0]
        self.pMax = struct.unpack('>f', struct.pack('>I', pMaxRaw))[0]

        print("[KellerLD] mode={} offset={:.5f} bar  pMin={:.4f}  pMax={:.4f}  "
              "(raw: 0x{:08X}, 0x{:08X})".format(
                  self.pMode, self.pModeOffset, self.pMin, self.pMax,
                  pMinRaw, pMaxRaw))

        return True

    def set_reference_pressure(self, pBar):
        """Set the reference pressure (bar) subtracted from raw readings."""
        self.pModeOffset = pBar
        self.debug(("pMode", self.pMode, "pressure offset [bar]", self.pModeOffset))

    def read(self):
        if self._bus is None:
            print("No bus!")
            return False

        if self.pMin is None or self.pMax is None:
            print("Init required! Call init() before read().")
            return False

        # Request measurement (pure write, STOP).
        self._bus.i2c_rdwr(
            i2c_msg.write(self._SLAVE_ADDRESS, [self._REQUEST_MEASUREMENT]))
        time.sleep(0.01)  # >=8 ms per datasheet

        # Pure read of 5 bytes: STATUS, P_MSB, P_LSB, T_MSB, T_LSB.
        r = i2c_msg.read(self._SLAVE_ADDRESS, 5)
        self._bus.i2c_rdwr(r)
        data = list(r)

        statusByte = data[0]
        pressureRaw = data[1] << 8 | data[2]
        temperatureRaw = data[3] << 8 | data[4]

        if statusByte & (0b11 << 3):
            print("Invalid mode: %d, expected 0!" % ((statusByte & (0b11 << 3)) >> 3))
            return False

        if statusByte & (1 << 2):
            print("Memory checksum error!")
            return False

        self._pressure = (pressureRaw - 16384) * (self.pMax - self.pMin) / 32768 \
            + self.pMin + self.pModeOffset
        self._temperature = ((temperatureRaw >> 4) - 24) * 0.05 - 50

        self.debug(("data:", data))
        self.debug(("pressureRaw:", pressureRaw, "pressure:", self._pressure))
        self.debug(("temperatureRaw", temperatureRaw, "temperature:", self._temperature))

        return True

    def temperature(self):
        if self._temperature is None:
            print("Call read() first to get a measurement")
            return None
        return self._temperature

    def pressure(self):
        if self._pressure is None:
            print("Call read() first to get a measurement")
            return None
        return self._pressure

    def debug(self, msg):
        if self._DEBUG:
            print(msg)

    def __str__(self):
        return ("Keller LD I2C Pressure/Temperature Transmitter\n"
                "\ttype: {}\n".format(self.pMode) +
                "\tcalibration date: {}-{}-{}\n".format(self.year, self.month, self.day) +
                "\tpressure offset: {:.5f} bar\n".format(self.pModeOffset) +
                "\tminimum pressure: {:.5f} bar\n".format(self.pMin) +
                "\tmaximum pressure: {:.5f} bar".format(self.pMax))


if __name__ == '__main__':
    sensor = KellerLD()
    if not sensor.init():
        print("Failed to initialize Keller LD sensor!")
        raise SystemExit(1)
    print(sensor)

    while True:
        try:
            if sensor.read():
                print("pressure: %7.4f bar\ttemperature: %0.2f C"
                      % (sensor.pressure(), sensor.temperature()))
            time.sleep(0.2)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(e)
