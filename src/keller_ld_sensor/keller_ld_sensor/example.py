#!/usr/bin/env python3
"""Standalone CLI test loop — mirrors example.py from KellerLD-python."""
import time

from keller_ld_sensor.kellerLD import KellerLD


def main():
    sensor = KellerLD()
    if not sensor.init():
        print("Failed to initialize Keller LD sensor!")
        raise SystemExit(1)

    print(sensor)
    print("Testing Keller LD series pressure sensor")
    print("Press Ctrl + C to quit")
    time.sleep(1.0)

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


if __name__ == '__main__':
    main()
