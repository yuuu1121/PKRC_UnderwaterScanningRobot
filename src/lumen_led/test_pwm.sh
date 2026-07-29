#!/bin/bash
# Lumen LED PWM quick test script
# Run after reboot: sudo bash test_pwm.sh
#
# Pin 32 (GPIO07/PG.06) → pwmchip3, channel 0
# Lumen LED: 50Hz PWM, 1100us=off, 1900us=full

CHIP=/sys/class/pwm/pwmchip3
CHAN=$CHIP/pwm0

echo "=== Lumen LED PWM Test ==="

# Export PWM channel
if [ ! -d "$CHAN" ]; then
    echo 0 > $CHIP/export
    sleep 0.5
fi

# 50Hz = 20ms period
echo 20000000 > $CHAN/period

echo "[1/4] LED OFF (1100us)"
echo 1100000 > $CHAN/duty_cycle
echo 1 > $CHAN/enable
sleep 2

echo "[2/4] LED 25% (1300us)"
echo 1300000 > $CHAN/duty_cycle
sleep 2

echo "[3/4] LED 50% (1500us)"
echo 1500000 > $CHAN/duty_cycle
sleep 2

echo "[4/4] LED 100% (1900us)"
echo 1900000 > $CHAN/duty_cycle
sleep 2

echo "[Done] Turning off"
echo 1100000 > $CHAN/duty_cycle
echo 0 > $CHAN/enable

echo "=== Test complete ==="
