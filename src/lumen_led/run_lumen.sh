#!/bin/bash
# Run lumen_led node
# Step 1: pinmux setup (requires root)
# Step 2: run node as current user

source /opt/ros/humble/setup.bash
source /home/hero/hero_ws/install/setup.bash

# Setup pinmux for PWM7 on SOC_GPIO19_PG6 (needs sudo)
sudo python3 /home/hero/hero_ws/setup_pwm7_pinmux.py

# Export PWM and fix permissions so non-root can use it
sudo bash -c '
  echo 0 > /sys/class/pwm/pwmchip3/export 2>/dev/null || true
  sleep 0.3
  chmod 666 /sys/class/pwm/pwmchip3/pwm0/period
  chmod 666 /sys/class/pwm/pwmchip3/pwm0/duty_cycle
  chmod 666 /sys/class/pwm/pwmchip3/pwm0/enable
'

ros2 run lumen_led lumen_node "$@"
