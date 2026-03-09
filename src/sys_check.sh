#!/bin/bash

echo "--- System Status ---"
date

echo -e "\n--- CPU Temperature ---"
cat /sys/class/thermal/thermal_zone0/temp | awk '{print $1/1000 " °C"}'

echo -e "\n--- CPU Frequency ---"
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq | awk '{print $1/1000 " MHz"}'

echo -e "\n--- Memory Usage ---"
free -h

echo -e "\n--- Disk Space ---"
df -h /