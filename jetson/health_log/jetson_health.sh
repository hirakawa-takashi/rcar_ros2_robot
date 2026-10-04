#!/bin/bash
# 1 行で Jetson の様子を出す（jetson-health.timer から 1 分ごと。出力は journal の jetson-health に入る）
read -r load1 load5 _ < /proc/loadavg
mem_avail=$(awk '/^MemAvailable/{print int($2/1024)}' /proc/meminfo)
mem_free=$(awk '/^MemFree/{print int($2/1024)}' /proc/meminfo)
swap_used=$(awk '/^SwapTotal/{t=$2} /^SwapFree/{f=$2} END{print int((t-f)/1024)}' /proc/meminfo)
tj=0
for z in /sys/class/thermal/thermal_zone*/temp; do
  t=$(cat "$z" 2>/dev/null) && [ "${t:-0}" -gt "$tj" ] && tj=$t
done
pwm=$(cat /sys/class/hwmon/hwmon0/pwm1 2>/dev/null)
rpm=$(cat /sys/class/hwmon/hwmon2/rpm 2>/dev/null)
down=""
for s in ssh tailscaled jetson-status ollama whisper-server jetson-face jetson-oled; do
  systemctl is-active --quiet "$s" || down="$down $s"
done
top=$(ps -eo rss=,comm= --sort=-rss | head -3 | awk '{printf "%s:%dM ", $2, $1/1024}')
echo "load ${load1}/${load5} mem_avail ${mem_avail}M free ${mem_free}M swap ${swap_used}M" \
     "tj $((tj / 1000))C fan ${pwm:-?}/255 ${rpm:-?}rpm down:${down:- none} top: ${top}"
