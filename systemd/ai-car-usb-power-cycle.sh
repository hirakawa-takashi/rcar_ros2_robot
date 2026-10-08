#!/bin/bash
# lidar_watchdog_node から sudo で動く。ラズパイ 5 の USB の電気（VBUS）を 3 秒切って入れ直す。
# ラズパイ 5 の USB の 4 つの口は電気がひとまとめなので、xHCI の 4 つのハブの口を全部切らないと切れない。
# LiDAR の CP2102 が固まると USBDEVFS_RESET では戻らないため。
set -euo pipefail

HUBS=()
for d in /sys/bus/usb/devices/usb*; do
    [ "$(basename "$(readlink -f "$d/../driver")")" = xhci-hcd ] || continue
    HUBS+=("$(cat "$d/busnum")")
done
[ "${#HUBS[@]}" -gt 0 ] || { echo 'xHCI の USB ハブがありません' >&2; exit 1; }

for h in "${HUBS[@]}"; do uhubctl -l "$h" -e -N -a 0 >/dev/null; done
sleep 3
for h in "${HUBS[@]}"; do uhubctl -l "$h" -e -N -a 1 >/dev/null; done
echo "USB の電気を 3 秒切って入れ直しました（ハブ ${HUBS[*]}）"
