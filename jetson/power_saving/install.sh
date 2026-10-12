#!/bin/bash
# Jetson の Wi-Fi の省電力を切る。sudo で実行する。
# SSD の APST・HMB を切る設定は、効かなかったので入れない（README）。前に入れた分は消す（次の起動から効く）。
set -eu
cd "$(dirname "$0")"

CONF=/boot/extlinux/extlinux.conf
for ARG in nvme_core.default_ps_max_latency_us=0 nvme.max_host_mem_size_mb=0; do
    sed -i "/^[[:space:]]*APPEND /s/ *$ARG//" "$CONF"
done
grep -n "APPEND" "$CONF"

install -m 644 zz-ai-car-wifi-powersave-off.conf /etc/NetworkManager/conf.d/
systemctl reload NetworkManager
for c in $(nmcli -t -f NAME,TYPE con show --active | awk -F: '$2=="802-11-wireless"{print $1}'); do
    nmcli con up "$c"
done
