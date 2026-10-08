#!/bin/bash
# Jetson の SSD（NVMe）の深いねむり（APST）・Jetson のメモリを SSD に貸すこと（HMB）と Wi-Fi の省電力を切る。sudo で実行する。
# SSD の設定は次の起動から効く（再起動がいる）。
set -eu
cd "$(dirname "$0")"

CONF=/boot/extlinux/extlinux.conf
cp -n "$CONF" "$CONF.orig-apst"
for ARG in nvme_core.default_ps_max_latency_us=0 nvme.max_host_mem_size_mb=0; do
    if ! grep -q "$ARG" "$CONF"; then
        sed -i "/^[[:space:]]*APPEND /s/\$/ $ARG/" "$CONF"
    fi
done
grep -n "APPEND" "$CONF"

install -m 644 zz-ai-car-wifi-powersave-off.conf /etc/NetworkManager/conf.d/
systemctl reload NetworkManager
for c in $(nmcli -t -f NAME,TYPE con show --active | awk -F: '$2=="802-11-wireless"{print $1}'); do
    nmcli con up "$c"
done
