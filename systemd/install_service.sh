#!/bin/bash
# AI-CAR ダッシュボードを systemd サービスとして登録する（sudo で実行）
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)/ai-car-dashboard.service"

# 手動起動中のノードがあればポート衝突を避けるため停止する
pkill -f 'ai_car_web|camera_node|rplidar' || true
sleep 3

sudo install -m 644 "$SRC" /etc/systemd/system/ai-car-dashboard.service
# ダッシュボードの「再起動」ボタン用（systemctl reboot だけをパスワードなしで許す）
SUDOERS="$(dirname "$SRC")/ai-car-reboot.sudoers"
sudo visudo -cf "$SUDOERS"
sudo install -m 440 "$SUDOERS" /etc/sudoers.d/ai-car-reboot
# ダッシュボードの「ラズパイを更新」ボタン用（ai-car-upgrade.service を始めることだけをパスワードなしで許す）
DIR="$(dirname "$SRC")"
sudo install -m 755 "$DIR/ai-car-upgrade.sh" /usr/local/sbin/ai-car-upgrade
sudo install -m 644 "$DIR/ai-car-upgrade.service" /etc/systemd/system/ai-car-upgrade.service
sudo visudo -cf "$DIR/ai-car-upgrade.sudoers"
sudo install -m 440 "$DIR/ai-car-upgrade.sudoers" /etc/sudoers.d/ai-car-upgrade
# lidar_watchdog_node が LiDAR の USB をつなぎ直せるようにする
sudo install -m 644 "$DIR/99-ai-car-lidar-usb.rules" /etc/udev/rules.d/99-ai-car-lidar-usb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=usb
# lidar_watchdog_node が、固まった LiDAR のために USB の電気を切って入れ直せるようにする
sudo apt-get install -y uhubctl
sudo install -m 755 "$DIR/ai-car-usb-power-cycle.sh" /usr/local/sbin/ai-car-usb-power-cycle
sudo visudo -cf "$DIR/ai-car-usb-power.sudoers"
sudo install -m 440 "$DIR/ai-car-usb-power.sudoers" /etc/sudoers.d/ai-car-usb-power
# バックアップ快照（ai-car-backup.sh）と復元（ai-car-restore.sh）をrootに導入する
sudo install -m 755 "$DIR/../scripts/ai-car-backup.sh" /usr/local/sbin/ai-car-backup
sudo install -m 644 "$DIR/ai-car-backup.service" /etc/systemd/system/ai-car-backup.service
sudo install -m 644 "$DIR/ai-car-backup.timer" /etc/systemd/system/ai-car-backup.timer
sudo visudo -cf "$DIR/ai-car-backup.sudoers"
sudo install -m 440 "$DIR/ai-car-backup.sudoers" /etc/sudoers.d/ai-car-backup
sudo install -m 755 "$DIR/../scripts/ai-car-restore.sh" /usr/local/sbin/ai-car-restore
sudo systemctl daemon-reload
sudo systemctl enable --now ai-car-backup.timer
sudo systemctl daemon-reload
sudo systemctl enable --now ai-car-dashboard.service
systemctl status ai-car-dashboard.service --no-pager
