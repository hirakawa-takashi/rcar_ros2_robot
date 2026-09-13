#!/bin/bash
# AI-CAR ダッシュボードを systemd サービスとして登録する（sudo で実行）
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)/ai-car-dashboard.service"

# 手動起動中のノードがあればポート衝突を避けるため停止する
pkill -f 'ai_car_web|camera_node|rplidar' || true
sleep 3

sudo install -m 644 "$SRC" /etc/systemd/system/ai-car-dashboard.service
sudo systemctl daemon-reload
sudo systemctl enable --now ai-car-dashboard.service
systemctl status ai-car-dashboard.service --no-pager
