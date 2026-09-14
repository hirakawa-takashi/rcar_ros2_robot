#!/bin/bash
# Raspberry Pi 5 純正ファンの作動温度（thermal_zone0 の active トリップ）を下げる。
# 既定は 50/60/67.5/75℃ で、CPU が 55℃ 前後でも最低速のままになるため前倒しする。
set -eu

ZONE=/sys/class/thermal/thermal_zone0
TEMPS=(${FAN_TRIP_TEMPS:-45000 50000 55000 62000})
HYST=${FAN_TRIP_HYST:-4000}

for i in "${!TEMPS[@]}"; do
  trip="$ZONE/trip_point_$((i + 1))"
  [ -w "${trip}_temp" ] || continue
  [ "$(cat "${trip}_type")" = active ] || continue
  echo "${TEMPS[$i]}" > "${trip}_temp"
  # ヒステリシスはカーネルによっては書き込めないため失敗しても続行する
  echo "$HYST" > "${trip}_hyst" 2>/dev/null || true
done
