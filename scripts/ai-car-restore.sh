#!/bin/bash
# AI-CAR 復元スクリプト（v1.0 仕様 R2: 稼働Pi全体復元）
# root で走る（sudo /usr/local/sbin/ai-car-restore <スナップショット名>）
set -euo pipefail
# ダッシュボードから発動したときはダッシュボードのcgroupから抜けて、dashboard停止に殺されない
if [ -w /sys/kernel/cgroup/procs ]; then echo $$ > /sys/kernel/cgroup/procs; fi
NAME="${1:?スナップショット名（ai-car-YYYYMMDD_HHMMSS-vX.Y.Z-<sha>）}"
OUT=/home/super/ai-car-backups
TAR="$OUT/$NAME.tar.zst"
MAN="$OUT/$NAME.manifest.json"
[ -f "$TAR" ] || { echo "スナップショットが見つからない: $TAR"; exit 1; }
[ -f "$MAN" ] || { echo "manifestが見つからない: $MAN"; exit 1; }
WANT=$(python3 -c "import json;print(json.load(open('$MAN'))['tar_sha256'])")
GOT=$(sha256sum "$TAR" | cut -d' ' -f1)
[ "$WANT" = "$GOT" ] || { echo "sha256不一致（スナップショット破損）: want=$WANT got=$GOT"; exit 2; }

echo "復元開始: $NAME (version=$(python3 -c "import json;print(json.load(open('$MAN'))['pi_version'])"))"
systemctl stop ai-car-dashboard.service || true
pkill -f 'ai_car_web|camera_node|rplidar' || true
sleep 2

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
tar --zstd -xf "$TAR" -C "$WORK"

# Tier A: home直下（tierB以外）→ 上書き復元（削除はしない＝既存維持）
cd "$WORK"
for item in $(ls -A); do
  [ "$item" = "tierB" ] && continue
  rsync -a --exclude 'ai-car-backups' "$item" /home/super/
done
chown -R super:super "$WORK" 2>/dev/null || true

# Tier B: units / udev / config.txt
cd "$WORK/tierB"
for f in ai-car-*.service ai-car-*.timer; do
  [ -f "$f" ] && install -m 644 "$f" /etc/systemd/system/
done
for f in 99-ai-car-*.rules; do
  [ -f "$f" ] && install -m 644 "$f" /etc/udev/rules.d/
done
if [ -f config.txt ]; then
  cp -p /boot/firmware/config.txt "/root/config.txt.pre-restore-$(date +%s)"
  install -m 644 config.txt /boot/firmware/config.txt
fi
systemctl daemon-reload
udevadm control --reload-rules && udevadm trigger --subsystem-match=usb || true

systemctl start ai-car-dashboard.service
echo "復元完了: $NAME"
echo "注意: colcon buildは実行していません。コードを復元した場合は"
echo "  cd /home/super/AI-CAR_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
echo "  を実行して、必要ならサービスを再起動してください。"
