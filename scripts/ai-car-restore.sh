#!/bin/bash
# AI-CAR 復元スクリプト（v1.0 仕様 R2: 稼働Pi全体復元）
# root で走る（sudo /usr/local/sbin/ai-car-restore <バックアップ名>）
set -euo pipefail
# ダッシュボードから発動したときはrootでルートcgroupへ抜け、ダッシュボード停止に殺されない
# （このPiに /sys/kernel/cgroup/procs は存在しない。2026-10-30 ssh実測: procs=no、cgroup v2）
if ! echo $$ > /sys/fs/cgroup/cgroup.procs 2>/dev/null; then
  echo "cgroupから抜けられない（ダッシュボード停止でこのスクリプトが殺され、復元が未完成になる）ため中断" >&2
  exit 3
fi
# sudo経由のSIGINT/SIGTERM/SIGHUP（Ctrl-C・ssh切断）を無視し、途中で殺されて部分復元になるのを防ぐ
trap '' INT TERM HUP
NAME="${1:?バックアップ名（ai-car-YYYYMMDD_HHMMSS-vX.Y.Z-<sha>）}"
# シェル記号を混ぜた名前を拒否（コマンド注入防止。ダッシュボード側の正規表現検証に続くroot側2番目の防御）
if [[ ! "$NAME" =~ ^ai-car-[0-9]{8}_[0-9]{6}-v[0-9]+\.[0-9]+\.[0-9]+-[0-9a-f]{7,40}$ ]]; then
  echo "バックアップ名が正しくありません（実行拒否）: $NAME"
  exit 1
fi
OUT=/home/super/ai-car-backups
TAR="$OUT/$NAME.tar.zst"
MAN="$OUT/$NAME.manifest.json"
[ -f "$TAR" ] || { echo "バックアップが見つからない: $TAR"; exit 1; }
[ -f "$MAN" ] || { echo "manifestが見つからない: $MAN"; exit 1; }
WANT=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["tar_sha256"])' "$MAN")
GOT=$(sha256sum "$TAR" | cut -d' ' -f1)
[ "$WANT" = "$GOT" ] || { echo "sha256不一致（バックアップ破損）: want=$WANT got=$GOT"; exit 2; }

echo "復元開始: $NAME (version=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["pi_version"])' "$MAN"))"
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
