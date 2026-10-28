#!/bin/bash
# AI-CAR バックアップスナップショット（v1.0 仕様）
# Tier A: homeデータ（AI-CAR_ws[build/install/log除く]、patrols、calib、poselog、設定、home直下のテストファイル）
# Tier B: root不要で読める系設定（systemdユニット、udev、config.txt、aptマニフェスト、tailscale、crontab）
#  sudoersの中身はリポジトリ systemd/*.sudoers が正（ファイル自体は440でsuperが読めない）
# root（ai-car-backup.service）と super（手動実行）の両対応。
set -euo pipefail
HOME_DIR="${SUDO_HOME:-$HOME}"
[ "$HOME_DIR" = "/root" ] && HOME_DIR=/home/super
WS="$HOME_DIR/AI-CAR_ws"
OUT="$HOME_DIR/ai-car-backups"
KEEP="${AI_CAR_BACKUP_KEEP:-7}"
mkdir -p "$OUT"
VERSION=$(grep -m1 -oE '<version>[^<]+' "$WS/src/ai_car_web/package.xml" | sed 's/<version>//')
SHA=$(git -c safe.directory="$WS" -C "$WS" rev-parse --short HEAD)
BRANCH=$(git -c safe.directory="$WS" -C "$WS" rev-parse --abbrev-ref HEAD)
TS=$(date '+%Y%m%d_%H%M%S')
NAME="ai-car-${TS}-v${VERSION}-${SHA}"
TAR="$OUT/$NAME.tar.zst"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# Tier B: 読み取れる設定をstagingへ
B="$WORK/tierB"
mkdir -p "$B"
for f in /etc/systemd/system/ai-car-*.service /etc/systemd/system/ai-car-*.timer; do
  [ -f "$f" ] && cp -p "$f" "$B/"
done
for f in /etc/udev/rules.d/99-ai-car-*.rules; do
  [ -f "$f" ] && cp -p "$f" "$B/"
done
cp -p /boot/firmware/config.txt "$B/" 2>/dev/null || true
cp -p /boot/firmware/cmdline.txt "$B/" 2>/dev/null || true
apt list --installed 2>/dev/null > "$B/apt-installed.txt" || true
tailscale status --json 2>/dev/null > "$B/tailscale.json" || true
crontab -l > "$B/crontab.txt" 2>/dev/null || true

cd "$HOME_DIR"
# home直下の散発ファイル（テストスクリプト・log・画像）をリスト化
find . -maxdepth 1 -type f -printf '%P\n' | grep -v -E '^\.' > "$WORK/homefiles.txt" || true
# Tier Aメンバは存在するものだけ選ぶ（calibはAI-CAR_ws/calib、patrolsはhome直下）
MEMBERS=(AI-CAR_ws)
for p in patrols calib AI-CAR_ws/calib .config/aicar .config/systemd/user; do
  [ -e "$p" ] && MEMBERS+=("$p")
done
# 単一パスで Tier A + homefiles + tierB をまとめる（zstdは追記不可）
tar --zstd -cf "$TAR" \
  --exclude='AI-CAR_ws/build' --exclude='AI-CAR_ws/install' --exclude='AI-CAR_ws/log' \
  --exclude='AI-CAR_ws/ai-car-backups' \
  --exclude='libcamera' --exclude='libpisp' --exclude='hailort' --exclude='hailort-drivers' \
  --exclude='pyhailort_build' --exclude='opt' \
  -T "$WORK/homefiles.txt" \
  "${MEMBERS[@]}" -C "$WORK" tierB

SHA256=$(sha256sum "$TAR" | cut -d' ' -f1)
SIZE=$(stat -c%s "$TAR")
cat > "$OUT/$NAME.manifest.json" <<EOF
{
  "name": "$NAME",
  "created": "$(date -Is)",
  "pi_version": "$VERSION",
  "git_sha": "$SHA",
  "git_branch": "$BRANCH",
  "os": "$(grep PRETTY_NAME /etc/os-release | cut -d'"' -f2)",
  "kernel": "$(uname -r)",
  "tar_sha256": "$SHA256",
  "tar_bytes": $SIZE,
  "tierB_items": $(ls "$B" | wc -l)
}
EOF
cd "$OUT"
ls -1t ai-car-*.tar.zst | tail -n +$((KEEP+1)) | while read -r old; do
  rm -f "$old" "${old%.tar.zst}.manifest.json"
done
echo "BACKUP OK $NAME"
echo "sha256=$SHA256 bytes=$SIZE"
echo "path=$TAR"
