#!/bin/bash
# Jetson バックアップ快照（AI-CAR Pi仕様 v1.30.2 と同一設計、Jetson用適応）
# Tier A: /home/jetsonのサービスコード群・データ・モデルキャッシュ（再生可能な .cache は除外可）
# Tier B: jetson-*ユニット、sudoers内容記録、extlinux.conf、netplan、aptマニフェスト、tailscale、crontab
# sudoersファイル自体は440でjetsonが読めないため、内容記録（sudoers-content.md）を同梱する。
set -euo pipefail
HOME_DIR=/home/jetson
OUT="$HOME_DIR/jetson-backups"
KEEP="${JETSON_BACKUP_KEEP:-7}"
mkdir -p "$OUT"
L4T=$(head -1 /etc/nv_tegra_release 2>/dev/null | tr -d '\n')
TS=$(date '+%Y%m%d_%H%M%S')
NAME="jetson-${TS}-L4T36.4.7"
TAR="$OUT/$NAME.tar.zst"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# Tier B: 読み取れる設定をstagingへ
B="$WORK/tierB"
mkdir -p "$B"
for f in /etc/systemd/system/jetson-*.service /etc/systemd/system/jetson-*.timer; do
  [ -f "$f" ] && cp -p "$f" "$B/"
done
cp -p /boot/extlinux/extlinux.conf "$B/" 2>/dev/null || true
for f in /etc/netplan/*.yaml; do
  [ -f "$f" ] && cp -p "$f" "$B/"
done
apt list --installed 2>/dev/null > "$B/apt-installed.txt" || true
tailscale status --json 2>/dev/null > "$B/tailscale.json" || true
crontab -l > "$B/crontab.txt" 2>/dev/null || true
cat > "$B/sudoers-content.md" <<'EOF'
# sudoersファイル（440）の中身の記録（strata設定・セッション知識）
# /etc/sudoers.d/apt-ops: jetson ALL=(root) NOPASSWD: /usr/bin/apt, /usr/bin/apt-get,
#   /usr/sbin/reboot, /usr/sbin/shutdown, systemctl reboot/poweroff, jetson-upgrade関連
# 新規設定時は visudo -cf 検証 + install -m 440 での導入を必ず行い、内容をこの記録へ反映する。
EOF

cd "$HOME_DIR"
# Tier Aメンバは存在するものだけ
MEMBERS=()
for p in stackchan_bridge face_id object_finder oled_panel owl_detector health_log \
         migrate-jetson-to-ssd objects objects_test fw_clear open-webui-backup-0.6.14 \
         .ollama/models .config .cache huggingface; do
  [ -e "$p" ] && MEMBERS+=("$p")
done
find . -maxdepth 1 -type f -printf '%P\n' | grep -v -E '^\.' > "$WORK/homefiles.txt" || true
tar --zstd -cf "$TAR" \
  --exclude='jetson-backups' --exclude='jetson-containers' --exclude='open-webui' \
  --exclude='.cache/pip' --exclude='.local/lib/python3.10/site-packages/open_webui' \
  -T "$WORK/homefiles.txt" \
  "${MEMBERS[@]}" -C "$WORK" tierB

SHA256=$(sha256sum "$TAR" | cut -d' ' -f1)
SIZE=$(stat -c%s "$TAR")
cat > "$OUT/$NAME.manifest.json" <<EOF
{
  "name": "$NAME",
  "created": "$(date -Is)",
  "l4t": "$L4T",
  "os": "$(grep PRETTY_NAME /etc/os-release | cut -d'"' -f2)",
  "kernel": "$(uname -r)",
  "tar_sha256": "$SHA256",
  "tar_bytes": $SIZE,
  "tierB_items": $(ls "$B" | wc -l)
}
EOF
cd "$OUT"
ls -1t jetson-*.tar.zst | tail -n +$((KEEP+1)) | while read -r old; do
  rm -f "$old" "${old%.tar.zst}.manifest.json"
done
echo "BACKUP OK $NAME"
echo "sha256=$SHA256 bytes=$SIZE"
echo "path=$TAR"
