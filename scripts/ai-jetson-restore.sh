#!/bin/bash
# Jetson 復元スクリプト（Pi仕様と同一設計、Jetson用適応）
# root で走る（sudo /usr/local/bin/ai-jetson-restore <バックアップ名>）
set -euo pipefail
NAME="${1:?バックアップ名（jetson-YYYYMMDD_HHMMSS-vX.Y.Z）}"
OUT=/home/jetson/jetson-backups
TAR="$OUT/$NAME.tar.zst"
MAN="$OUT/$NAME.manifest.json"
[ -f "$TAR" ] || { echo "バックアップが見つからない: $TAR"; exit 1; }
[ -f "$MAN" ] || { echo "manifestが見つからない: $MAN"; exit 1; }
WANT=$(python3 -c "import json;print(json.load(open('$MAN'))['tar_sha256'])")
GOT=$(sha256sum "$TAR" | cut -d' ' -f1)
[ "$WANT" = "$GOT" ] || { echo "sha256不一致（バックアップ破損）: want=$WANT got=$GOT"; exit 2; }

echo "復元開始: $NAME"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
tar --zstd -xf "$TAR" -C "$WORK"

# Tier A: /home/jetson へ上書き復元（削除はしない＝既存維持、バックアップ同梱は除外）
cd "$WORK"
for item in $(ls -A); do
  [ "$item" = "tierB" ] && continue
  rsync -a --exclude 'jetson-backups' "$item" /home/jetson/
done
chown -R jetson:jetson /home/jetson 2>/dev/null || true

# Tier B: units / extlinux / netplan（復元前のコピーを残す）
cd "$WORK/tierB"
for f in jetson-*.service jetson-*.timer; do
  [ -f "$f" ] && install -m 644 "$f" /etc/systemd/system/
done
if [ -f extlinux.conf ]; then
  cp -n /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.pre-restore || true
  install -m 644 extlinux.conf /boot/extlinux/extlinux.conf
fi
for f in netplan*.yaml *.yaml; do
  [ -f "$f" ] || continue
  cp -n "/etc/netplan/$f" "/etc/netplan/$f.pre-restore" 2>/dev/null || true
  install -m 600 -o root -g root "$f" "/etc/netplan/$f"
done
systemctl daemon-reload
for s in jetson-*.service; do
  [ -f "$s" ] && systemctl restart "${s%.service}" || true
done
systemctl restart ollama 2>/dev/null || true
echo "復元完了: $NAME"
echo "注意: netplan apply はSSH切断リスクがあるので必要時手動で。sudoersの中身は tierB/sudoers-content.md に記録（visudo -f で人手検証して導入）。"
