#!/bin/bash
# Jetson 復元スクリプト（Pi仕様と同一設計、Jetson用適応）
# root で走る（sudo /usr/local/bin/ai-jetson-restore <バックアップ名>）
set -euo pipefail
NAME="${1:?バックアップ名（jetson-YYYYMMDD_HHMMSS-vX.Y.Z）}"
# シェル記号を混ぜた名前を拒否（コマンド注入防止。ダッシュボード側の正規表現検証に続くroot側2番目の防御）
if [[ ! "$NAME" =~ ^jetson-[0-9]{8}_[0-9]{6}(-v[0-9]+\.[0-9]+\.[0-9]+|-L4T[0-9]+\.[0-9]+\.[0-9]+)$ ]]; then
  echo "バックアップ名が正しくありません（実行拒否）: $NAME"
  exit 1
fi
OUT=/home/jetson/jetson-backups
TAR="$OUT/$NAME.tar.zst"
MAN="$OUT/$NAME.manifest.json"
[ -f "$TAR" ] || { echo "バックアップが見つからない: $TAR"; exit 1; }
[ -f "$MAN" ] || { echo "manifestが見つからない: $MAN"; exit 1; }
WANT=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["tar_sha256"])' "$MAN")
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
echo "復元完了: $NAME"
echo "注意: netplan apply はSSH切断リスクがあるので必要時手動で。sudoersの中身は tierB/sudoers-content.md に記録（visudo -f で人手検証して導入）。"
# サービスは1つずつ再起動せず、Jetson ごと再起動する（電源を入れたときと同じ順番で全部が起動する）
# 1つずつ再起動すると、8 GB の GPU メモリを先に取ったほうが勝ち、声のLLM（qwen2.5:3b）か物さがし（jetson-finder）が
# cudaMalloc out of memory で載らなかった（2026-10-30 実測）。作業フォルダの削除が終わるよう 15 秒あけてから再起動する。
sync
systemd-run --on-active=15 --unit=ai-jetson-restore-reboot /usr/bin/systemctl reboot \
  && echo "15秒後に Jetson を再起動します（約3分で全サービスが戻る）" \
  || echo "再起動の予約に失敗: 手で sudo reboot してください"
