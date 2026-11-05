#!/bin/bash
# バックアップを Windows PC（WSL）へミラーする。ラズパイで super が手動で動かす（定時は禁止）。
#   bash scripts/ai-backup-mirror.sh pi|jetson [ファイル名.tar.zst]
#   bash scripts/ai-backup-mirror.sh videos   # AI-CAR_ws/videos・videos_old_* を PC の ai-car-videos へ（新しい・変わったものだけ、消さない）
# ファイル名を省くと、manifest のある一番新しいバックアップ。PC にとちゅうまであれば続きから送り、
# manifest もコピーして、PC で sha256 を manifest の tar_sha256 と照合する。
# 記録は ~/mirror-pi.log・~/mirror-jetson.log。Devin を終えても続けるには setsid nohup で動かす。
# 前提：ラズパイの公開鍵が PC（WSL）と Jetson の authorized_keys にある。
set -u
KIND=${1:-}
F=${2:-}
PC=${AI_CAR_MIRROR_PC:-super@100.86.172.21}
W=${AI_CAR_MIRROR_DIR:-/mnt/c/Users/hirak/Documents/Strata/Workspace}
O=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=15 -o ServerAliveCountMax=8)
if [ "$KIND" = videos ]; then
  LOG=$HOME/mirror-videos.log
  echo "$(date '+%F %T') start videos → $PC:$W/ai-car-videos" | tee -a "$LOG"
  rsync -a --partial -e "ssh ${O[*]}" "$HOME"/AI-CAR_ws/videos "$HOME"/AI-CAR_ws/videos_old_* \
    "$PC:$W/ai-car-videos/" 2>&1 | tee -a "$LOG"
  if [ "${PIPESTATUS[0]}" -eq 0 ]; then
    echo "$(date '+%F %T') OK videos" | tee -a "$LOG"; exit 0
  fi
  echo "$(date '+%F %T') NG videos" | tee -a "$LOG"; exit 1
fi
case "$KIND" in
  pi) SD=$HOME/ai-car-backups; D=$W/ai-car-backups; PFX=ai-car-
      src() { bash -c "$1"; } ;;
  jetson) SD=/home/jetson/jetson-backups; D=$W/jetson-backups; PFX=jetson-
      src() { ssh "${O[@]}" jetson@192.168.11.23 "$1"; } ;;
  *) echo "使い方: $0 pi|jetson [ファイル名.tar.zst] / $0 videos" >&2; exit 2 ;;
esac
pc() { ssh "${O[@]}" "$PC" "$1"; }
LOG=$HOME/mirror-$KIND.log
log() { echo "$(date '+%F %T') $*" | tee -a "$LOG"; }

if [ -z "$F" ]; then
  M=$(src "ls -1 $SD/${PFX}*.manifest.json 2>/dev/null | sort | tail -1")
  [ -n "$M" ] || { log "NG バックアップがない: $SD"; exit 1; }
  F=$(basename "${M%.manifest.json}").tar.zst
fi
[[ "$F" =~ ^${PFX}[A-Za-z0-9._-]+\.tar\.zst$ ]] || { log "NG 名前がちがう: $F"; exit 2; }
M=${F%.tar.zst}.manifest.json

log "start $F → $PC:$D"
TOTAL=$(src "stat -c %s $SD/$F") || { log "NG 元のファイルが読めない"; exit 1; }
pc "mkdir -p '$D'" || { log "NG PC につながらない"; exit 1; }
for try in 1 2 3 4 5 6; do
  S=$(pc "stat -c %s '$D/$F' 2>/dev/null || echo 0") || { sleep 30; continue; }
  log "try $try: PC $S / $TOTAL"
  [ "$S" -ge "$TOTAL" ] && break
  src "tail -c +$((S + 1)) $SD/$F" | pc "cat >> '$D/$F'"
done
S=$(pc "stat -c %s '$D/$F' 2>/dev/null || echo 0")
[ "$S" -eq "$TOTAL" ] || { log "NG 大きさがちがう: PC $S / $TOTAL"; exit 1; }
MJ=$(src "cat $SD/$M") || { log "NG manifest が読めない"; exit 1; }
WANT=$(printf '%s' "$MJ" | python3 -c 'import json,sys; print(json.load(sys.stdin)["tar_sha256"])') \
  || { log "NG manifest がこわれている"; exit 1; }
GOT=$(pc "sha256sum '$D/$F'" | cut -d' ' -f1)
if [ -n "$WANT" ] && [ "$WANT" = "$GOT" ]; then
  printf '%s\n' "$MJ" | pc "cat > '$D/$M.part' && mv '$D/$M.part' '$D/$M'" \
    || { log "NG manifest を PC に置けない"; exit 1; }
  log "OK sha256 $GOT $F"
else
  log "NG sha256 want=$WANT got=$GOT $F"; exit 1
fi
