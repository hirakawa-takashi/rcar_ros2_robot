#!/bin/bash
# バックアップを Windows PC（WSL）へミラーする。ラズパイで super が手動で動かす（定時は禁止）。
#   bash scripts/ai-backup-mirror.sh pi|jetson [ファイル名.tar.zst]
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
case "$KIND" in
  pi) SD=$HOME/ai-car-backups; D=$W/ai-car-backups; PFX=ai-car-
      src() { bash -c "$1"; } ;;
  jetson) SD=/home/jetson/jetson-backups; D=$W/jetson-backups; PFX=jetson-
      src() { ssh "${O[@]}" jetson@192.168.11.23 "$1"; } ;;
  *) echo "使い方: $0 pi|jetson [ファイル名.tar.zst]" >&2; exit 2 ;;
esac
pc() { ssh "${O[@]}" "$PC" "$1"; }
LOG=$HOME/mirror-$KIND.log
log() { echo "$(date '+%F %T') $*" | tee -a "$LOG"; }

if [ -z "$F" ]; then
  M=$(src "ls -1 $SD/${PFX}*.manifest.json 2>/dev/null | sort | tail -1")
  [ -n "$M" ] || { log "NG バックアップがない: $SD"; exit 1; }
  F=$(basename "${M%.manifest.json}").tar.zst
fi
case "$F" in ${PFX}*.tar.zst) ;; *) log "NG 名前がちがう: $F"; exit 2 ;; esac
case "$F" in */*|*" "*) log "NG 名前がちがう: $F"; exit 2 ;; esac
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
src "cat $SD/$M" | pc "cat > '$D/$M'"
WANT=$(src "cat $SD/$M" | python3 -c 'import json,sys; print(json.load(sys.stdin)["tar_sha256"])')
GOT=$(pc "sha256sum '$D/$F'" | cut -d' ' -f1)
if [ -n "$WANT" ] && [ "$WANT" = "$GOT" ]; then
  log "OK sha256 $GOT $F"
else
  log "NG sha256 want=$WANT got=$GOT $F"; exit 1
fi
