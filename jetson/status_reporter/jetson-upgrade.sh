#!/bin/bash
# ダッシュボードの「Jetson を更新」から jetson-upgrade.service（root）で動く。
# NVIDIA の部品（JetPack: nvidia-・cuda-・libcudnn・libnvinfer・tensorrt）は変えずに、ほかを更新する。
set -euo pipefail

HOLD='nvidia-|cuda-|libcudnn|libnvinfer|tensorrt'
export DEBIAN_FRONTEND=noninteractive
OPTS=(-y -q -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold)
# 進み具合「<0〜100> <prepare|download|install>」。ダウンロードを 0〜30 %、インストールを 30〜100 % とする
PROGRESS=/run/jetson-upgrade.progress
trap 'rm -f "$PROGRESS"' EXIT
echo '0 prepare' >"$PROGRESS"

apt-get update -q
mapfile -t PKGS < <(apt list --upgradable 2>/dev/null | tail -n +2 | cut -d/ -f1 | grep -vE "^($HOLD)" || true)
if [ "${#PKGS[@]}" -eq 0 ]; then
    echo '更新するものはありません'
    exit 0
fi

PLAN=$(apt-get -s install --only-upgrade "${OPTS[@]}" "${PKGS[@]}")
if grep -qE "^Remv |^Inst ($HOLD)" <<<"$PLAN"; then
    echo 'NVIDIA の部品が変わるか、消える部品があるので、更新をやめました:' >&2
    grep -E "^Remv |^Inst ($HOLD)" <<<"$PLAN" >&2
    exit 1
fi

echo "${#PKGS[@]} 個を更新します"
apt-get install --only-upgrade "${OPTS[@]}" -o APT::Status-Fd=3 "${PKGS[@]}" \
    3> >(awk -F: -v f="$PROGRESS" '
        $1 == "dlstatus" { p = $3 * 0.3; s = "download" }
        $1 == "pmstatus" { p = 30 + $3 * 0.7; s = "install" }
        s { printf "%d %s\n", p, s > f; close(f) }')
echo '更新が終わりました'
