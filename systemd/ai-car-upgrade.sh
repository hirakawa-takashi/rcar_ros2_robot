#!/bin/bash
# ダッシュボードの「ラズパイを更新」から ai-car-upgrade.service（root）で動く。
# 更新できる部品（ROS 2 も含む）を更新する。消える部品があるときはやめる。
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
OPTS=(-y -q -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold)
# 進み具合「<0〜100> <prepare|download|install>」。ダウンロードを 0〜30 %、インストールを 30〜100 % とする
PROGRESS=/run/ai-car-upgrade.progress
trap 'rm -f "$PROGRESS"' EXIT
echo '0 prepare' >"$PROGRESS"

apt-get update -q
mapfile -t PKGS < <(apt list --upgradable 2>/dev/null | tail -n +2 | cut -d/ -f1 || true)
if [ "${#PKGS[@]}" -eq 0 ]; then
    echo '更新するものはありません'
    exit 0
fi

PLAN=$(apt-get -s install --only-upgrade "${OPTS[@]}" "${PKGS[@]}")
if grep -q '^Remv ' <<<"$PLAN"; then
    echo '消える部品があるので、更新をやめました:' >&2
    grep '^Remv ' <<<"$PLAN" >&2
    exit 1
fi

echo "${#PKGS[@]} 個を更新します"
apt-get install --only-upgrade "${OPTS[@]}" -o APT::Status-Fd=3 "${PKGS[@]}" \
    3> >(awk -F: -v f="$PROGRESS" '
        $1 == "dlstatus" { p = $3 * 0.3; s = "download" }
        $1 == "pmstatus" { p = 30 + $3 * 0.7; s = "install" }
        s { printf "%d %s\n", p, s > f; close(f) }')
echo '更新が終わりました'
