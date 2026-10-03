#!/bin/bash
# ダッシュボードの「ラズパイを更新」から ai-car-upgrade.service（root）で動く。
# 更新できる部品（ROS 2 も含む）を更新する。消える部品があるときはやめる。
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
OPTS=(-y -q -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold)

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
apt-get install --only-upgrade "${OPTS[@]}" "${PKGS[@]}"
echo '更新が終わりました'
