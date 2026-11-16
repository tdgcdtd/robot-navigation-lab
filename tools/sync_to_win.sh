#!/bin/bash
# 把 Mac 上还没提交的文件直接拷到 Windows 的 WSL 仓库里（开发时用；正式同步还是走 git）。
#   tools/sync_to_win.sh arm_ws/src/e6_mujoco [更多路径...]
# 先 scp 到 Windows 用户目录，再在 WSL 里解包：ssh 命令行太长会被 Windows 的 cmd 截断，所以不走命令行传内容。
set -euo pipefail
cd "$(dirname "$0")/.."
tmp=$(mktemp -t sync).tgz
COPYFILE_DISABLE=1 tar czf "$tmp" --exclude __pycache__ --exclude '*.pyc' "$@"
scp -q "$tmp" win:sync_to_wsl.tgz
rm -f "$tmp"
tools/wsl.sh 'cd ~/project/robot-navigation-lab && tar xzf /mnt/c/Users/$(cmd.exe /c "echo %USERNAME%" 2>/dev/null | tr -d "\r")/sync_to_wsl.tgz 2>&1 | grep -v "Ignoring unknown extended header" ; rm -f /mnt/c/Users/*/sync_to_wsl.tgz; echo "已同步："; ls -d '"$*" 2>&1 | grep -v "systemd user session"
