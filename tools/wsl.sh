#!/bin/bash
# 在 Windows 笔记本的 WSL（Ubuntu）里执行一条命令：  tools/wsl.sh 'ros2 topic list'
# 前提：两台电脑都登录了 Tailscale；~/.ssh/config 里有 Host win（见 docs/remote-windows.md）。
# 命令先转成 base64 再传过去：中间隔着 ssh → Windows cmd → wsl → bash 四层，引号直接传会被改坏。
set -euo pipefail
b64=$(printf '%s' "$*" | base64 | tr -d '\n')
ssh -o ConnectTimeout=15 win "wsl -d Ubuntu -- bash -lc \"echo $b64 | base64 -d | bash -l\""
