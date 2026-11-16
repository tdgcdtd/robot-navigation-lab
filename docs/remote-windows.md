# 从 Mac 远程操作 Windows 笔记本（WSL / Docker / ROS2）

为什么需要：模型在 Mac 上跑（M3、M4），机械臂的 ROS2 / MoveIt 在 Windows 的 WSL 里。里程碑 5 要两边互通。

## 结构

```
Mac ──Tailscale──> Windows（OpenSSH 服务器，只认密钥）──> wsl -d Ubuntu ──> bash
```

- **为什么用 Tailscale**：校园网隔离了设备（同一个 Wi‑Fi 下互相连不上，实测两个方向都不通）。Tailscale 给两台电脑各一个固定的 `100.x` 地址，换网络也不变。
- **为什么只认密钥**：Windows 的 `sshd_config` 里关掉了密码登录，没有 Mac 上的私钥登不进去。

## 用法

```bash
tools/wsl.sh 'ros2 topic list'        # 在 WSL 里执行一条命令
ssh win "docker ps"                   # 在 Windows 本身执行
scp win:/path/on/windows ./           # 拷文件
```

## 配置（已做过一次，换电脑时照这个重做）

1. 两台电脑装 Tailscale，登录同一个账号。
2. Windows（管理员 PowerShell）：装 OpenSSH 服务器、把 Mac 公钥写进 `C:\ProgramData\ssh\administrators_authorized_keys`、`sshd_config` 加 `PasswordAuthentication no`。
3. Mac：`~/.ssh/config` 里加 `Host win`（HostName 填 Windows 的 Tailscale 地址，User 填 Windows 用户名，IdentityFile 指向私钥）。地址和用户名不写进仓库。

## 踩过的坑

- 校园网下直连不通，Windows 的 ping 回的是"无法访问目标主机"（它自己找不到对方）→ 是网络隔离，不是防火墙。
- Mac 上开着代理软件时，没登录 Tailscale 也会显示"端口通"（代理接了连接）→ 以能否真正登录为准。
- `wsl -e bash -lc '...'` 经过 Windows 的 cmd 引号会坏 → `tools/wsl.sh` 用 base64 传命令。
- Docker Desktop 要在 Windows 上手动打开，SSH 里启动不了图形程序。
