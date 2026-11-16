"""MuJoCo "机器人"服务：管着仿真世界，听网络上发来的请求。它扮演真机械臂的控制柜。

为什么是"整条轨迹一次发过来"：Mac 和 Windows 之间的网络往返约 230 ms 还丢包（校园网 + Tailscale），
做不了每秒几十次的实时控制。所以 MoveIt 那边把整条轨迹交过来，这里自己按仿真时间一步步执行，
执行完只回报一次结果——网络快慢不影响执行精度。真机械臂的控制柜也是这样工作的。

协议：一行一个 JSON（请求和回答都是）。
    {"cmd": "state"}                              → {"ok": true, "q": [6 个关节角]}
    {"cmd": "traj", "t": [...], "q": [[6]...]}    → 按时间 t（秒）走过这些路点，回报最终关节角和跟踪误差
    {"cmd": "render", "path": "x.png"}            → 存一张相机图片（调试用）

用法：../.venv/bin/python robot_server.py [--host 监听地址] [--port 5600]
默认只监听 Tailscale 的地址（100.x）和本机回环，不对校园网开放。
"""
import argparse, json, os, socket, threading, time
import mujoco
import numpy as np
import world

SETTLE = 0.5      # 轨迹走完后再等这么久（仿真时间），让机械臂停稳


def tailscale_ip():
    """本机的 Tailscale 地址。往 Tailscale 的固定地址"假装"发一个 UDP 包，看系统选了哪个本机地址（不会真的发出数据）。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("100.100.100.100", 9))
        ip = s.getsockname()[0]
        return ip if ip.startswith("100.") else "127.0.0.1"
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


class Robot:
    def __init__(self, cubes):
        self.model, self.data = world.build(cubes)
        self.adr = world.joint_addr(self.model)
        self.lock = threading.Lock()      # 仿真世界只有一个：同一时刻只处理一个请求

    def q(self):
        return self.data.qpos[self.adr].copy()

    def run_trajectory(self, t, q):
        """按时间走过路点。每个仿真步（2 ms）：算出此刻该在的角度 → 交给伺服 → 物理走一步。"""
        t, q = np.asarray(t, float), np.asarray(q, float)
        dt, track = self.model.opt.timestep, 0.0
        start_gap = float(np.abs(q[0] - self.q()).max())      # 轨迹起点和机械臂现在的位置差多少
        for k in range(int(round((t[-1] + SETTLE) / dt))):
            now = min(k * dt, t[-1])
            target = np.array([np.interp(now, t, q[:, j]) for j in range(6)])   # 路点之间线性插值
            self.data.ctrl[:6] = target
            mujoco.mj_step(self.model, self.data)
            if k * dt <= t[-1]:
                track = max(track, float(np.abs(self.q() - target).max()))
        return dict(ok=True, q=self.q().tolist(), start_gap=start_gap, track_err=track,
                    final_err=float(np.abs(self.q() - q[-1]).max()), duration=float(t[-1]))

    def handle(self, req):
        with self.lock:
            return self._handle(req)

    def _handle(self, req):
        cmd = req.get("cmd")
        if cmd == "state":
            return dict(ok=True, q=self.q().tolist(), link6=self.data.body("Link6").xpos.tolist(),
                        cubes={self.model.body(b).name: self.data.xpos[b].tolist()
                               for b in range(self.model.nbody) if self.model.body(b).name.endswith("_cube")})
        if cmd == "traj":
            return self.run_trajectory(req["t"], req["q"])
        if cmd == "render":
            import imageio.v2 as imageio
            path = os.path.join(world.HERE, os.path.basename(req.get("path", "render.png")))
            imageio.imwrite(path, world.render(self.model, self.data))
            return dict(ok=True, path=path)
        return dict(ok=False, error=f"不认识的命令：{cmd}")


def client_loop(robot, conn, addr):
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"已连接：{addr[0]}", flush=True)
    f = conn.makefile("rwb")
    try:
        for line in f:
            req = json.loads(line)
            t0 = time.time()
            try:
                rep = robot.handle(req)
            except Exception as e:                      # 一条坏请求不能把"控制柜"弄死
                rep = dict(ok=False, error=repr(e))
            f.write((json.dumps(rep) + "\n").encode()); f.flush()
            if req.get("cmd") == "traj" and rep.get("ok"):
                print(f"轨迹 {len(req['t'])} 个路点，时长 {rep['duration']:.2f} s（算了 {time.time() - t0:.2f} s）："
                      f"跟踪误差最大 {np.rad2deg(rep['track_err']):.2f}°，停稳后误差 {np.rad2deg(rep['final_err']):.3f}°",
                      flush=True)
    except (ConnectionError, json.JSONDecodeError) as e:
        print(f"连接出错：{e!r}", flush=True)
    finally:
        conn.close()
        print(f"连接断开：{addr[0]}", flush=True)


def serve(robot, hosts, port):
    """每个连接一个线程（桥接节点一个、Mac 上的感知程序一个……），共用同一个仿真世界。"""
    def listen(host):
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port)); srv.listen(4)
        print(f"MuJoCo 机器人服务已启动，监听 {host}:{port}", flush=True)
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=client_loop, args=(robot, conn, addr), daemon=True).start()
    for h in hosts[1:]:
        threading.Thread(target=listen, args=(h,), daemon=True).start()
    listen(hosts[0])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=5600)
    args = ap.parse_args()
    robot = Robot({"red": (0.30, 0.06), "blue": (0.30, -0.06)})      # 和 e6_llm/config/world.yaml 一样的摆放
    # 两个地址：Tailscale（给 Windows 上的桥接节点）+ 本机回环（给 Mac 上自己的程序）
    serve(robot, [args.host] if args.host else sorted({tailscale_ip(), "127.0.0.1"}), args.port)
