"""M5 的"大脑"：一句话 → 看 → 想 → 定位 → 做 → 再看。全部在 Mac 本地，不联网。

谁负责什么（每一步换一个专家，出错时知道是哪一环）：
  看    M4 的 VLM（Qwen3-VL-4B + LoRA）：画面里有哪些方块、谁压着谁
  定位  M3 的 DINO（ft_m5，已适配新画面）+ SAM + 深度：每个方块中心的 xyz，毫米级
  想    同一个 Qwen3-VL，关掉 LoRA 当规划器（planner.py）：一句话 → 步骤；先在纸面上走一遍，错了让它重想
  做    Windows 容器里的 task_executor + MoveIt 规划轨迹 → MuJoCo（robot_server.py）执行
  再看  每做完一步重新拍照、重新感知：东西真的到了才算数；没到就停下，按新画面重新规划

连接：
  robot_server.py（本机 5600）        拍照、查真值（真值只用于最后打分，决策里一概不用）
  relay_node（容器里，主动连本机 5601） 发指令到 /task_command，收 /task_status、/world_state

用法：
  ../.venv/bin/python brain.py                          # 交互：一句一句输入
  ../.venv/bin/python brain.py --say "把两个方块换个位置"   # 只执行这一句
  加 --scene swap / stack / mixed：开始前把桌面摆成预设的样子（演示用）
"""
import argparse, json, os, queue, socket, sys, threading, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "m3_perception"), os.path.join(HERE, "..", "m4_vlm")]
import yaml                                                # noqa: E402
from planner import Planner, TARGETS                       # noqa: E402
import world                                               # noqa: E402

WORLD_YAML = os.path.join(HERE, "..", "..", "arm_ws", "src", "e6_llm", "config", "world.yaml")
ZONE_XY = {k: tuple(v["xy"]) for k, v in yaml.safe_load(open(WORLD_YAML))["/**"]["ros__parameters"]["targets"].items()}
CUBE = world.CUBE
SCENES = {   # 演示用的桌面摆放（方块中心 x, y[, z]，机械臂底座坐标系）
    "swap": {"red": ZONE_XY["left_zone"], "blue": ZONE_XY["right_zone"]},
    "stack": {"green": (0.30, 0.0), "yellow": (0.30, 0.0, 0.06), "red": (0.34, -0.12)},
    "mixed": {"red": (0.30, 0.06), "blue": (0.30, -0.06), "orange": (0.26, 0.14)},
}


def log(msg):
    print(msg, flush=True)


class Robot:
    """和 robot_server.py 说话（本机）。"""
    def __init__(self, port=5600):
        self.f = socket.create_connection(("127.0.0.1", port)).makefile("rwb")

    def ask(self, **req):
        self.f.write((json.dumps(req) + "\n").encode()); self.f.flush()
        return json.loads(self.f.readline())


class Executor:
    """和容器里的 task_executor 说话（经 relay_node）。"""
    def __init__(self, port=5601):
        self.status, self.known = queue.Queue(), set()
        srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", port)); srv.listen(1)
        log(f"等容器里的 relay_node 连上来（端口 {port}）……")
        self.conn, addr = srv.accept()
        self.wfile, self.lock = self.conn.makefile("wb"), threading.Lock()
        log(f"已连上执行端 {addr[0]}")
        threading.Thread(target=self._read, daemon=True).start()
        time.sleep(1.0)                                     # 等第一条 /world_state

    def _read(self):
        for line in self.conn.makefile("rb"):
            msg = json.loads(line)
            if "world" in msg:
                self.known = {l.split()[0] for l in msg["world"].splitlines() if l.strip()}
            elif "status" in msg:
                self.status.put(msg["status"])

    def send(self, cmd):
        with self.lock:
            self.wfile.write((json.dumps({"cmd": cmd}) + "\n").encode()); self.wfile.flush()

    def run(self, cmd, timeout=240):
        """发一步、等它汇报"完成"或"失败 / 拒绝"。返回 (成功?, 汇报原文)。"""
        while not self.status.empty():
            self.status.get_nowait()
        self.send(cmd)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                s = self.status.get(timeout=1.0)
            except queue.Empty:
                continue
            if s.startswith("[执行] 完成") or s.startswith("[执行] 失败") or s.startswith("[执行] 拒绝"):
                return s.startswith("[执行] 完成"), s
        return False, "等了 4 分钟没有回音"


class Eyes:
    """看 + 定位。"""
    def __init__(self):
        from perceive import Perceiver, locate
        from vlm import VLM
        self.locate = locate
        self.vlm = VLM(adapter=os.path.join(HERE, "..", "m4_vlm", "adapters", "lora"))
        self.dino = Perceiver(dino_path=os.path.join(HERE, "..", "m3_perception", "ckpt", "ft_m5"), edge=480)

    def look(self, robot):
        """返回 {物体id: {"xyz": 中心坐标 或 None, "on": 下面的物体id 或 "table"}}，以及原始画面。"""
        d = np.load(robot.ask(cmd="camera")["path"])
        rgb, depth = d["rgb"], d["depth"]
        cam = {k: d[k] for k in ("f", "cx", "cy", "pos", "R")}
        _, scene = self.vlm.describe(rgb)
        seen = {}
        for o in (scene or {}).get("objects", []):
            oid = f"{o['color']}_cube"
            seen[oid] = {"on": "table" if o.get("on") == "table" else f"{o.get('on')}_cube", "xyz": None}
        covered = {v["on"] for v in seen.values()}
        for oid, info in seen.items():
            if oid in covered:
                continue                                    # 被压住的：顶面看不见，定位会偏，等上面的挪走了再看
            dets = self.dino.detect(rgb, oid.replace("_", " "))
            if dets:
                info["xyz"] = self.locate(self.dino.segment(rgb, dets[0][1]), depth, cam)
        for oid in covered & set(seen):                     # 被压住的：位置按"正下方一层"估计，只给规划器知道它在哪
            top = next((o for o, v in seen.items() if v["on"] == oid and v["xyz"] is not None), None)
            if top:
                seen[oid]["xyz"] = seen[top]["xyz"] - [0, 0, CUBE]
        return {o: v for o, v in seen.items() if v["xyz"] is not None}, rgb


def zone_of(xyz):
    for zid, (x, y) in ZONE_XY.items():
        if np.hypot(xyz[0] - x, xyz[1] - y) < CUBE:
            return zid
    return "table"


def describe(seen):
    return ", ".join(f"{o.removesuffix('_cube')}@({v['xyz'][0]:.3f},{v['xyz'][1]:.3f},{v['xyz'][2]:.3f})"
                     + (f"压在{v['on'].removesuffix('_cube')}上" if v["on"] != "table" else "") for o, v in seen.items())


class Brain:
    def __init__(self):
        self.robot = Robot()
        log("加载模型（VLM + DINO + SAM）……")
        self.eyes = Eyes()
        self.planner = Planner(self.eyes.vlm)
        self.ex = Executor()

    def perceive_and_sync(self):
        """看一眼，把看到的真实位置告诉执行端（observe），看不见的删掉（forget）。"""
        seen, rgb = self.eyes.look(self.robot)
        for oid, v in seen.items():
            self.ex.send(f"observe {oid} {v['xyz'][0]:.4f} {v['xyz'][1]:.4f} {v['xyz'][2]:.4f}")
        for oid in self.ex.known - set(seen):
            self.ex.send(f"forget {oid}")
        return seen

    def verify(self, cmd, before, after):
        """做完一步后：目标物体真的到了吗？"""
        _, obj, tgt = cmd.split()
        if obj not in after:
            return f"做完之后看不到 {obj} 了"
        xyz = after[obj]["xyz"]
        if after[obj]["on"] != "table" or xyz[2] > CUBE:
            return f"{obj} 没有平放在桌上（高度 {xyz[2] * 100:.1f} cm）"
        if tgt == "aside":
            moved = np.linalg.norm(xyz[:2] - before[obj]["xyz"][:2])
            return None if moved > CUBE else f"{obj} 只挪了 {moved * 100:.1f} cm"
        off = np.hypot(xyz[0] - ZONE_XY[tgt][0], xyz[1] - ZONE_XY[tgt][1])
        return None if off < 0.02 else f"{obj} 离 {tgt} 中心 {off * 1000:.0f} mm"

    def do(self, text, max_rounds=2):
        t0 = time.time()
        for round_ in range(1, max_rounds + 1):
            seen = self.perceive_and_sync()
            log(f"【看】{describe(seen)}")
            state = {o: zone_of(v["xyz"]) for o, v in seen.items()}
            on = {o: v["on"] for o, v in seen.items()}
            commands, reply = self.planner.plan(text, state, on, log=log)
            log(f"【想】{reply}")
            if not commands:
                return False
            ok_all = True
            for i, cmd in enumerate(commands, start=1):
                log(f"【做 {i}/{len(commands)}】{cmd}")
                ok, status = self.ex.run(cmd)
                log(f"  {status}")
                after = self.perceive_and_sync() if cmd != "home" else seen
                problem = None if cmd == "home" else (self.verify(cmd, seen, after) if ok else status)
                if problem:
                    log(f"【再看】❌ {problem}")
                    ok_all = False
                    break
                if cmd != "home":
                    log(f"【再看】✅ {cmd.split()[1]} 到位")
                seen = after
            if ok_all:
                log(f"全部完成，用时 {time.time() - t0:.0f} 秒")
                return True
            log(f"按现在看到的画面重新规划（第 {round_ + 1} 轮）")
        return False


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--say", default=None)
    ap.add_argument("--scene", choices=list(SCENES), default=None)
    args = ap.parse_args()
    brain = Brain()
    if args.scene:
        brain.robot.ask(cmd="reset", cubes={c: list(p) for c, p in SCENES[args.scene].items()})
        log(f"桌面已摆成「{args.scene}」")
    if args.say:
        ok = brain.do(args.say)
        truth = brain.robot.ask(cmd="state")["cubes"]
        log("真值（只用于核对）：" + ", ".join(f"{k.removesuffix('_cube')}=({v[0]:.3f},{v[1]:.3f},{v[2]:.3f})" for k, v in truth.items()))
        brain.robot.ask(cmd="render", path=f"brain_{args.scene or 'run'}.png")
        sys.exit(0 if ok else 1)
    while True:
        try:
            text = input("\n你说：").strip()
        except EOFError:
            break
        if text:
            brain.do(text)
