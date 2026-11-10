"""实时看训练进度：一行进度条原地刷新，每 20 步打一行 train loss，每 200 步打一行 val loss。

用法：../.venv/bin/python watch_train.py [日志文件=results/log_train_lora.txt]
Ctrl-C 只是关掉显示，不影响训练。
"""
import os, re, sys, time

path = sys.argv[1] if len(sys.argv) > 1 else "results/log_train_lora.txt"
best = None


def handle(piece):
    global best
    m = re.search(r"(\d+)/(\d+) \[([\d:]+)<([\d:?]+)", piece)
    if m and piece.startswith("Training:"):
        done, total, used, left = m.groups()
        bar = "█" * (30 * int(done) // int(total))
        print(f"\r  {done:>4}/{total} [{bar:<30}] 已用 {used}  还剩 {left}   ", end="", flush=True)
        return
    m = re.match(r"optimizer_step=(\d+), train_loss=([\d.]+).*train_token_accuracy=([\d.]+)", piece)
    if m:
        step, loss, acc = m.groups()
        print(f"\r  第 {step:>4} 步  train loss {float(loss):.4f}  token 准确率 {float(acc):.1%}" + " " * 20, flush=True)
        return
    m = re.match(r"val_loss=([\d.]+).*val_token_accuracy=([\d.]+)", piece)
    if m:
        v, acc = float(m.group(1)), float(m.group(2))
        note = "（目前最低）" if best is None or v < best else f"（比最低的 {best:.4f} 高：留意过拟合）"
        best = v if best is None else min(best, v)
        print(f"\r  ── 验证集 val loss {v:.4f}  token 准确率 {acc:.1%} {note}" + " " * 10, flush=True)
        return
    if "Error" in piece or "Traceback" in piece or "Saved final" in piece:
        print("\r  " + piece.strip(), flush=True)


while not os.path.exists(path):
    time.sleep(1)
with open(path, errors="ignore") as f:
    buf = ""
    while True:
        chunk = f.read()
        if not chunk:
            time.sleep(1)
            continue
        buf += chunk
        *pieces, buf = re.split(r"[\r\n]", buf)   # 进度条用 \r 刷新，日志用 \n 换行，都当分隔符
        for p in pieces:
            handle(p)
