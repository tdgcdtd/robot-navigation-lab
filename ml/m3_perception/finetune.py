"""全量微调 Grounding DINO base（2.32 亿参数）：让它分清"颜色 + 方块"。

和里程碑 2 的训练循环是同一个骨架：前向 → loss → 反向 → 更新。多出来的东西：
1. 梯度累积：内存只够 batch 1，就连算 ACCUM 张图的梯度再更新一次（效果≈batch ACCUM）
2. 学习率预热 + 余弦衰减：开头小步试探，中间正常走，结尾越走越小（里程碑 2 提到的"学习率衰减"）
3. 定期在验证集上看 loss、存档；Air 没风扇会降频，中断后 --resume 接着训
4. 只用解码器三项 loss：transformers 里编码器那项分类 loss 数值异常（约 4 万），先排除

用法：
    ../.venv/bin/python finetune.py              # 从头训
    ../.venv/bin/python finetune.py --resume     # 从 ckpt/<out>_latest 接着训
    ../.venv/bin/python finetune.py --out ft_neg  # 带负样本的数据，存到 ckpt/ft_neg
    ../.venv/bin/python finetune.py --data ../m5_robot/dino_data --init ckpt/ft_neg --out ft_m5   # M5：适配有机械臂的新画面
    ../.venv/bin/tensorboard --logdir runs       # 看 train / val 曲线
"""
import argparse, json, os, random, shutil, time
import torch
from PIL import Image
from torch.utils.tensorboard import SummaryWriter
from transformers import (AutoModelForZeroShotObjectDetection, AutoProcessor,
                          get_cosine_schedule_with_warmup)

parser = argparse.ArgumentParser()
parser.add_argument("--epochs", type=int, default=1)
parser.add_argument("--accum", type=int, default=4, help="梯度累积：几张图更新一次")
parser.add_argument("--lr", type=float, default=1e-5)
parser.add_argument("--edge", type=int, default=480, help="输入短边像素数（评估时要一致）")
parser.add_argument("--eval-every", type=int, default=125, help="每多少次更新看一次验证集并存档")
parser.add_argument("--resume", action="store_true")
parser.add_argument("--data", default="data", help="数据文件夹（里面有 train/、val/ 和各自的 labels.json）")
parser.add_argument("--init", default=None, help="从哪个模型开始训（默认官方原版；M5 从 ckpt/ft_neg 接着适配新画面）")
parser.add_argument("--out", default="final", help="模型存到 ckpt/<out>，存档 ckpt/<out>_latest，曲线 runs/<out>")
parser.add_argument("--max-updates", type=int, default=0, help="冒烟测试用：只跑这么多次更新就停（0 = 不限制）")
parser.add_argument("--val-n", type=int, default=0, help="冒烟测试用：验证集只用前几个（0 = 全部）")
args = parser.parse_args()

ID = "IDEA-Research/grounding-dino-base"
DEV = "mps"
random.seed(0); torch.manual_seed(0)

proc = AutoProcessor.from_pretrained(ID)
proc.image_processor.size = {"shortest_edge": args.edge, "longest_edge": 1333}
train = json.load(open(f"{args.data}/train/labels.json"))
val = json.load(open(f"{args.data}/val/labels.json"))
if args.val_n:
    val = val[:args.val_n]


def batch_of(split, item):
    """一个样本 → 模型输入 + 标签。标签里 class_labels=0：句子里只有一个短语（"xxx cube"），目标就是它。
    负样本（box 为 None）给 0 个目标：框的两项 loss 为 0，只剩分类 loss，把所有输出都往"不是它"压。"""
    image = Image.open(f"{args.data}/{split}/{item['image']}").convert("RGB")
    inputs = proc(images=[image], text=[item["text"]], return_tensors="pt").to(DEV)
    if item["box"] is None:
        labels = [{"class_labels": torch.zeros(0, dtype=torch.long, device=DEV),
                   "boxes": torch.zeros(0, 4, device=DEV)}]
    else:
        labels = [{"class_labels": torch.tensor([0], device=DEV),
                   "boxes": torch.tensor([item["box"]], device=DEV)}]
    return inputs, labels


def decoder_loss(out):
    d = out.loss_dict
    return d["loss_ce"] + 5 * d["loss_bbox"] + 2 * d["loss_giou"]   # 和官方系数一致：分类 1、框 L1 5、框 GIoU 2


start = 0
src = f"ckpt/{args.out}_latest" if args.resume else (args.init or ID)
model = AutoModelForZeroShotObjectDetection.from_pretrained(src).to(DEV)
opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
total_updates = len(train) * args.epochs // args.accum
sched = get_cosine_schedule_with_warmup(opt, num_warmup_steps=total_updates // 20, num_training_steps=total_updates)
if args.resume:
    state = torch.load(f"ckpt/{args.out}_latest/train_state.pt", map_location=DEV)
    opt.load_state_dict(state["opt"]); sched.load_state_dict(state["sched"]); start = state["update"]
    print(f"从第 {start} 次更新接着训")


def train_mode():
    model.train()
    model.model.text_backbone.eval()   # MPS 的注意力不支持 dropout：BERT 只关 dropout，权重照样训练


@torch.no_grad()
def val_loss():
    model.eval()
    total = sum(decoder_loss(model(**i, labels=l)).item() for i, l in (batch_of("val", it) for it in val))
    train_mode(); torch.mps.empty_cache()
    return total / len(val)


def save(update):
    tmp = f"ckpt/{args.out}_latest_tmp"                       # 先存到临时目录再改名：存一半断电也不会坏掉旧存档
    model.save_pretrained(tmp)
    torch.save({"opt": opt.state_dict(), "sched": sched.state_dict(), "update": update}, f"{tmp}/train_state.pt")
    shutil.rmtree(f"ckpt/{args.out}_latest", ignore_errors=True); os.replace(tmp, f"ckpt/{args.out}_latest")


writer = SummaryWriter(f"runs/{args.out}")
order = [i for _ in range(args.epochs) for i in random.Random(0).sample(range(len(train)), len(train))]
if start == 0:
    v = val_loss(); writer.add_scalar("loss/val", v, 0)
    print(f"训练前 验证集 loss {v:.3f}")
train_mode()
t0, run_loss, n_run = time.time(), 0.0, 0
last = min(total_updates, start + args.max_updates) if args.max_updates else total_updates
for update in range(start, last):
    opt.zero_grad()
    for k in range(args.accum):                       # 梯度累积：连算 accum 张图
        inputs, labels = batch_of("train", train[order[update * args.accum + k]])
        loss = decoder_loss(model(**inputs, labels=labels)) / args.accum   # 除以 accum：相当于取平均
        loss.backward()                               # 梯度会累加（这正是不能忘 zero_grad 的原因，这里反过来利用它）
        run_loss += loss.item()
    torch.nn.utils.clip_grad_norm_(model.parameters(), 0.1)   # 梯度裁剪：防止某一步梯度过大把模型改坏（DINO 官方用 0.1）
    opt.step(); sched.step()
    n_run += 1

    done = update + 1
    if done % 20 == 0 or done == last:
        sec = (time.time() - t0) / (done - start)
        writer.add_scalar("loss/train", run_loss / n_run, done); writer.add_scalar("lr", sched.get_last_lr()[0], done)
        print(f"更新 {done}/{total_updates}  train loss {run_loss / n_run:.3f}  lr {sched.get_last_lr()[0]:.1e}  "
              f"{sec:.1f} 秒/次  还剩约 {sec * (total_updates - done) / 60:.0f} 分钟  "
              f"内存 {torch.mps.driver_allocated_memory() / 1e9:.1f} GB", flush=True)
        run_loss, n_run = 0.0, 0
    if done % args.eval_every == 0 or done == last:
        v = val_loss(); writer.add_scalar("loss/val", v, done)
        save(done)
        print(f"  ── 验证集 loss {v:.3f}，已存档 ckpt/{args.out}_latest", flush=True)

if args.max_updates:
    raise SystemExit("冒烟测试结束（没有存 ckpt/final）")
model.save_pretrained(f"ckpt/{args.out}")
writer.close()
print(f"训练完成，模型存在 ckpt/{args.out}，总用时 {(time.time() - t0) / 60:.0f} 分钟")
