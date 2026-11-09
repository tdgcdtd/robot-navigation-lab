"""零样本感知流程：一句话 → 框（Grounding DINO base）→ 轮廓（SAM 2.1 large）→ 3D 坐标（深度反投影）。

谁负责什么：
- Grounding DINO：同时看图和文字，输出"文字描述的东西在哪"的框 + 置信度。不会抠轮廓
- SAM 2.1：给它一个框，抠出框里物体的精确像素轮廓。不懂文字
- 我们的代码：轮廓内的像素 + 深度 → 世界坐标（scene.pixel_to_world），再从顶面推出方块中心
"""
import time
import numpy as np
import torch
from PIL import Image
from transformers import (AutoModelForZeroShotObjectDetection, AutoProcessor,
                          Sam2Model, Sam2Processor)
from scene import CUBE, pixel_to_world

DEVICE = "mps" if torch.backends.mps.is_available() else "cpu"
DINO_ID = "IDEA-Research/grounding-dino-base"
SAM_ID = "facebook/sam2.1-hiera-large"


class Perceiver:
    def __init__(self, device=DEVICE, dino_path=DINO_ID, edge=800):
        """dino_path：默认用官方原版；微调后传入自己存的模型文件夹。
        edge：DINO 输入图片的短边像素数（官方默认 800；微调用 480 省内存，评估时要和训练一致）。"""
        self.device = device
        self.dino_proc = AutoProcessor.from_pretrained(DINO_ID)
        self.dino_proc.image_processor.size = {"shortest_edge": edge, "longest_edge": 1333}
        self.dino = AutoModelForZeroShotObjectDetection.from_pretrained(dino_path).to(device).eval()
        self.sam_proc = Sam2Processor.from_pretrained(SAM_ID)
        self.sam = Sam2Model.from_pretrained(SAM_ID).to(device).eval()

    @torch.no_grad()
    def detect(self, rgb, text, threshold=0.3):
        """文字 → 框。返回按置信度从高到低排好的 [(score, (x0,y0,x1,y1)), ...]"""
        image = Image.fromarray(rgb)
        # Grounding DINO 的约定：小写、以句号结尾，如 "a red cube."
        inputs = self.dino_proc(images=image, text=text.lower().rstrip(".") + ".",
                                return_tensors="pt").to(self.device)
        out = self.dino(**inputs)
        res = self.dino_proc.post_process_grounded_object_detection(
            out, inputs.input_ids, threshold=threshold, text_threshold=threshold,
            target_sizes=[image.size[::-1]])[0]
        dets = [(s.item(), tuple(b.tolist())) for s, b in zip(res["scores"], res["boxes"])]
        return sorted(dets, reverse=True)

    @torch.no_grad()
    def segment(self, rgb, box):
        """框 → 轮廓（H,W 的布尔数组）。"""
        inputs = self.sam_proc(images=Image.fromarray(rgb), input_boxes=[[list(box)]],
                               return_tensors="pt").to(self.device)
        out = self.sam(**inputs, multimask_output=False)
        masks = self.sam_proc.post_process_masks(out.pred_masks.cpu(), inputs["original_sizes"])[0]
        return masks[0, 0].numpy().astype(bool)


def locate(mask, depth, camera):
    """轮廓 + 深度 → 方块中心的世界坐标。

    相机斜着往下看，能看到方块的顶面和侧面。取最高的那层点（顶面），
    顶面中心再往下半个方块，就是方块中心。用中位数而不是平均值：边缘抠歪的几个点影响不大。
    """
    vs, us = np.nonzero(mask)
    if len(us) == 0:
        return None
    pts = pixel_to_world(us, vs, depth[vs, us], camera)
    z_top = np.percentile(pts[:, 2], 95)
    top = pts[pts[:, 2] > z_top - 0.005]
    center = np.median(top, axis=0)
    center[2] -= CUBE / 2
    return center


if __name__ == "__main__":
    from scene import random_cubes, render
    t0 = time.time()
    p = Perceiver()
    print(f"加载两个模型用了 {time.time() - t0:.1f} 秒（设备 {p.device}）")

    rng = np.random.default_rng(0)
    cubes = random_cubes(rng, ["red", "blue", "green", "yellow"])
    rgb, depth, seg, cam = render(cubes)
    vis = rgb.astype(float)
    labels = []
    for c in cubes:
        t0 = time.time(); dets = p.detect(rgb, f"{c.color} cube"); t1 = time.time()
        if not dets:
            print(f"{c.color:6s} 没找到"); continue
        score, box = dets[0]
        mask = p.segment(rgb, box); t2 = time.time()
        est = locate(mask, depth, cam)
        err = np.linalg.norm(est - c.pos) * 1000
        print(f"{c.color:6s} 置信度 {score:.2f}  框 {tuple(round(v) for v in box)}  "
              f"误差 {err:5.1f} mm  （检测 {t1 - t0:.2f}s，分割 {t2 - t1:.2f}s）")
        vis[mask] = vis[mask] * 0.4 + np.array([255, 255, 255]) * 0.6   # 轮廓内涂白
        labels.append((box, f"{c.color} {score:.2f}"))

    # 存一张结果图：白色 = SAM 抠出的轮廓，青色框 = DINO 的框
    from PIL import ImageDraw
    img = Image.fromarray(vis.astype(np.uint8))
    draw = ImageDraw.Draw(img)
    for box, text in labels:
        draw.rectangle(box, outline=(0, 255, 255), width=2)
        draw.text((box[0], box[1] - 12), text, fill=(0, 255, 255))
    img.save("perceive_result.png")
    print("结果图已存为 perceive_result.png")
