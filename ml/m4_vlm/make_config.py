"""生成 LoRA 训练配置 lora_config.json：语言部分 + 视觉部分都挂 LoRA 小矩阵。

为什么要列视觉层的全名：mlx-vlm 只认得语言部分的短名字（q_proj 这种），
视觉部分（vision_tower）得写完整路径，24 层 × 4 个 + 合并层 = 104 个，手写太长，用脚本生成。
"""
import json

language = ["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"]   # 36 层都有
vision = [f"vision_tower.blocks.{i}.{m}" for i in range(24)
          for m in ("attn.qkv", "attn.proj", "mlp.linear_fc1", "mlp.linear_fc2")]
vision += [f"vision_tower.merger.linear_fc{k}" for k in (1, 2)]                        # 把图像特征"翻译"给语言部分
vision += [f"vision_tower.deepstack_merger_list.{j}.linear_fc{k}" for j in range(3) for k in (1, 2)]

config = dict(target_modules=language + vision)
json.dump(config, open("lora_config.json", "w"), indent=1)
print(f"语言 {len(language)} 种 × 36 层，视觉 {len(vision)} 个，已写入 lora_config.json")
