# 演示素材

| 文件 | 内容 |
|---|---|
| `p1_demo.gif` | P1 演示动图（README 首页使用，建议 < 10 MB） |
| `p1_demo.mp4` | P1 演示视频原片（可选，较大时不提交，上传到视频网站后放链接） |

## 录制建议（30–60 秒）
1. 启动 `ros2 launch robot_sim nav_demo.launch.py`，把 Gazebo 和 RViz 窗口左右并排
2. 录制内容：小车自动开往默认目标 → 在 RViz 点 2–3 个需要绕路的目标 → 点一个障碍物内的目标（被拒绝、停车）
3. Windows 11 录屏：`Win + Shift + R`（截图工具录屏）或 `Win + Alt + R`（Xbox Game Bar，保存在"视频\捕获"）

## mp4 转 GIF（WSL 中执行）
```bash
sudo apt install ffmpeg
ffmpeg -i p1_demo.mp4 -vf "fps=10,scale=960:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse" p1_demo.gif
```
`fps=10` 每秒 10 帧、`scale=960:-1` 宽度 960 像素：文件太大时可以调小这两个数。
