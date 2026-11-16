# P2 / P3 机械臂环境：Ubuntu 22.04 + ROS 2 Humble + MoveIt 2 + Gazebo Classic
# 越疆官方包 DOBOT_6Axis_ROS2_V4 只支持 Humble，所以放进容器里用。
#
# 不直接用 osrf/ros 镜像：从 Docker Hub / 镜像站拉几 GB 的大镜像经常中途失败。
# 改为从本地已有的 ubuntu:22.04 开始，用清华 apt 源安装 ROS。
# 安装拆成几个 RUN：每一步成功后都会被缓存，某一步失败时，重新 build 只需从那一步开始。

FROM ubuntu:22.04

# noninteractive：安装时不弹出交互问题（例如选时区），否则 build 会卡住失败
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# 第 1 步：Ubuntu 软件源换成清华镜像；下载失败自动重试 5 次
RUN sed -i 's@//archive.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g; s@//security.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list \
    && echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80retries

# 第 2 步：装好证书后把 Ubuntu 源改成 https（这台电脑上 http 80 端口经常超时）；
#         再添加 ROS 2 软件源（清华镜像）和它的签名密钥
RUN apt-get update && apt-get install -y curl gnupg ca-certificates sudo \
    && sed -i 's@http://mirrors.tuna@https://mirrors.tuna@g' /etc/apt/sources.list \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
         > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# 第 3 步：ROS 2 Humble 桌面版（含 RViz）+ 编译工具（最大的一步）
RUN apt-get update && apt-get install -y \
      ros-humble-desktop \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 第 4 步：官方包 README 和各 package.xml 要求的依赖
RUN apt-get update && apt-get install -y \
      ros-humble-moveit \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros2-control \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-joint-state-publisher-gui \
      ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# 第 5 步：建一个和 WSL 里同 ID（1000）的普通用户。
# 否则容器里以 root 编译，挂载目录里生成的 build/ install/ 在 WSL 里属于 root，删不掉。
RUN useradd -m -u 1000 -s /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev

# 第 6 步（C 项目语音输入）：录音工具 + 本地语音识别
#   pulseaudio-utils：parecord 命令，通过 WSLg 的 PulseAudio 通道拿到 Windows 的麦克风
#   sherpa-onnx：语音识别推理库（自带 onnxruntime，不需要 PyTorch，CPU 就能跑）
#   SenseVoice-Small：阿里开源的语音识别模型，中文效果好、速度快（int8 量化版约 230 MB）
#   放在后面单独一层：前面几层（几 GB 的 ROS）不受影响，重新 build 只做这一步
RUN apt-get update && apt-get install -y pulseaudio-utils python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple sherpa-onnx numpy \
    && mkdir -p /opt/asr && cd /opt/asr \
    && curl -sSL -o sv.tar.bz2 https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17.tar.bz2 \
    && tar xjf sv.tar.bz2 && rm sv.tar.bz2 \
    && mv sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17 sense-voice \
    && rm -f sense-voice/model.onnx

# 第 7 步（C 项目摄像头）：v4l2-ctl 查看 / 设置摄像头（分辨率、自动对焦、曝光……）。OpenCV 和 cv_bridge 在第 3 步已经有了
RUN apt-get update && apt-get install -y v4l-utils && rm -rf /var/lib/apt/lists/*

USER dev
WORKDIR /home/dev/arm_ws

# 每开一个终端自动加载 ROS 环境（相当于主系统里的 ros2on）
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/arm_ws/install/setup.bash ] && source ~/arm_ws/install/setup.bash" >> ~/.bashrc
