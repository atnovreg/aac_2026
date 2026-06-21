ARG TAG

FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04

ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV TORCH_CUDA_ARCH_LIST=12.0
# Persist Hugging Face model downloads in the host-mounted ROS workspace.
ENV HF_HOME=/ros_ws/src/aac_2026/.hf_home
ENV HF_HUB_DISABLE_TELEMETRY=1

ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"] 

## Install from packages list
COPY docker/dockerfiles/apt-packages /tmp/
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages \
    && apt-get clean

# 기본 설정
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3.10 python3-pip python3.10-dev python3.10-venv git wget curl ca-certificates unzip && \
    ln -sf python3.10 /usr/bin/python3 && \
    pip3 install --upgrade pip && \
    pip3 install --no-cache-dir --force-reinstall --no-deps 'setuptools==59.6.0'

# PyTorch install for RTX 50-series / Blackwell sm_120.
# Keep the global Python usable even when WildGS-SLAM's venv is not activated.
RUN pip install --no-cache-dir \
    torch==2.11.0 torchvision==0.26.0 torchaudio==2.11.0 \
    --index-url https://download.pytorch.org/whl/cu128
RUN pip install --no-cache-dir xformers==0.0.35
RUN pip install --no-cache-dir onnxruntime-gpu==1.23.2

## Install ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

#
# install OpenCV (with CUDA)
#
# PyTorch 설치 시 numpy 2.x가 당겨질 수 있으므로, OpenCV 소스 빌드 전에
# numpy를 requirements.txt와 동일한 버전으로 고정합니다.
# (OpenCV Python 바인딩은 빌드 시점의 numpy C ABI에 링크됩니다.)
RUN pip install --no-cache-dir 'numpy==1.26.4'
COPY docker/opencv_install.sh /tmp/opencv_install.sh
RUN cd /tmp && ./opencv_install.sh

# 
# install development packages
#
COPY docker/dockerfiles/ros2-packages /tmp/
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/ros2-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/ros2-packages \
    && apt-get clean


# Additional layer
#######################################################################
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends \
        libzmq3-dev libczmq-dev \
 && rm -rf /var/lib/apt/lists/*

# Python runtime dependencies, including Transformers for EfficientLoFTR.
# ROS 2 Humble colcon-python-setup-py is incompatible with setuptools 80+.
COPY requirements.txt /tmp
# Ubuntu installs blinker through distutils, so pip cannot uninstall it cleanly
# when open3d pulls a newer Flask stack. Install only that package with
# --ignore-installed, then keep normal dependency resolution for everything
# else so the existing cu128 PyTorch wheel is not replaced.
RUN pip install --ignore-installed --no-cache-dir --no-deps 'blinker==1.9.0'
RUN cd /tmp && pip install --no-cache-dir -r requirements.txt
RUN pip install --no-cache-dir --force-reinstall --no-deps \
    'numpy==1.26.4' \
    'setuptools==59.6.0'
RUN python3 - <<'PY'
import setuptools
import torch

assert setuptools.__version__ == '59.6.0', (
    f'Expected setuptools==59.6.0 for ROS 2 Humble ament_python, '
    f'found {setuptools.__version__} at {setuptools.__file__}'
)
assert torch.version.cuda == '12.8', (
    f'Expected the cu128 PyTorch build, found torch={torch.__version__}, '
    f'torch.version.cuda={torch.version.cuda}'
)
print(f'setuptools dependency check: {setuptools.__version__}')
print(f'PyTorch dependency check: {torch.__version__}, CUDA {torch.version.cuda}')
PY

ARG UID=1000
ARG USER
ENV USER=$USER
RUN useradd -u $UID -ms /bin/bash $USER | echo $USER
ENV CMAKE_MODULE_PATH=/usr/local/share/cmake-3.22/Modules

# RUN useradd -ms /bin/bash usrg
COPY . /ros_ws/src/aac_2026
RUN mkdir /ros_ws/build
RUN mkdir /ros_ws/install
RUN mkdir /ros_ws/map

# Upgrade colcon-python-setup-py: apt version has bug with --symlink-install calling
# 'python setup.py --editable' which is not recognized. Newer pip version fixes this.
# Pin coverage<7.0: coverage 7.x removed coverage.types which numba depends on.

RUN chown -R ${USER}:${USER} /ros_ws
RUN chmod 755 /ros_ws

# SUDOERS settings
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

# bashrc setting finalize
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /home/$USER/.bashrc && \
    echo '[ -f "/ros_ws/install/setup.bash" ] && source "/ros_ws/install/setup.bash"' >> /home/$USER/.bashrc && \
    echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-3}' >> /home/$USER/.bashrc

WORKDIR /ros_ws

COPY docker/env.sh /env.sh
RUN chmod +x /env.sh
ENTRYPOINT ["/env.sh"]
