FROM ros:jazzy-ros-base@sha256:386d06ec6d4188f731bae5678e07b4cb64a4e4d4152090c0bd1f881dcf7706f5
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config libapr1-dev python3-dev python3-numpy python3-yaml \
    python3-colcon-common-extensions libeigen3-dev libpcl-dev \
    ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-rviz2 \
    ros-jazzy-rosbag2-storage-mcap ros-jazzy-std-srvs ros-jazzy-ament-cmake-auto \
    && rm -rf /var/lib/apt/lists/*
COPY src/aac_2026/docker/f1tenth/vendor/Livox-SDK2 /tmp/Livox-SDK2
RUN cmake -S /tmp/Livox-SDK2 -B /tmp/livox-build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/livox-build --parallel 2 \
    && cmake --install /tmp/livox-build && ldconfig \
    && rm -rf /tmp/Livox-SDK2 /tmp/livox-build
WORKDIR /ws
COPY src/aac_2026/docker/f1tenth/entrypoint.sh /fastlio-entrypoint.sh
ENTRYPOINT ["/fastlio-entrypoint.sh"]
CMD ["bash"]
