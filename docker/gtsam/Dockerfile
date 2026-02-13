# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

ARG DOCKER_USER=frostlab-docker
ENV DOCKER_USER=$DOCKER_USER

# Create the user
RUN useradd -ms /bin/bash "$DOCKER_USER" \
    && usermod -aG sudo "$DOCKER_USER" \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && echo "$DOCKER_USER:$DOCKER_USER" | chpasswd

# Install system dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt update && apt install -y --no-install-recommends \
    # Install basic system utilities - Nelson Durrant, Jan 2026
    curl wget git vim nano python3-pip python3-venv gosu procps ccache \
    # Install remote access utilities - Nelson Durrant, Jan 2026
    openssh-server mosh tmux xclip \
    # Install graphical utilities - Nelson Durrant, Jan 2026
    x11-apps x11-utils x11-xserver-utils xauth

# Install GTSAM (build from source for ARM)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt update && apt install -y --no-install-recommends \
    build-essential cmake libboost-dev libtbb-dev \
    libboost-serialization-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-date-time-dev \
    libboost-timer-dev \
    libboost-chrono-dev \
    libboost-regex-dev
WORKDIR /home/$DOCKER_USER
RUN git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git \
    && cmake -B gtsam/build -S gtsam \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_WITH_TBB=ON \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    && make -C gtsam/build -j$(nproc) \
    && sudo make -C gtsam/build install \
    && sudo ldconfig \
    && rm -rf gtsam

CMD ["bash"]
