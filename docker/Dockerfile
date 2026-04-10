ARG BASE_IMAGE=kuavo_opensource_mpc_wbc_img:latest
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
ENV ROBOT_VERSION=46

RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime \
    && echo "${TZ}" > /etc/timezone

RUN set -eux; \
    for file in /etc/apt/sources.list.d/*; do \
        [ -f "${file}" ] || continue; \
        if grep -qE 'mirrors\.ustc\.edu\.cn/ros/ubuntu|packages\.ros\.org/ros/ubuntu|snapshots\.ros\.org/noetic|drake-apt\.csail\.mit\.edu' "${file}"; then \
            rm -f "${file}"; \
        fi; \
    done; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        gnupg \
        lsb-release \
        wget; \
    wget -qO- https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.ustc.edu.cn/ros/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros1-latest.list; \
    wget -qO- https://drake-apt.csail.mit.edu/drake.asc \
        | gpg --dearmor --yes -o /etc/apt/trusted.gpg.d/drake.gpg; \
    echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/drake.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ccache \
        cmake \
        git \
        pkg-config \
        python3-catkin-tools \
        zsh \
        build-essential \
        libacl1-dev \
        libmodbus-dev \
        libprotobuf-c-dev \
        libyaml-cpp-dev; \
    if [ ! -d /opt/drake ] && ! dpkg -s drake-dev >/dev/null 2>&1; then \
        apt-get install -y --no-install-recommends drake-dev=1.19.0-1 \
        || apt-get install -y --no-install-recommends drake-dev; \
    fi; \
    rm -rf /var/lib/apt/lists/*

RUN printf '%s\n' \
    '#!/bin/bash' \
    'if [ -n "${ZSH_VERSION:-}" ]; then' \
    '  source /opt/ros/noetic/setup.zsh' \
    'else' \
    '  source /opt/ros/noetic/setup.bash' \
    'fi' \
    'export ROBOT_VERSION=46' \
    'export OpenVINO_DIR="/root/lejulab/src/leju-controllers/leju-rl-controller/3rd_party/openvino_toolkit/runtime/cmake"' \
    'export LD_LIBRARY_PATH="/opt/drake/lib:/root/lejulab/src/leju-mujoco-sim/3rd_party/mujoco-3.2.0/lib:/root/lejulab/src/leju-controllers/leju-rl-controller/3rd_party/onnxruntime_toolkit/onnxruntime-linux-x64-1.18.0/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"' \
    'if [ -f /root/lejulab/src/leju-controllers/leju-rl-controller/3rd_party/openvino_toolkit/setupvars.sh ]; then' \
    '  source /root/lejulab/src/leju-controllers/leju-rl-controller/3rd_party/openvino_toolkit/setupvars.sh' \
    'fi' \
    > /etc/profile.d/lejulab_env.sh \
    && chmod 644 /etc/profile.d/lejulab_env.sh

RUN printf '%s\n' \
    '' \
    'if [ -f /etc/profile.d/lejulab_env.sh ]; then' \
    '  source /etc/profile.d/lejulab_env.sh' \
    'fi' \
    >> /root/.bashrc \
    && printf '%s\n' \
    '' \
    'if [ -f /etc/profile.d/lejulab_env.sh ]; then' \
    '  source /etc/profile.d/lejulab_env.sh' \
    'fi' \
    >> /root/.zshrc \
    && sed -i 's/^export ROBOT_VERSION=.*/export ROBOT_VERSION=46/' /root/.bashrc /root/.zshrc \
    && if ! grep -q '^export ROBOT_VERSION=' /root/.bashrc; then echo 'export ROBOT_VERSION=46' >> /root/.bashrc; fi \
    && if ! grep -q '^export ROBOT_VERSION=' /root/.zshrc; then echo 'export ROBOT_VERSION=46' >> /root/.zshrc; fi

RUN git config --global --add safe.directory /root/lejulab

WORKDIR /root/lejulab

CMD ["zsh"]
