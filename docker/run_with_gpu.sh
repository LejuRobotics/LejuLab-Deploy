#!/bin/bash
set -e

xhost +

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

CCACHE_DIR="${HOME}/.ccache"
mkdir -p "$CCACHE_DIR"

BASE_IMAGE_URL="https://kuavo.lejurobot.com/docker_images/kuavo_opensource_mpc_wbc_img_latest.tar.gz"
BASE_IMAGE_TARBALL_NAME="kuavo_opensource_mpc_wbc_img_latest.tar.gz"
BASE_IMAGE_REPO="kuavo_opensource_mpc_wbc_img"
DEFAULT_IMAGE_NAME="lejulab_platform:dev"

DIR_HASH=$(echo "$PARENT_DIR" | md5sum | cut -c1-8)
echo "Directory $PARENT_DIR hash: $DIR_HASH"
CONTAINER_NAME="lejulab_container_GPU_${DIR_HASH}"
CONFLICT_CONTAINER_NAME="lejulab_container_${DIR_HASH}"

get_base_image_name() {
    docker images "${BASE_IMAGE_REPO}" --format "{{.Repository}}:{{.Tag}}" | sort -V | tail -n1
}

find_latest_image() {
    docker images "$1" --format "{{.Repository}}:{{.Tag}}" | sort -V | tail -n1
}

print_gpu_setup_instructions() {
    cat <<'EOF'

Host GPU support for Docker is not configured yet.
Please finish the following setup first, then rerun ./docker/run_with_gpu.sh:

  sudo apt-get update
  sudo apt-get install -y ca-certificates curl gnupg

  distribution=$(. /etc/os-release; echo ${ID}${VERSION_ID})

  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

  curl -s -L https://nvidia.github.io/libnvidia-container/${distribution}/libnvidia-container.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

  sudo apt-get update
  sudo apt-get install -y nvidia-container-toolkit

  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker

  sudo docker info | sed -n '/Runtimes/,+6p'
  sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
EOF
}

install_gpu_runtime() {
    echo "Installing NVIDIA Container Toolkit on the host..."
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl gnupg

    local distribution
    distribution=$(. /etc/os-release; echo "${ID}${VERSION_ID}")

    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
        | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

    curl -s -L "https://nvidia.github.io/libnvidia-container/${distribution}/libnvidia-container.list" \
        | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
        | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
}

ensure_gpu_runtime() {
    if ! command -v nvidia-smi >/dev/null 2>&1; then
        echo "Error: nvidia-smi is not available on the host. Please install a working NVIDIA driver first."
        exit 1
    fi

    if ! nvidia-smi -L >/dev/null 2>&1; then
        echo "Error: nvidia-smi cannot access the host GPU right now."
        exit 1
    fi

    local missing_setup=0
    if ! command -v nvidia-ctk >/dev/null 2>&1; then
        echo "Error: nvidia-ctk was not found."
        missing_setup=1
    fi

    if ! docker info --format '{{range $k, $v := .Runtimes}}{{println $k}}{{end}}' 2>/dev/null | grep -qx 'nvidia'; then
        echo "Error: Docker runtime 'nvidia' is not registered."
        missing_setup=1
    fi

    if [ "${missing_setup}" -ne 0 ]; then
        print_gpu_setup_instructions
        read -r -p "Would you like this script to install/configure NVIDIA Container Toolkit now? (yes/no): " response
        if [[ "$response" =~ ^([yY][eE][sS])$ ]]; then
            install_gpu_runtime
        else
            exit 1
        fi
    fi

    if ! command -v nvidia-ctk >/dev/null 2>&1; then
        echo "Error: nvidia-ctk is still unavailable after setup."
        exit 1
    fi

    if ! docker info --format '{{range $k, $v := .Runtimes}}{{println $k}}{{end}}' 2>/dev/null | grep -qx 'nvidia'; then
        echo "Error: Docker runtime 'nvidia' is still not registered after setup."
        print_gpu_setup_instructions
        exit 1
    fi
}

ensure_default_image() {
    if docker image inspect "${DEFAULT_IMAGE_NAME}" >/dev/null 2>&1; then
        return 0
    fi

    local base_image_name
    base_image_name="$(get_base_image_name)"

    if [[ -z "${base_image_name}" ]]; then
        echo -e "\033[33mWarning: No '${BASE_IMAGE_REPO}' Docker image found.\033[0m"
        read -r -p "The script can attempt to automatically download and import the base image. Would you like to proceed? (yes/no): " response
        if [[ ! "$response" =~ ^([yY][eE][sS])$ ]]; then
            echo "Okay. Please download/import '${BASE_IMAGE_REPO}' manually, or run './docker/build.sh' after loading it."
            exit 1
        fi

        local download_path="${SCRIPT_DIR}/${BASE_IMAGE_TARBALL_NAME}"
        echo "Downloading Docker image from ${BASE_IMAGE_URL}..."
        if ! wget -O "${download_path}" "${BASE_IMAGE_URL}"; then
            echo -e "\033[31mError: Failed to download Docker image from ${BASE_IMAGE_URL}.\033[0m"
            exit 1
        fi

        echo "Download successful. Loading image into Docker..."
        if ! sudo docker load -i "${download_path}"; then
            echo -e "\033[31mError: Failed to load Docker image from ${download_path}.\033[0m"
            rm -f "${download_path}"
            exit 1
        fi
        rm -f "${download_path}"

        base_image_name="$(get_base_image_name)"
        if [[ -z "${base_image_name}" ]]; then
            echo -e "\033[31mError: Failed to find '${BASE_IMAGE_REPO}' after loading.\033[0m"
            exit 1
        fi
        echo -e "\033[32mSuccessfully loaded base image: ${base_image_name}\033[0m"
    fi

    echo "Building '${DEFAULT_IMAGE_NAME}' from base image '${base_image_name}' ..."
    if ! "${SCRIPT_DIR}/build.sh" --base-image "${base_image_name}" --tag dev; then
        echo -e "\033[31mError: Failed to build '${DEFAULT_IMAGE_NAME}'.\033[0m"
        exit 1
    fi
}

resolve_image_name() {
    if [[ -n "${1:-}" ]]; then
        local user_image_name="$1"
        local candidate_image_name="$user_image_name"

        echo "User provided image name: ${user_image_name}"
        if [[ "${user_image_name}" != *":"* ]]; then
            echo "No tag provided for '${user_image_name}'. Attempting to find the latest version locally..."
            candidate_image_name="$(find_latest_image "${user_image_name}")"
            if [[ -z "${candidate_image_name}" ]]; then
                candidate_image_name="${user_image_name}"
            fi
        fi

        if ! docker image inspect "${candidate_image_name}" >/dev/null 2>&1; then
            echo -e "\033[31mError: User-provided image '${candidate_image_name}' not found locally.\033[0m"
            exit 1
        fi

        IMAGE_NAME="${candidate_image_name}"
        echo "Found user-provided image: ${IMAGE_NAME}"
        return 0
    fi

    ensure_default_image
    IMAGE_NAME="${DEFAULT_IMAGE_NAME}"
}

show_container_info() {
    local div_line="━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "\n$div_line"
    echo -e "📌 \033[34mContainer Info\033[0m: $CONTAINER_NAME"
    echo -e "📂 \033[32mWorking Directory\033[0m:"
    echo -e "   $PARENT_DIR"
    echo -e "🔗 \033[33mMounted Volumes\033[0m:"
    docker inspect -f '{{range .Mounts}}   {{.Source}} → {{.Destination}}{{println}}{{end}}' "$CONTAINER_NAME"
    echo -e "$div_line\n"
}

ensure_no_conflicting_container() {
    if [[ -z "$(docker ps -aq -f name="^${CONFLICT_CONTAINER_NAME}$")" ]]; then
        return 0
    fi

    echo -e "\033[33mWarning: normal container '${CONFLICT_CONTAINER_NAME}' already exists.\033[0m"
    read -r -p "run.sh and run_with_gpu.sh cannot keep containers at the same time. Remove '${CONFLICT_CONTAINER_NAME}' and continue? (yes/no): " response
    if [[ "$response" =~ ^([yY][eE][sS])$ ]]; then
        docker rm -f "${CONFLICT_CONTAINER_NAME}"
    else
        echo "Please remove '${CONFLICT_CONTAINER_NAME}' first, then rerun ./docker/run_with_gpu.sh."
        exit 1
    fi
}

ensure_no_conflicting_container
ensure_gpu_runtime
resolve_image_name "${1:-}"

if [[ $(docker ps -aq -f ancestor="${IMAGE_NAME}" -f name="${CONTAINER_NAME}") ]]; then
    echo "Container '${CONTAINER_NAME}' based on image '${IMAGE_NAME}' already exists."
    if [[ $(docker ps -aq -f status=exited -f name="${CONTAINER_NAME}") ]]; then
        echo "Restarting exited container '$CONTAINER_NAME' ..."
        docker start "$CONTAINER_NAME"
    fi
    show_container_info
    echo "Exec into container '$CONTAINER_NAME' ..."
    docker exec -it "$CONTAINER_NAME" zsh
else
    echo "Creating a new container '${CONTAINER_NAME}' based on image '${IMAGE_NAME}' ..."
    docker run -it --net host --gpus all \
        --runtime nvidia \
        --name "$CONTAINER_NAME" \
        --privileged \
        -v /dev:/dev \
        -v "${HOME}/.ros:/root/.ros" \
        -v "$CCACHE_DIR:/root/.ccache" \
        -v "$PARENT_DIR:/root/lejulab" \
        -v "${HOME}/.config/lejuconfig:/root/.config/lejuconfig" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all,display \
        -e CARB_GRAPHICS_API=vulkan \
        -e GDK_SYNCHRONIZE=1 \
        --group-add=dialout \
        --ulimit rtprio=99 \
        --cap-add=sys_nice \
        -e DISPLAY="$DISPLAY" \
        -e ROBOT_VERSION=46 \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        "${IMAGE_NAME}" \
        zsh
fi
