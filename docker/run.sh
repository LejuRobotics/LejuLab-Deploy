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
LEJULAB_IMAGE_NAME="lejulab_platform:dev"

DIR_HASH=$(echo "$PARENT_DIR" | md5sum | cut -c1-8)
echo "Directory $PARENT_DIR hash: $DIR_HASH"
CONTAINER_NAME="lejulab_container_${DIR_HASH}"
CONFLICT_CONTAINER_NAME="lejulab_container_GPU_${DIR_HASH}"

get_base_image_name() {
    docker images "${BASE_IMAGE_REPO}" --format "{{.Repository}}:{{.Tag}}" | sort -V | tail -n1
}

ensure_lejulab_image() {
    if docker image inspect "${LEJULAB_IMAGE_NAME}" >/dev/null 2>&1; then
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

    echo "Building '${LEJULAB_IMAGE_NAME}' from base image '${base_image_name}' ..."
    if ! "${SCRIPT_DIR}/build.sh" --base-image "${base_image_name}" --tag dev; then
        echo -e "\033[31mError: Failed to build '${LEJULAB_IMAGE_NAME}'.\033[0m"
        exit 1
    fi
}

ensure_no_conflicting_container() {
    if [[ -z "$(docker ps -aq -f name="^${CONFLICT_CONTAINER_NAME}$")" ]]; then
        return 0
    fi

    echo -e "\033[33mWarning: GPU container '${CONFLICT_CONTAINER_NAME}' already exists.\033[0m"
    read -r -p "run.sh and run_with_gpu.sh cannot keep containers at the same time. Remove '${CONFLICT_CONTAINER_NAME}' and continue? (yes/no): " response
    if [[ "$response" =~ ^([yY][eE][sS])$ ]]; then
        docker rm -f "${CONFLICT_CONTAINER_NAME}"
    else
        echo "Please remove '${CONFLICT_CONTAINER_NAME}' first, then rerun ./docker/run.sh."
        exit 1
    fi
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

ensure_no_conflicting_container
ensure_lejulab_image

if [[ $(docker ps -aq -f ancestor="${LEJULAB_IMAGE_NAME}" -f name="${CONTAINER_NAME}") ]]; then
    echo "Container '${CONTAINER_NAME}' based on image '${LEJULAB_IMAGE_NAME}' already exists."
    if [[ $(docker ps -aq -f status=exited -f name="${CONTAINER_NAME}") ]]; then
        echo "Restarting exited container '$CONTAINER_NAME' ..."
        docker start "$CONTAINER_NAME"
    fi
    show_container_info
    echo "Exec into container '$CONTAINER_NAME' ..."
    docker exec -it "$CONTAINER_NAME" zsh
else
    echo "Creating a new container '${CONTAINER_NAME}' based on image '${LEJULAB_IMAGE_NAME}' ..."
    docker run -it --net host \
        --name "$CONTAINER_NAME" \
        --privileged \
        -v /dev:/dev \
        -v "${HOME}/.ros:/root/.ros" \
        -v "$CCACHE_DIR:/root/.ccache" \
        -v "$PARENT_DIR:/root/lejulab" \
        -v "${HOME}/.config/lejuconfig:/root/.config/lejuconfig" \
        --group-add=dialout \
        --ulimit rtprio=99 \
        --cap-add=sys_nice \
        -e DISPLAY="$DISPLAY" \
        -e ROBOT_VERSION=46 \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        "${LEJULAB_IMAGE_NAME}" \
        zsh
fi
