#!/usr/bin/env bash
# 一键启动: Dingo 容器(ROS + bridge + rosbridge) + 宿主机 AI 视觉
set -e

CONTAINER="dingo_robot_v2"
IMAGE="dingo_image:pi5-gpiod"
GESTURE_PY="/home/htt/gesture_v2_ros.py"

# 如果你知道虚拟环境路径，直接写这里；如果留空，脚本会自动查找。
VENV_ACTIVATE="/home/htt/dingo-vision-venv/bin/activate"

echo "[start] checking vision virtual environment..."

if [ -z "${VENV_ACTIVATE}" ]; then
    if [ -f "/home/htt/dingo-vision-venv/bin/activate" ]; then
        VENV_ACTIVATE="/home/htt/dingo-vision-venv/bin/activate"
    else
        VENV_ACTIVATE="$(find /home/htt -maxdepth 3 -type f -path "*/bin/activate" 2>/dev/null | head -n 1 || true)"
    fi
fi

if [ -z "${VENV_ACTIVATE}" ] || [ ! -f "${VENV_ACTIVATE}" ]; then
    echo "[error] cannot find Python virtual environment."
    echo "[hint] run this command to find it:"
    echo "       find /home/htt -maxdepth 3 -type f -path \"*/bin/activate\" 2>/dev/null"
    exit 1
fi

if [ ! -f "${GESTURE_PY}" ]; then
    echo "[error] gesture python file not found: ${GESTURE_PY}"
    exit 1
fi

echo "[start] using venv: ${VENV_ACTIVATE}"
echo "[start] using gesture script: ${GESTURE_PY}"

# 1) 启动容器，使用 host 网络
if [ -z "$(sudo docker ps -q -f name=^/${CONTAINER}$)" ]; then
    if [ -n "$(sudo docker ps -aq -f name=^/${CONTAINER}$)" ]; then
        echo "[start] starting existing container ${CONTAINER}"
        sudo docker start "${CONTAINER}"
    else
        echo "[start] creating new container ${CONTAINER}"
        sudo docker run -d \
            --name "${CONTAINER}" \
            --network host \
            --privileged \
            --restart unless-stopped \
            -v /dev:/dev \
            -v /sys:/sys \
            -v /run/udev:/run/udev:ro \
            "${IMAGE}"
    fi
else
    echo "[start] container ${CONTAINER} is already running"
fi

# 2) 在容器里启动 dingo.launch，包含 bridge + rosbridge
echo "[start] launching dingo.launch inside container..."

sudo docker exec -d "${CONTAINER}" bash -lc "
    source /opt/ros/noetic/setup.bash &&
    source /root/dingo_ws/devel/setup.bash &&
    roslaunch dingo dingo.launch use_ai_camera:=1 use_joystick:=0
"

# 等 rosbridge 启动
echo "[start] waiting for rosbridge..."
sleep 5

# 3) 在宿主机启动 AI 视觉
echo "[start] launching gesture script on host..."

source "${VENV_ACTIVATE}"
python3 "${GESTURE_PY}"
