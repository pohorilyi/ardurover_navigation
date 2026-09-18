#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="ardurover-navigation"
CONTAINER_NAME="ardurover-navigation"
WORKDIR="/home/developer/ardurover_navigation"

xhost +local:docker >/dev/null 2>&1 || xhost +local: >/dev/null 2>&1 || true

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
STABLE_AUTH="${RUNTIME_DIR}/.ardurover-nav.Xauthority"
HOST_DISPLAY="${DISPLAY:-:0}"

refresh_xauthority() {
  local src=""
  if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
    src="${XAUTHORITY}"
  elif [[ -f "${RUNTIME_DIR}/gdm/Xauthority" ]]; then
    src="${RUNTIME_DIR}/gdm/Xauthority"
  elif compgen -G "${RUNTIME_DIR}/.mutter-Xwaylandauth.*" >/dev/null; then
    src="$(ls -t "${RUNTIME_DIR}"/.mutter-Xwaylandauth.* | head -1)"
  elif [[ -f "${HOME}/.Xauthority" ]]; then
    src="${HOME}/.Xauthority"
  fi

  if [[ -d "${STABLE_AUTH}" ]]; then
    rmdir "${STABLE_AUTH}" 2>/dev/null || true
  fi
  if [[ ! -e "${STABLE_AUTH}" ]]; then
    : >"${STABLE_AUTH}"
  fi
  if [[ -n "${src}" ]]; then
    cat "${src}" >"${STABLE_AUTH}"
  fi
  chmod 600 "${STABLE_AUTH}" 2>/dev/null || true
}

refresh_xauthority

container_has_gpu() {
  docker exec "${CONTAINER_NAME}" bash -lc \
    '[[ -e /dev/dxg || -e /dev/dri/renderD128 || -e /dev/nvidia0 ]]' \
    >/dev/null 2>&1
}

if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  if [[ "$(docker container inspect -f '{{.State.Running}}' "${CONTAINER_NAME}")" != "true" ]]; then
    docker start "${CONTAINER_NAME}" >/dev/null
  fi
  if command -v nvidia-smi >/dev/null 2>&1 && ! container_has_gpu; then
    echo "Existing container has no GPU devices (Gazebo will stay on CPU / llvmpipe)." >&2
    echo "Recreate it from the WSL host:" >&2
    echo "  docker rm -f ${CONTAINER_NAME} && ${ROOT}/docker/run.sh" >&2
  fi
  exec bash "${ROOT}/docker/attach.sh"
fi

GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1; then
  GPU_ARGS+=(
    --gpus=all
    --env=NVIDIA_VISIBLE_DEVICES=all
    --env=NVIDIA_DRIVER_CAPABILITIES=all
  )
fi

# WSL2 GPU: OpenGL is Mesa d3d12 via DXG, not libGLX_nvidia. --gpus=all alone is not enough.
if [[ -e /dev/dxg ]]; then
  GPU_ARGS+=(
    --device=/dev/dxg
    --env=GALLIUM_DRIVER=d3d12
    --env=MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
  )
fi
if [[ -d /usr/lib/wsl ]]; then
  GPU_ARGS+=(
    -v /usr/lib/wsl:/usr/lib/wsl:ro
    --env=LD_LIBRARY_PATH=/usr/lib/wsl/lib
  )
fi
if [[ -d /dev/dri ]]; then
  GPU_ARGS+=(--device=/dev/dri)
fi
if [[ -d /mnt/wslg ]]; then
  GPU_ARGS+=(-v /mnt/wslg:/mnt/wslg)
fi
# Host 'render' GID (often 992 on WSL) has no name in the image and only prints
# "cannot find name for group ID". /dev/dxg is world-writable; skip it.
if getent group video >/dev/null 2>&1; then
  GPU_ARGS+=(--group-add "$(getent group video | cut -d: -f3)")
fi

if ((${#GPU_ARGS[@]})); then
  echo "GPU: passing host devices into the container" >&2
else
  echo "GPU: not detected; Gazebo will use CPU rendering" >&2
fi

docker run -it \
  --name="${CONTAINER_NAME}" \
  --init \
  --network=host \
  "${GPU_ARGS[@]}" \
  --env=DISPLAY="${HOST_DISPLAY}" \
  --env=XAUTHORITY=/home/developer/.Xauthority \
  --env=QT_QPA_PLATFORM=xcb \
  --env=QT_X11_NO_MITSHM=1 \
  --env=ARDUROVER_NAV_ROOT="${WORKDIR}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${STABLE_AUTH}:/home/developer/.Xauthority:ro" \
  -v "${ROOT}:${WORKDIR}" \
  --workdir="${WORKDIR}" \
  --user=developer \
  "${IMAGE_NAME}" \
  bash
