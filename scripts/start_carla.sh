#!/usr/bin/env bash
# Start a headless CARLA server on this (AWS GPU) machine.
#
#   scripts/start_carla.sh                 # port 2000, Docker image
#   PORT=3000 scripts/start_carla.sh       # a second server for a second person (ports 3000-3002)
#   CARLA_ROOT=~/CARLA_0.9.16 scripts/start_carla.sh   # use a native install instead of Docker
#
# Only ONE client may tick a CARLA server in synchronous mode, so each person running the
# stack at the same time needs their own server (own port, or own instance).
set -euo pipefail
VERSION="${CARLA_VERSION:-0.9.16}"
PORT="${PORT:-2000}"
QUALITY="${QUALITY:-Low}"     # Low is plenty for planning; use Epic for camera perception work
ARGS="-RenderOffScreen -nosound -quality-level=${QUALITY} -carla-rpc-port=${PORT}"

if [[ -n "${CARLA_ROOT:-}" ]]; then
  echo "Starting native CARLA from ${CARLA_ROOT} on port ${PORT}"
  exec "${CARLA_ROOT}/CarlaUE4.sh" ${ARGS}
fi

NAME="carla-${PORT}"
docker rm -f "${NAME}" >/dev/null 2>&1 || true
echo "Starting carlasim/carla:${VERSION} as container ${NAME} on port ${PORT}"
docker run -d --name "${NAME}" --privileged --gpus all --net=host \
  "carlasim/carla:${VERSION}" bash ./CarlaUE4.sh ${ARGS}
echo "Waiting for the server..."
for _ in $(seq 1 60); do
  if python3 -c "import carla,sys; c=carla.Client('localhost',${PORT}); c.set_timeout(2.0); c.get_server_version()" 2>/dev/null; then
    echo "CARLA ready on port ${PORT}"; exit 0
  fi
  sleep 2
done
echo "CARLA did not answer within 2 minutes; check: docker logs ${NAME}" >&2
exit 1
