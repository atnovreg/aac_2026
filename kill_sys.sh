#!/bin/bash
CONTAINER_NAME="aac-2026-container-all"
echo "Stopping ${CONTAINER_NAME} ..."
if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  docker exec "${CONTAINER_NAME}" bash -lc 'cd /ros_ws/src/aac_2026 && bash stop.sh' || true
fi
docker rm -f "${CONTAINER_NAME}"
echo "Done!"
