#!/bin/bash
CONTAINER_NAME="aac-2026-container-all"
echo "Stopping ${CONTAINER_NAME} ..."
docker rm -f ${CONTAINER_NAME}
echo "Done!"
