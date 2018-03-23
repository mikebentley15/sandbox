#!/bin/bash

CONTAINER_NAME=archflit-container

if docker ps | grep -w $CONTAINER_NAME &>/dev/null; then
  echo "Arch FLiT is already running in another shell..."
elif docker ps -a | grep $CONTAINER_NAME &>/dev/null; then
  echo "Starting Arch FLiT using the current shell"
  docker start -ai $CONTAINER_NAME
else
  # In order for the license to work, it must have the same network mac address
  echo "Creating Arch FLiT container and running using the current shell"
  docker run \
    --mac-address=00:23:ae:b3:33:48 \
    -it \
    -v /opt/intel:/opt/intel \
    --name $CONTAINER_NAME \
    arch-flit
fi

