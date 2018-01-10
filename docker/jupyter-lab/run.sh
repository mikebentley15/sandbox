#!/bin/bash

CONTAINER_NAME=jupyterlab-container

if docker ps | grep $CONTAINER_NAME &>/dev/null; then
  echo "Jupyter Lab is already running in another shell..."
elif docker ps -a | grep $CONTAINER_NAME &>/dev/null; then
  echo "Starting Jupyter Lab using the current shell"
  docker start -a $CONTAINER_NAME
else
  echo "Creating Jupyter Lab container and running using the current shell"
  docker run \
    -p 127.0.0.1:8000:8000 \
    --name $CONTAINER_NAME \
    jupyter-lab
fi
