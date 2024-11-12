#!/usr/bin/env bash

docker run \
  -it \
  -v /home/bentley:/home/bentley \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --name latex \
  latex:18.04 \
  bash
