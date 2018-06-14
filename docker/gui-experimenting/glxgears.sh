#!/bin/bash

xhost +local:root

# TODO: not working
nvidia-docker run \
  -d \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ubuntu:latest \
  sh -c 'apt-get update && apt-get install -qqy mesa-utils && glxgears'
