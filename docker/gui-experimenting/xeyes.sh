#!/bin/bash

xhost +local:root

docker run \
  -d \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ubuntu:latest \
  sh -c 'apt-get update && apt-get install -qqy x11-apps && xeyes'
