#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

# Create the X11 authority file
xauth nlist $DISPLAY | \
  sed -e 's/^..../ffff/' | \
  xauth -f $XAUTH nmerge -

hardware_3d="--device /dev/dri"
audio="--device /dev/snd"
webcam="--device /dev/video0"
timezone="-v /etc/localtime:/etc/localtime:ro"

docker run \
  -v $XSOCK:$XSOCK \
  -v $XAUTH:$XAUTH \
  -e XAUTHORITY=$XAUTH \
  -e DISPLAY \
  $hardware_3d \
  $audio \
  $webcam \
  $timezone \
  -it \
  --rm \
  --name ompl-temporary \
  ompl:latest
