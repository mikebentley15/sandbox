#!/bin/bash
set -e

DOCKERBASE="mikebentley15/"
DART_VERSION=6.9
GAZEBO_VERSION=11.0

docker build -t ${DOCKERBASE}dart -f 01-dart-fromsource.dockerfile .
docker tag ${DOCKERBASE}dart:latest ${DOCKERBASE}dart:${DART_VERSION}

docker build -t ${DOCKERBASE}gazebo -f 02-gazebo-fromsource.dockerfile .
docker tag ${DOCKERBASE}gazebo:latest ${DOCKERBASE}gazebo:${GAZEBO_VERSION}

docker build -t ${DOCKERBASE}gzweb -f 03-gzweb-fromsource.dockerfile .
docker tag ${DOCKERBASE}gzweb:latest ${DOCKERBASE}gzweb:${GAZEBO_VERSION}

docker build -t ${DOCKERBASE}gzllama -f 04-gzllama-fromsource.dockerfile .
docker tag ${DOCKERBASE}gzllama:latest ${DOCKERBASE}gzllama:${GAZEBO_VERSION}

