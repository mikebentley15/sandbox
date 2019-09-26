#!/bin/bash
set -e

DOCKERBASE="mikebentley15/"
PAGMO_VERSION=2.11.3
DART_VERSION=6.9
GAZEBO_VERSION=11.0
GZWEB_VERSION=1.4

docker build -t ${DOCKERBASE}pagmo2 -f 01-pagmo2-fromsource.dockerfile .
docker tag ${DOCKERBASE}pagmo2:latest ${DOCKERBASE}pagmo2:${PAGMO_VERSION}

docker build -t ${DOCKERBASE}dart -f 02-dart-fromsource.dockerfile .
docker tag ${DOCKERBASE}dart:latest ${DOCKERBASE}dart:${DART_VERSION}

docker build -t ${DOCKERBASE}gazebo -f 03-gazebo-fromsource.dockerfile .
docker tag ${DOCKERBASE}gazebo:latest ${DOCKERBASE}gazebo:${GAZEBO_VERSION}

docker build -t ${DOCKERBASE}gzweb -f 04-gzweb-fromsource.dockerfile .
docker tag ${DOCKERBASE}gzweb:latest ${DOCKERBASE}gzweb:${GZWEB_VERSION}

#docker build -t ${DOCKERBASE}gzllama -f 05-gzllama-fromsource.dockerfile .
#docker tag ${DOCKERBASE}gzllama:latest ${DOCKERBASE}gzllama:${GAZEBO_VERSION}

