#!/bin/bash

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

docker stop jupyterhub
docker rm jupyterhub
docker build "$SCRIPT_DIR" --tag jupyterhub-mod
bash "$SCRIPT_DIR"/run.sh

