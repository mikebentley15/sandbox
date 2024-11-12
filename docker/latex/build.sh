#!/bin/bash

set -u # fail on error

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

cd "${SCRIPT_DIR}"

docker build \
  ./ \
  --build-arg user=${USER} \
  --build-arg user_id=$(id -u) \
  --build-arg user_group_id=$(id -g) \
  --build-arg user_shell=${SHELL} \
  --build-arg user_home=${HOME} \
  --tag latex:18.04

echo
echo "Built latex:18.04 ($SECONDS seconds)"
echo
