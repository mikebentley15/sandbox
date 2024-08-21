#!/usr/bin/env bash

set -eo pipefail

function cropvideo() {
  local input="${1}"
  local width="${2}"
  local height="${3}"
  local x="${4}"
  local y="${5}"
  local output="${6}"
  shift 6
  local remaining=("${@}")

  ffmpeg -y \
    -i "${input}" \
    -filter:v "crop=${width}:${height}:${x}:${y}" \
    "${remaining[@]}" \
    "${output}"
}

echo "${1}"
./dimensions.bash "${1}"
cropvideo "${@}"
