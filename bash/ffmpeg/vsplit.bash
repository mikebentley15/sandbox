#!/usr/bin/env bash

set -eou pipefail

function joinvideo() {
  local firstin="${1}"
  local secondin="${2}"
  local output="${3}"

  ffmpeg -y \
    -i "${firstin}" \
    -i "${secondin}" \
    -filter_complex "[0:v] [1:v] hstack=inputs=2" \
    "${output}"
}

echo "${1}"
./dimensions.bash "${1}"
echo "${2}"
./dimensions.bash "${2}"
joinvideo "${@}"
