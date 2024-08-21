#!/usr/bin/env bash

set -eou pipefail

function videosize() {
  ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of compact "${@}"
}

videosize "${@}"

