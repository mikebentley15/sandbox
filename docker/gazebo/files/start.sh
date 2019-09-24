#!/bin/bash
set -e

gzserver --verbose &

cd "$GZWEB_WS"
npm start -p 8080 &

# wait for all background processes to finish
wait
