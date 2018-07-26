#!/bin/bash

docker run \
  --detach \
  --name jupyterhub \
  --restart always \
  -v ~/jupyter-users.txt:/srv/jupyterhub/users.txt \
  -v /var/jupyterhub/home:/home \
  -e VIRTUAL_ROOT=/jupyterhub/ \
  -e VIRTUAL_PORT=8000 \
  -e VIRTUAL_DIR=/jupyterhub/ \
  -e VIRTUAL_HOST=localhost,formal.cs.utah.edu \
  jupyterhub-mod

