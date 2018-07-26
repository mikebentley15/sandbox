#!/bin/bash

docker run \
  --detach \
  --hostname formal.cs.utah.edu \
  --publish 8021:8021 \
  --publish 8022:22 \
  --name gitlab \
  --restart always \
  --volume /var/gitlab/config:/etc/gitlab \
  --volume /var/gitlab/logs:/var/log/gitlab \
  --volume /var/gitlab/data:/var/opt/gitlab \
  gitlab/gitlab-ce
