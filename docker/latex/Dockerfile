FROM ubuntu:18.04
MAINTAINER mikebentley15@gmail.com

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      build-essential \
      bash-completion \
      texlive-full \
      && \
    rm -rf /var/lib/apt/lists/*

ARG user
ARG user_id
ARG user_group_id
ARG user_shell=/bin/bash
ARG user_home=/home/${user}

# Create user with sudo access.  Sudo does not require a password
RUN groupadd --gid ${user_group_id} sudo-nopw && \
    useradd \
      --home-dir ${user_home} \
      --create-home \
      --shell ${user_shell} \
      --gid sudo-nopw \
      --groups sudo,dialout \
      --uid ${user_id} \
      ${user} \
      && \
    passwd -d ${user} && \
    mkdir -p /etc/sudoers.d && \
    echo "%sudo-nopw ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudo-nopw-group

# Have some way to know what kind of docker container we're in
RUN echo "ros2-dev" > /etc/docker-name

USER ${user}
WORKDIR ${user_home}
