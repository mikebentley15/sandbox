FROM ubuntu:xenial
MAINTAINER mikebentley15@gmail.com

# Setup development environment
RUN apt-get update && \
    apt-get install -y \
      bash-completion \
      cmake \
      curl \
      g++ \
      gcc \
      gdb \
      git \
      htop \
      make \
      python3 \
      python3-pip \
      screen \
      sudo \
      tmux \
      tree \
      vim \
      vim-gtk \
      wget \
      && \
    rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --shell /bin/bash --gid sudo dockuser && \
    echo 'dockuser:dockpass' | chpasswd

USER dockuser
WORKDIR /home/dockuser

# Setup my dotfiles
RUN mkdir git && \
    cd git && \
    git clone https://bitbucket.org/mikebentley15/configurations.git && \
    cd configurations && \
    make -f link-config.mk backup && \
    make -f link-config.mk all

RUN rmdir ~/.vim/bundle/Vundle.vim && \
    git clone https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim && \
    vim +PluginInstall +qall

COPY Dockerfile Dockerfile-travis-cheapclone

CMD ["/bin/bash"]
