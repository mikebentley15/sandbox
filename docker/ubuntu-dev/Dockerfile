FROM ubuntu:16.04
MAINTAINER mikebentley15@gmail.com

# Setup development environment
RUN apt-get update && \
    apt-get install -y \
      cmake \
      curl \
      g++ \
      gcc \
      git \
      htop \
      make \
      moreutils \
      ninja-build \
      openssh-server \
      python3 \
      screen \
      sudo \
      tmux \
      tree \
      vim \
      vim-gtk \
      wget \
      && \
    rm -rf /var/lib/apt/lists/*

# Install pystruct
RUN apt-get update && \
    apt-get install -y \
      build-essential \
      python-dev \
      python-setuptools \
      python-numpy \
      python-scipy \
      libatlas-dev \
      libatlas3-base \
      python-cvxopt \
      python-pip \
      ipython \
      && \
    rm -rf /var/lib/apt/lists/* && \
    pip install --upgrade pip && \
    pip install --upgrade \
      scikit-learn \
      pystruct \
      pyqpbo

# More development tools
RUN apt-get update && \
    apt-get install -y \
      gdb \
      bsdmainutils \
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

RUN git clone https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim && \
    vim +PluginInstall +qall

COPY Dockerfile Dockerfile-ubuntu-dev

CMD ["/bin/bash"]
