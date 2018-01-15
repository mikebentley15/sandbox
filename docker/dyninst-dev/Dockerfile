FROM dyninst:latest
MAINTAINER mikebentley15@gmail.com

USER root

# Setup development environment
RUN apt-get update && \
    apt-get install -y \
      python2.7 \
      screen \
      vim \
      vim-gtk \
      tree \
      moreutils \
      htop \
      openssh-server \
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

USER dynuser
WORKDIR /home/dynuser

# Setup my dotfiles
RUN mkdir git && \
    cd git && \
    git clone https://bitbucket.org/mikebentley15/configurations.git && \
    cd configurations && \
    make -f link-config.mk backup && \
    make -f link-config.mk all

# Get the source files installed
RUN cd git && \
    git clone https://github.com/mikebentley15/dyninst.git && \
    git clone https://github.com/mikebentley15/toolchain-origin.git && \
    git clone https://github.com/PRUNERS/FLiT.git && \
    git clone https://github.com/pystruct/pystruct.git

RUN git clone https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim && \
    vim +PluginInstall +qall

COPY Dockerfile Dockerfile-dev

CMD ["/bin/bash"]
