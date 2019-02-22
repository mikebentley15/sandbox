FROM ubuntu:18.04
MAINTAINER mikebentley15@gmail.com

ENV DEBIAN_FRONTEND=noninteractive

# install common dependencies
RUN apt-get -y update && \
    apt-get -y upgrade && \
    apt-get -y install \
      build-essential \
      cmake \
      doxygen \
      git \
      libboost-filesystem-dev \
      libboost-program-options-dev \
      libboost-serialization-dev \
      libboost-system-dev \
      libboost-test-dev \
      libeigen3-dev \
      libexpat1 \
      libflann-dev \
      libode-dev \
      libtinfo5 \
      libtriangle-dev \
      pkg-config \
      wget \
      && \
    rm -rf /var/lib/apt/lists/*

ENV CXX=g++

# Install python bindings dependencies
RUN apt-get -y update && \
    apt-get -y install \
      castxml \
      libboost-numpy-dev \
      libboost-python-dev \
      python3-dev \
      python3-numpy \
      python3-pip \
      && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install -vU \
      pygccxml \
      pyplusplus

# Install app dependencies
RUN apt-get -y update && \
    apt-get -y install \
      freeglut3-dev \
      libassimp-dev \
      libccd-dev \
      libfcl-dev \
      python3-celery \
      python3-flask \
      python3-opengl \
      python3-pyqt5.qtopengl \
      && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install -vU \
      PyOpenGL-accelerate

# Download ompl
RUN mkdir -p /opt && \
    cd /opt && \
    git clone https://github.com/ompl/omplapp.git && \
    cd /opt/omplapp && \
    git checkout e2aa31 && \
    git clone https://github.com/ompl/ompl.git && \
    cd /opt/omplapp/ompl && \
    git checkout adc802

RUN apt-get -y update && \
    apt-get -y install \
      lsb-release \
      sudo \
      && \
    rm -rf /var/lib/apt/lists/*

# Create ompl user with no password
RUN useradd --create-home --shell /bin/bash --gid sudo ompl && \
    passwd -d ompl

# Build and install ompl
RUN wget http://ompl.kavrakilab.org/install-ompl-ubuntu.sh && \
    bash -x install-ompl-ubuntu.sh --app

USER ompl
WORKDIR /home/ompl

COPY Dockerfile Dockerfile.ompl
