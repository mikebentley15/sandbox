FROM ubuntu:16.04
# TODO: figure out the problems from compiling boost from source to be able to
# TODO- use pagmo with dart
# TODO: ask Bala about how he handles this...
#FROM mikebentley15/pagmo2:2.11.3

# TODO: with installing boost from source, we will need to compile the
# TODO- following packages from source as well
# - libfcl-dev
# - liburdfdom-dev

# Install dart dependencies
RUN apt-get update && \
    apt-get install -y \
      build-essential \
      cmake \
      coinor-libipopt-dev \
      freeglut3-dev \
      git \
      graphviz-dev \
      libassimp-dev \
      libbullet-dev \
      libboost-regex-dev \
      libboost-system-dev \
      libccd-dev \
      libeigen3-dev \
      libfcl-dev \
      libflann-dev \
      libnlopt-dev \
      liboctomap-dev \
      libode-dev \
      libopenal-dev \
      libopenscenegraph-dev \
      libtinyxml2-dev \
      liburdfdom-dev \
      libxi-dev \
      libxmu-dev \
      make \
      ninja-build \
      pkg-config \
      python3 \
      python3-pip \
      wget \
      && \
    rm -rf /var/lib/apt/lists/*

# Install pybind
RUN git clone https://github.com/pybind/pybind11 -b 'v2.2.4' \
      --single-branch --depth 1 \
      /opt/pybind11 && \
    mkdir /opt/pybind11/build && \
    cd /opt/pybind11/build && \
    cmake .. -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DPYBIND11_TEST=OFF \
      -DCMAKE_INSTALL_PREFIX=/usr/ \
      && \
    ninja && \
    ninja install && \
    cd / && \
    rm -rf /opt/pybind

# Install dart and then dartpy (into python3)
# TODO: use -DHAVE_pagmo to build Dart with pagmo
RUN git clone https://github.com/dartsim/dart.git -b release-6.9 /opt/dart && \
    cd /opt/dart && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \
      && \
    ninja && \
    ninja install && \
    cmake .. -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DDART_BUILD_DARTPY=ON \
      && \
    ninja && \
    ninja install && \
    cd / && \
    rm -rf dart

# TODO: does dart use graphics accelerators?  make a version based on the
# TODO- nvidia containers?

# Copy the Dockerfile used to generate the image
COPY 02-dart-fromsource.dockerfile /dart.dockerfile
