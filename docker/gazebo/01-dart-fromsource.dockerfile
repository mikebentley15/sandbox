FROM ubuntu:16.04

# install base dependencies
RUN apt-get update && \
    apt-get install -y \
      git \
      lsb-release \
      sudo \
      wget \
      && \
    rm -rf /var/lib/apt/lists/*

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
      libboost-regex-dev \
      libboost-system-dev \
      libbullet-dev \
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
      && \
    rm -rf /var/lib/apt/lists/*

# Setup Gazebo and ROS repositories
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu xenial main" \
      > /etc/apt/sources.list.d/ros-latest.list && \
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

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
    cd .. && \
    rm -rf build

# Install dart and then dartpy (into python3)
# TODO: use -DHAVE_pagmo to build Dart with pagmo
# TODO: remove the build directory when stable
RUN git clone https://github.com/dartsim/dart.git -b release-6.9 /opt/dart && \
    cd /opt/dart && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/ \
      && \
    ninja && \
    ninja install && \
    cmake .. -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/ \
      -DDART_BUILD_DARTPY=ON \
      && \
    ninja && \
    ninja install && \
    cd .. && \
    #rm -rf build
    echo

# TODO: does dart use graphics accelerators?  make a version based on the
# TODO- nvidia containers?

# Copy the Dockerfile used to generate the image
COPY 01-dart-fromsource.dockerfile /dart.dockerfile
