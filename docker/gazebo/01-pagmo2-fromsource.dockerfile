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

# Create a user and group that can do sudo without a password
RUN groupadd sudo-nopw && \
    useradd --create-home --shell /bin/bash --gid sudo-nopw builder && \
    echo "%sudo-nopw ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudo-nopw-group

USER builder

# Install pagmo dependencies
RUN sudo apt-get update && \
    sudo apt-get install -y \
      build-essential \
      cmake \
      coinor-libipopt-dev \
      debhelper \
      libeigen3-dev \
      libnlopt-dev \
      libtbb-dev \
      make \
      ninja-build \
      python3 \
      python3-pip \
      && \
    sudo rm -rf /var/lib/apt/lists/* && \
    sudo pip3 install numpy cloudpickle

# TODO: compile boost into a deb package and install that
# Compile boost from source
RUN cd /tmp && \
    wget https://downloads.sourceforge.net/project/boost/boost/1.70.0/boost_1_70_0.tar.bz2 && \
    tar -xf boost_1_70_0.tar.bz2 && \
    cd /tmp/boost_1_70_0 && \
    git clone https://github.com/mikebentley15/boost-deb.git debian && \
    ./debian/rules build && \
    fakeroot ./debian/rules binary && \
    cd .. && \
    sudo dpkg -i libboost*.deb && \
    #./bootstrap.sh \
    #  --prefix=/usr \
    #  --with-python=$(which python3) \
    #  && \
    #./b2 -j $(nproc) && \
    #sudo ./b2 install -j $(nproc) && \
    cd / && \
    rm -rf /tmp/boost_1_70_0 && \
    rm /tmp/boost_1_70_0.tar.bz2 && \
    rm /tmp/libboost*.deb

COPY files/01-cmake-with-boost-1.70.patch /patches/01-pagmo2-cmake-with-boost-1.70.patch

# Install pagmo and pygmo
RUN git clone https://github.com/esa/pagmo2.git -b v2.11.3 /tmp/pagmo2 && \
    cd /tmp/pagmo2 && \
    git apply /patches/01-pagmo2-cmake-with-boost-1.70.patch && \
    mkdir build && \
    cd build && \
    cmake .. -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DBUILD_SHARED_LIBS=ON \
      -DPAGMO_BUILD_TESTS=ON \
      -DPAGMO_WITH_EIGEN3=ON \
      -DPAGMO_WITH_IPOPT=ON \
      -DPAGMO_WITH_NLOPT=ON \
      && \
    ninja && \
    ninja test && \
    sudo ninja install && \
    cmake .. -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DPAGMO_BUILD_PAGMO=OFF \
      -DPAGMO_BUILD_PYGMO=ON \
      && \
    ninja && \
    sudo ninja install && \
    cd / && \
    rm -rf /tmp/pagmo2

# Copy the Dockerfile used to generate the image
COPY 01-pagmo2-fromsource.dockerfile /docker/pagmo2.dockerfile
