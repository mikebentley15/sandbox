FROM ubuntu:14.04
MAINTAINER mikebentley15@gmail.com

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
      bash \
      curl \
      g++-4.4 \
      gzip \
      libgmp-dev \
      libisl-dev \
      libmpc-dev \
      libmpfr-dev \
      make \
      tar \
      zip \
      && \
    rm -rf /var/lib/apt/lists/*

# Create a local user and give her sudo access
RUN echo "%sudo ALL=(ALL) ALL" > /etc/sudoers.d/sudo-group && \
    useradd --create-home --shell /bin/bash --gid sudo user && \
    echo 'user:pass' | chpasswd

USER user
WORKDIR /home/user
COPY compile-gcc.sh compile-gcc.sh
COPY compile-llvm.sh compile-llvm.sh
ENV CC=gcc-4.4
ENV CXX=g++-4.4

RUN MAKEFLAGS="-j20" bash compile-gcc.sh 4.8.5
RUN MAKEFLAGS="-j20" bash compile-gcc.sh 5.5.0
RUN MAKEFLAGS="-j20" bash compile-gcc.sh 6.4.0
RUN MAKEFLAGS="-j20" bash compile-gcc.sh 7.2.0
RUN MAKEFLAGS="-j20" bash compile-gcc.sh 7.3.0
RUN MAKEFLAGS="-j20" bash compile-gcc.sh 8.1.0
RUN MAKEFLAGS="-j20" bash compile-gcc.sh 8.2.0
RUN MAKEFLAGS="-j20" bash compile-llvm.sh 3.6.1
RUN MAKEFLAGS="-j20" bash compile-llvm.sh 3.7.1
RUN MAKEFLAGS="-j20" bash compile-llvm.sh 3.8.1
RUN MAKEFLAGS="-j20" bash compile-llvm.sh 3.9.1
RUN MAKEFLAGS="-j20" bash compile-llvm.sh 4.0.1
RUN MAKEFLAGS="-j20" bash compile-llvm.sh 5.0.2
RUN MAKEFLAGS="-j20" bash compile-llvm.sh 6.0.1

# Make a copy of the Dockerfile
COPY Dockerfile Dockerfile-compilers

CMD ["/bin/bash"]
