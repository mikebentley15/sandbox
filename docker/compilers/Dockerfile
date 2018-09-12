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
      && \
    rm -rf /var/lib/apt/lists/*

# Create a local user and give her sudo access
RUN groupadd sudo && \
    echo "%sudo ALL=(ALL) ALL" > /etc/sudoers.d/sudo-group && \
    useradd --create-home --shell /bin/bash --gid sudo user && \
    echo 'user:pass' | chpasswd

USER user
WORKDIR /home/user
COPY compile-gcc.sh compile-gcc.sh
ENV CC=gcc-4.4

RUN bash compile-gcc.sh 4.8.5
RUN bash compile-gcc.sh 5.5.0
RUN bash compile-gcc.sh 6.4.0
RUN bash compile-gcc.sh 7.2.0
RUN bash compile-gcc.sh 7.3.0
RUN bash compile-gcc.sh 8.1.0
RUN bash compile-gcc.sh 8.2.0

# Make a copy of the Dockerfile
COPY Dockerfile Dockerfile-compilers

CMD ["/bin/bash"]