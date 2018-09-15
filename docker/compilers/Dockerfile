FROM ubuntu:14.04
MAINTAINER mikebentley15@gmail.com

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
      bash \
      cmake \
      curl \
      g++-4.7 \
      gzip \
      libgmp-dev \
      libisl-dev \
      libmpc-dev \
      libmpfr-dev \
      make \
      python \
      python3 \
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
COPY install_compiler.py install_compiler.py
ENV CC="gcc-4.7"
ENV CXX="g++-4.7"
ENV MAKEFLAGS="-j20"

RUN python3 install_compiler.py gcc 4.8.5
RUN python3 install_compiler.py gcc 5.5.0
RUN python3 install_compiler.py gcc 6.4.0
RUN python3 install_compiler.py gcc 7.2.0
RUN python3 install_compiler.py gcc 7.3.0
RUN python3 install_compiler.py gcc 8.1.0
RUN python3 install_compiler.py gcc 8.2.0

#RUN python3 install_compiler.py llvm 3.6.1
RUN python3 install_compiler.py llvm 3.7.1
#RUN python3 install_compiler.py llvm 3.8.1
#RUN python3 install_compiler.py llvm 3.9.1
#RUN python3 install_compiler.py llvm 4.0.1
#RUN python3 install_compiler.py llvm 5.0.2
#RUN python3 install_compiler.py llvm 6.0.1

# Make a copy of the Dockerfile
COPY Dockerfile Dockerfile-compilers

CMD ["/bin/bash"]
