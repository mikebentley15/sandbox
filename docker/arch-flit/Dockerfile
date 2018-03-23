FROM base/archlinux:latest
MAINTAINER mikebentley15@gmail.com

# Install dependencies
RUN pacman -Syy --noconfirm && \
    pacman -Syyu --noconfirm && \
    pacman-db-upgrade && \
    pacman -Sy --noconfirm \
      clang \
      gcc \
      git \
      make \
      python \
      python-pip \
      sudo \
      && \
    pacman -Scc --noconfirm && \
    pip install toml

# Install FLiT system-wide
RUN git clone https://github.com/PRUNERS/FLiT.git && \
    cd FLiT && \
    git checkout issue91-flit-bisect && \
    make -j10 && \
    make install && \
    cd .. && \
    rm -rf FLiT

# Create a local user and give him sudo access
RUN groupadd sudo && \
    echo "%sudo ALL=(ALL) ALL" > /etc/sudoers.d/sudo-group && \
    useradd --create-home --shell /bin/bash --gid sudo flituser && \
    echo 'flituser:flitpass' | chpasswd

USER flituser
WORKDIR /home/flituser

# Install a copy of FLiT into my home directory and compile it
RUN mkdir git && \
    cd git && \
    git clone https://github.com/PRUNERS/FLiT.git && \
    cd FLiT && \
    git checkout issue91-flit-bisect && \
    make -j40

# Download and compile MFEM into my home directory
RUN cd git && \
    git clone https://github.com/mikebentley15/mfem.git && \
    cd mfem && \
    git checkout flit-examples && \
    make config && \
    make -j40

# Append to the PATH variable for the intel compiler (veritually linked)
ENV PATH="${PATH}:/opt/intel/bin"

# Make a copy of the Dockerfile
COPY Dockerfile Dockerfile-arch-flit

CMD ["/bin/bash"]
