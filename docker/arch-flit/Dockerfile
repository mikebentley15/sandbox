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

# Create a local user and give him sudo access
RUN groupadd sudo && \
    echo "%sudo ALL=(ALL) ALL" > /etc/sudoers.d/sudo-group && \
    useradd --create-home --shell /bin/bash --gid sudo flituser && \
    echo 'flituser:flitpass' | chpasswd

USER flituser
WORKDIR /home/flituser

# Install a copy of FLiT into my home directory and compile it
RUN git clone --branch devel https://github.com/PRUNERS/FLiT.git git/FLiT && \
    make --directory git/FLiT -j40

# Install FLiT system-wide
USER root
RUN make --directory git/FLiT install
USER flituser

# Download and compile MFEM into my home directory
#RUN git clone --branch flit-examples https://github.com/mikebentley15/mfem.git git/mfem && \
#    make --directory git/mfem config && \
#    make --directory git/mfem -j40

# Download and setup corrupt-clang
# Download and setup LULESH
# Download and setup Laghos

# Append to the PATH variable for the intel compiler (virtually linked)
ENV PATH="${PATH}:/opt/intel/bin"

# Make a copy of the Dockerfile
COPY Dockerfile Dockerfile-arch-flit

CMD ["/bin/bash"]
