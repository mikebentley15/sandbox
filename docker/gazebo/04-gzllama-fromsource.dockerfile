FROM mikebentley15/gzweb:11.0

# Install gazebo_ros_pkgs
##RUN git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git \
##      -b kinetic-devel \
##      /opt/gazebo_ros_pkgs && \
##    mkdir /opt/gazebo_ros_pkgs/build && \
##    cd /opt/gazebo_ros_pkgs/build && \
##    cmake .. -G Ninja \
##      -DCMAKE_BUILD_TYPE=Release \
##      && \
###    ninja && \
###    ninja install && \
##    cd .. && \
##    # rm -rf build
##    echo

# setup development tools and bash shell
RUN apt-get update && \
    apt-get install -y \
      bash-completion \
      vim \
      vim-gnome \
      tmux \
      screen \
      moreutils \
      python-pygments \
      mlocate
      && \
    rm -rf /var/lib/apt/lists/*

# add a separate non-root user with sudo permissions
# username: llama
# password: llamapass
#RUN useradd --create-home --shell /bin/bash --gid sudo llama && \
#    echo 'llama:llamapass' | chpasswd
#
## make following commands run as this new llama user
#USER llama
#WORKDIR /home/llama

COPY 04-gzllama-fromsource.dockerfile /gzllama.dockerfile
