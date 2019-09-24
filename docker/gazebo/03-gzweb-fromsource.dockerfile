FROM mikebentley15/gazebo:11.0

# Install dependencies for gzweb
RUN apt-get update && \
    apt-get install -y \
      build-essential \
      cmake \
      imagemagick \
      libboost-all-dev \
      libgts-dev \
      libjansson-dev \
      libtinyxml-dev \
      mercurial \
      nodejs \
      nodejs-legacy \
      npm \
      pkg-config \
      psmisc \
      xvfb \
      && \
    rm -rf /var/lib/apt/lists/*

# install gzweb to do a webserver and a webclient
ENV GZWEB_WS /opt/gzweb
RUN hg clone https://bitbucket.org/osrf/gzweb $GZWEB_WS && \
    cd $GZWEB_WS && \
    hg up gzweb_1.4.0 && \
    . /usr/share/gazebo/setup.sh && \
    xvfb-run -s "-screen 0 1280x1024x24" npm run deploy --- -m -t -c

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

EXPOSE 8080
EXPOSE 7681

# add a separate non-root user with sudo permissions
# username: llama
# password: llamapass
#RUN useradd --create-home --shell /bin/bash --gid sudo llama && \
#    echo 'llama:llamapass' | chpasswd
#
## make following commands run as this new llama user
#USER llama
#WORKDIR /home/llama

# Copy the Dockerfile used to generate the image
COPY 03-gzweb-fromsource.dockerfile /gzweb.dockerfile
COPY files/start.sh /start.sh

CMD ["/start.sh"]

