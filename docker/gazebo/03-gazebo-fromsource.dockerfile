FROM mikebentley15/dart:6.9

# Setup Gazebo and ROS repositories
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu xenial main" \
      > /etc/apt/sources.list.d/ros-latest.list && \
    wget http://packages.ros.org/ros.key -O - | apt-key add -

# Install ros and gazebo dependencies
RUN apt-get update && \
    apt-get install -y \
      bc \
      build-essential \
      cmake \
      cppcheck \
      debhelper \
      freeglut3-dev \
      gnupg2 \
      libbullet-dev \
      libcurl4-openssl-dev \
      libfreeimage-dev \
      libgts-dev \
      libignition-common-dev \
      libignition-fuel-tools-dev \
      libignition-math4-dev \
      libignition-msgs-dev \
      libignition-transport4-dev \
      libltdl-dev \
      libogre-1.9-dev \
      libprotobuf-dev \
      libprotoc-dev \
      libqwt-qt5-dev \
      libsdformat6-dev \
      libsimbody-dev \
      libtar-dev \
      libtbb-dev \
      libtinyxml-dev \
      libtinyxml2-dev \
      libxml2-dev \
      locales \
      mesa-utils \
      net-tools \
      netcat-openbsd \
      pkg-config \
      protobuf-compiler \
      python \
      python-lxml \
      python-psutil \
      qtbase5-dev \
      ros-kinetic-desktop \
      ros-kinetic-perception \
      ros-kinetic-ros-control \
      ros-kinetic-ros-controllers \
      x11-utils \
      xsltproc \
      && \
    rm -rf /var/lib/apt/lists/*

# Install optional gazebo dependencies
# - ruby-ronn for man page support
RUN apt-get update && \
    apt-get install -y \
      mercurial \
      ruby-ronn \
      && \
    rm -rf /var/lib/apt/lists/*

# Install gazebo
# TODO: remove the build directory when stable
RUN hg clone https://bitbucket.org/osrf/gazebo \
      /opt/gazebo && \
    mkdir /opt/gazebo/build && \
    cd /opt/gazebo/build && \
    cmake .. -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_SCREEN_TESTS=OFF \
      -DCMAKE_INSTALL_PREFIX=/usr/ \
      && \
    ninja && \
    ninja install && \
    cd / && \
    rm -rf /opt/gazebo

# TODO: does dart or gazebo use graphics accelerators?  make a version based on
# TODO- the nvidia containers?
# TODO: is it better to launch gazebo from docker, or use gzserver from docker?
# TODO: how to integrate gazebo and ros with docker?

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

EXPOSE 11345

# Copy the Dockerfile used to generate the image
RUN ln -s /usr/share/gazebo/setup.sh /etc/profile.d/gazebo.sh
COPY 03-gazebo-fromsource.dockerfile /gazebo.dockerfile
COPY files/profile_entrypoint.sh /profile_entrypoint.sh

ENTRYPOINT ["/profile_entrypoint.sh"]
CMD ["gzserver", "--verbose"]

