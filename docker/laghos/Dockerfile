FROM nvidia/cuda:9.1-devel-ubuntu16.04

# setup dependencies
RUN apt-get update && \
    apt-get install -y \
      cmake \
      cuda-samples-9-1 \
      g++ \
      git \
      make \
      mpi-default-dev \
      sudo \
      tar \
      wget \
      && \
    rm -rf /var/lib/apt/lists/*

# add a separate non-root user with sudo permissions
# username: labuser
# password: labpass
RUN useradd --create-home --shell /bin/bash --gid sudo labuser && \
    echo 'labuser:labpass' | chpasswd

USER labuser
WORKDIR /home/labuser

# Download all files
RUN wget https://computation.llnl.gov/projects/hypre-scalable-linear-solvers-multigrid-methods/download/hypre-2.11.2.tar.gz && \
    tar -xf hypre-2.11.2.tar.gz && \
    rm hypre-2.11.2.tar.gz && \
    wget http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/OLD/metis-4.0.3.tar.gz && \
    tar -xf metis-4.0.3.tar.gz && \
    rm metis-4.0.3.tar.gz && \
    git clone https://github.com/mfem/mfem.git && \
    git clone --recursive https://github.com/llnl/raja.git && \
    git clone https://github.com/CEED/Laghos.git

# Build hypre
RUN ln -s hypre-2.11.2 hypre-2.10.0b && \
    cd hypre-2.11.2/src && \
    ./configure --disable-fortran && \
    make -j20

# Build metis
RUN make -j20 --directory metis-4.0.3 && \
    ln -s metis-4.0.3 metis-4.0

# Build mfem
RUN cd mfem && \
    git checkout v3.4 && \
    make parallel -j20

# Build and install Raja
RUN mkdir raja/build && \
    cd raja/build && \
    cmake -DENABLE_CUDA=ON ../ && \
    make -j20 && \
    make install DESTDIR=/home/labuser/raja_install

# Build Laghos
ENV CUDA_HOME /usr/local/cuda
ENV CUDA_DIR  /usr/local/cuda
ENV MFEM_DIR  /home/labuser/mfem
ENV RAJA_DIR  /home/labuser/raja_install/usr/local
ENV MPI_HOME  /usr/lib/openmpi
RUN cd Laghos && \
    git checkout raja-dev && \
    make raja

COPY Dockerfile ./

CMD ["mpirun", "-np", "1", \
     "Laghos/laghos", \
       "--max-steps", "1", \
       "--mesh", "Laghos/data/square01_quad.mesh", \
       "--refine-serial", "7", \
       "--refine-parallel", "0", \
       "--order-thermo", "2", \
       "--order-kinematic", "3", \
       "--problem", "1"]
