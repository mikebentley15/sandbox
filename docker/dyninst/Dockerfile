FROM ubuntu:16.04
MAINTAINER mikebentley15@gmail.com

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
      curl \
      g++ \
      gcc \
      git \
      libboost-date-time-dev \
      libboost-dev \
      libboost-system-dev \
      libboost-thread-dev \
      libdwarf-dev \
      libelf-dev \
      libiberty-dev \
      make \
      && \
    rm -rf /var/lib/apt/lists/*

# Build libdwarf.so since libdwarf-dev only comes with libdwarf.a
RUN curl http://pkgs.fedoraproject.org/repo/pkgs/libdwarf/libdwarf-20130207.tar.gz/64b42692e947d5180e162e46c689dfbf/libdwarf-20130207.tar.gz \
      | tar -xzv && \
    cd dwarf-20130207 && \
    ./configure --enable-shared && \
    make && \
    cp libdwarf/libdwarf.so /usr/lib/ && \
    cd .. && \
    rm -rf dwarf-20130207

# Build dyninst
RUN apt-get update && \
    apt-get install -y cmake && \
    git clone https://github.com/dyninst/dyninst.git && \
    mkdir -p dyninst/build && \
    cd dyninst/build && \
    cmake .. && \
    make && \
    make install && \
    cd ../.. && \
    rm -rf dyninst && \
    apt-get remove -y cmake && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*

# Add dependencies for toolchain-origin
RUN apt-get update && \
    apt-get install -y libgoogle-perftools-dev && \
    rm -rf /var/lib/apt/lists/*

# Build toolchain-origin
RUN git clone https://github.com/dyninst/toolchain-origin.git && \
    cd toolchain-origin && \
    (cd liblbfgs-1.10 && ./configure --prefix=/usr/local) && \
    make -C liblbfgs-1.10 && \
    make install -C liblbfgs-1.10 && \
    (cd crfsuite-0.12 && ./configure --prefix=/usr/local --with-liblbfgs=/usr/local) && \
    make -C crfsuite-0.12 && \
    make install -C crfsuite-0.12 && \
    sed -i -e 's/\s*-D_GLIBCXX_USE_CXX11_ABI=0\s*//g' function-features/Makefile && \
    make -C function-features \
      DYNINST_ROOT=/usr/local \
      GPERF=/usr \
      ELF_LIB=/usr/lib/x86_64-linux-gnu \
      && \
    cp function-features/extractFeat /usr/local/bin/ && \
    sed -i -e 's,../install/bin/extractFeat,/usr/local/bin/extractFeat,g' script/FeatGen.py && \
    mkdir -p /usr/local/script && \
    cp script/* /usr/local/script && \
    cd .. && \
    rm -rf toolchain-origin

# Add ability to sudo
RUN apt-get update && \
    apt-get install -y sudo && \
    rm -rf /var/lib/apt/lists/*

# Add a separate non-root user with sudo permissions
RUN useradd --create-home --shell /bin/bash --gid sudo dynuser && \
    echo 'dynuser:dynpass' | chpasswd

USER dynuser
WORKDIR /home/dynuser
COPY Dockerfile .

CMD ["/bin/bash"]
