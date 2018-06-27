#!/bin/bash

set -x
set -e

SCRIPT_DIR="$(readlink -f "$(dirname "${BASH_SOURCE[0]}")")"

# Download the hypre and metis source files if they are not already here.

HYPRE_TAR=hypre-2.11.2.tar.gz
METIS_TAR=metis-4.0.3.tar.gz

HYPRE_URL=https://computation.llnl.gov/projects/hypre-scalable-linear-solvers-multigrid-methods/download/hypre-2.11.2.tar.gz
METIS_URL=http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/OLD/metis-4.0.3.tar.gz

if [ ! -f "$HYPRE_TAR" ]; then
  wget "$HYPRE_URL"
fi

if [ ! -f "$METIS_TAR" ]; then
  wget "$METIS_URL"
fi


#
# Build hypre 2.11.2 but make it look like 2.10.0b
#

HYPRE_DIR="$SCRIPT_DIR/$(basename "$HYPRE_TAR" .tar.gz)"
HYPRE_LINK="$SCRIPT_DIR/hypre-2.10.0b"

rm -rf "$HYPRE_DIR"
rm -rf "$HYPRE_LINK"
tar -xf "$HYPRE_TAR"

pushd "$HYPRE_DIR/src"
./configure --disable-fortran
make
popd

ln -s "$HYPRE_DIR" "$HYPRE_LINK"


#
# Build metis
#

METIS_DIR="$SCRIPT_DIR/$(basename "$METIS_TAR" .tar.gz)"
METIS_LINK="$SCRIPT_DIR/metis-4.0"

rm -rf "$METIS_DIR"
rm -rf "$METIS_LINK"
tar -xf "$METIS_TAR"

pushd "$METIS_DIR"
make
popd

ln -s "$METIS_DIR" "$METIS_LINK"


#
# Clone and build mfem laghos-v1.0
#

MFEM_DIR="$SCRIPT_DIR/mfem"

rm -rf "$MFEM_DIR"
git clone https://github.com/mfem/mfem.git "$MFEM_DIR"

pushd "$MFEM_DIR"
git checkout v3.4
make parallel
popd


#
# Clone, build, and install raja
#

RAJA_DIR="$SCRIPT_DIR/raja"
RAJA_INSTALL="$SCRIPT_DIR/raja_install"

git clone --recursive https://github.com/llnl/raja.git "$RAJA_DIR"

pushd "$RAJA_DIR"
mkdir build
cd build
cmake -DENABLE_CUDA=ON ../
make
make DESTDIR="$RAJA_INSTALL" install
popd


#
# Clone and build Laghos
#

LAGHOS_DIR="$SCRIPT_DIR/Laghos"

rm -rf "$LAGHOS_DIR"
git clone https://github.com/CEED/Laghos.git "$LAGHOS_DIR"

pushd "$LAGHOS_DIR"
git checkout raja-dev
export CUDA_DIR="$CUDA_HOME"
export MFEM_DIR
export RAJA_DIR="$RAJA_INSTALL/usr/local"
export MPI_HOME="$MPI_ROOT"
make raja
popd

