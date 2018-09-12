#!/bin/bash

VERSION="$1"
REMOTE="https://bigsearcher.com/mirrors/gcc/releases/gcc-$VERSION/gcc-$VERSION.tar.gz"
GCC_BASE="$HOME/gcc"
SRC="${GCC_BASE}/src/gcc-${VERSION}"
BUILD="${GCC_BASE}/build/gcc-${VERSION}-build"
INSTALL="$HOME/install/gcc-${VERSION}"

export CFLAGS="-O2"
export CXXFLAGS="-O2"
export MAKEFLAGS="-j20"

cd $HOME

mkdir -p "$SRC"
mkdir -p "$BUILD"
mkdir -p "$INSTALL"

cd "$(dirname "$SRC")"
curl "$REMOTE" | tar -xz
cd "$BUILD"

"$SRC"/configure \
  --prefix="$INSTALL" \
  --disable-multilib
make
make install

cd "$HOME"
rm -rf "${GCC_BASE}"
