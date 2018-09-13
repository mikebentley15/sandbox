#!/bin/bash

set -e
set -u
set -x

VERSION="$1"
REMOTE="https://bigsearcher.com/mirrors/gcc/releases/gcc-$VERSION/gcc-$VERSION.tar.gz"
GCC_BASE="/tmp/gcc"
SRC="${GCC_BASE}/src"
GCC_SRC="${SRC}/gcc-${VERSION}"
BUILD="${GCC_BASE}/build/gcc-${VERSION}-build"
INSTALL="$HOME/install/gcc-${VERSION}"

export CFLAGS="-O2"
export CXXFLAGS="-O2"

clean_up() {
  rm -rf "${GCC_BASE}"
}
trap clean_up EXIT

rm -rf "$SRC" "$BUILD" "$INSTALL"
mkdir -p "$SRC" "$BUILD" "$INSTALL"

cd "$SRC"
curl "$REMOTE" | tar -x --gzip
cd "$BUILD"

"$GCC_SRC"/configure \
  --prefix="$INSTALL" \
  --disable-multilib \
  --enable-languages=c,c++
make
make install
