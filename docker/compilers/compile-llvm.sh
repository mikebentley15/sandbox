#!/bin/bash

set -e
set -u
set -x

VERSION="$1"
REMOTE="http://releases.llvm.org/${VERSION}"
LLVM_BASE="/tmp/llvm"
SRC="${LLVM_BASE}/src"
LLVM_SRC="${SRC}/llvm-${VERSION}.src"
BUILD="${LLVM_BASE}/build/llvm-${VERSION}-build"
INSTALL="$HOME/install/llvm-${VERSION}"

clean_up () {
    rm -rf "${LLVM_BASE}"
}
trap clean_up EXIT

rm -rf "$SRC" "$BUILD" "$INSTALL"
mkdir -p "$SRC" "$BUILD" "$INSTALL"

cd "$SRC"
curl "${REMOTE}/llvm-${VERSION}.src.tar.xz"        | tar -x --xz
curl "${REMOTE}/cfe-${VERSION}.src.tar.xz"         | tar -x --xz
curl "${REMOTE}/compiler-rt-${VERSION}.src.tar.xz" | tar -x --xz
curl "${REMOTE}/polly-${VERSION}.src.tar.xz"       | tar -x --xz
curl "${REMOTE}/libcxx-${VERSION}.src.tar.xz"      | tar -x --xz
mv "cfe-${VERSION}.src" "$LLVM_SRC/tools/clang"
mv "compiler-rt-${VERSION}.src" "$LLVM_SRC/projects/compiler-rt"
mv "polly-${VERSION}.src" "$LLVM_SRC/tools/polly"
mv "libcxx-${VERSION}.src" "$LLVM_SRC/projects/libcxx"

# Configure, Build, and Install
cd "$BUILD"
cmake "$LLVM_SRC" \
  -DCMAKE_INSTALL_PREFIX="$INSTALL" \
  -DLLVM_TARGETS_TO_BUILD="host" \
  -DCMAKE_BUILD_TYPE=Release
make
make install
