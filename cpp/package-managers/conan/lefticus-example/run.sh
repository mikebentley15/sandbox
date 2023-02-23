#!/usr/bin/env bash

BUILDDIR=build

rm -rf "${BUILDDIR}"
mkdir -p "${BUILDDIR}"
source activate
pushd "${BUILDDIR}"
conan install ../conanfile.txt --build=missing --output-folder .
conan profile detect
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
ninja
popd

echo
"${BUILDDIR}/fmt_test"
echo
