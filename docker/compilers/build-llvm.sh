#!/bin/bash


# Locations
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BASE="$( cd "${SCRIPT_DIR}" && cd .. && pwd )"
TMP=/tmp/buildtemp




# Optionally get the install version
if [ $# -eq 0 ]
then
  VERSION=6.0.1
else
    VERSION="$1"
    case "${VERSION}" in
	6.0.1) ;;
	6.0.0) ;;
	5.0.2) ;;
	5.0.1) ;;
	5.0.0) ;;
	4.0.1) ;;
	4.0.0) ;;
	3.9.1) ;;
	3.9.0) ;;
	3.8.1) ;;
	3.8.0) ;;
	3.7.1) ;;
	3.7.0) ;;
	*)
	    echo "Unsupported llvm version '${VERSION}'"
	    exit 0
	    ;;
    esac
    echo "Building specified version '${VERSION}'"
fi
LLVM_VERSIONS="${BASE}/llvm-versions"
LLVM_TARS="${BASE}/llvm-versions/tars"
LLVM_INSTALL="${BASE}/llvm-versions/llvm-${VERSION}.install"
LLVM_SOURCE="${TMP}/llvm-${VERSION}.src"
LLVM_BUILD="${TMP}/llvm-${VERSION}.build"


# Test for reinstall
if [ -f "${BASE}/activate-llvm-${VERSION}" ]
then
  echo "LLVM ${VERSION} was already installed"
  echo "To force a reinstall remove ${BASE}/activate-llvm-${VERSION}"
  exit 0
fi


# Exit on error
set -e
clean_up () {
    ARG=$?
    echo "Cleaning up"
    rm -rf "${TMP}"
    exit "${ARG}"
}
trap clean_up EXIT


# Make sure everything was cleaned from prior runs
rm -rf "${LLVM_INSTALL}"
rm -rf "${TMP}"


# Dowload tars for LLVM
llvm_download_tar () {
    BASE_URL="http://releases.llvm.org/${VERSION}"
    FILE="$1"
    if [ ! -f "${LLVM_TARS}/${FILE}" ]
    then
      mkdir -p "${LLVM_TARS}"
      pushd "${LLVM_TARS}"
      wget "${BASE_URL}/${FILE}"
      popd
    fi
}

llvm_download_tar "llvm-${VERSION}.src.tar.xz"
llvm_download_tar "cfe-${VERSION}.src.tar.xz"
llvm_download_tar "compiler-rt-${VERSION}.src.tar.xz"
llvm_download_tar "polly-${VERSION}.src.tar.xz"


# Extract
mkdir "${TMP}"
cd "${TMP}"
tar xf "${LLVM_TARS}/llvm-${VERSION}.src.tar.xz"
tar xf "${LLVM_TARS}/cfe-${VERSION}.src.tar.xz"
tar xf "${LLVM_TARS}/compiler-rt-${VERSION}.src.tar.xz"
tar xf "${LLVM_TARS}/polly-${VERSION}.src.tar.xz"


# Move into place
mv "${TMP}/cfe-${VERSION}.src" "${LLVM_SOURCE}/tools/clang"
mv "${TMP}/compiler-rt-${VERSION}.src" "${LLVM_SOURCE}/projects/compiler-rt"
mv "${TMP}/polly-${VERSION}.src" "${LLVM_SOURCE}/tools/polly"


# Configure, Build, and Install
mkdir "${LLVM_BUILD}"
cd "${LLVM_BUILD}"
cmake "${LLVM_SOURCE}" \
      -DCMAKE_INSTALL_PREFIX="${LLVM_INSTALL}" \
      -DLLVM_TARGETS_TO_BUILD="host" \
      -DCMAKE_BUILD_TYPE=Release
make
make install


# Clean up
rm -rf "${TMP}"


# Add activation scripts
SRC_SCRIPT="${SCRIPT_DIR}/scripts/activate_llvm_VERSION"
DST_SCRIPT="${BASE}/activate-llvm-${VERSION}"
sed "s|%VERSION%|${VERSION}|g" "${SRC_SCRIPT}" > "${DST_SCRIPT}"
cp "${PRESAGE_SOURCE}/scripts/deactivate_llvm" "${BASE}/deactivate-llvm"


echo "To add or remove LLVM ${VERSION} from your path use:"
echo "'source ./activate-llvm-${VERSION}' and 'source ./deactivate-llvm'"
