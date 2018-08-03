#!/bin/bash
set -e

CURDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

if [ -z "$SOURCEDIR" ]; then
    SOURCEDIR="$CURDIR"
fi
if [ -z "$BUILD_TYPE" ]; then
    BUILD_TYPE=Release
fi
if [ ! "$BUILD_TYPE" = "Debug" ] && [ ! "$BUILD_TYPE" = "Release" ] && [ ! "$BUILD_TYPE" = "RelWithDebInfo" ]; then
  echo "Unknown BUILD_TYPE: $BUILD_TYPE, using Release"
  BUILD_TYPE=Release
fi
if [ -z "$BUILD_DIR" ]; then
  BUILD_DIR=$SOURCEDIR/build/$BUILD_TYPE
fi
if [ -z "$INSTALL_DIR" ]; then
  INSTALL_DIR=$SOURCEDIR/install/$BUILD_TYPE
fi
# cmake version 3 is called 'cmake3' on centos, but 'cmake' on ubuntu
if [ -e "/usr/bin/cmake3" ]; then 
  CMAKE=cmake3; 
else
  CMAKE=cmake
fi

rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR
$CMAKE $SOURCEDIR/src/ -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_BUILD_TYPE=$BUILD_TYPE
make -j 10 install
