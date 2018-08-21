#!/bin/sh
set -e

# Add required repos

# lsb_release and wget are required to install the correct repositories, so we install it beforehand
apt-get update
apt-get install -y lsb-release wget gnupg

# Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# OpenModelica
for deb in deb deb-src; do echo "$deb http://build.openmodelica.org/apt `lsb_release -cs` nightly"; done | tee /etc/apt/sources.list.d/openmodelica.list
wget -q http://build.openmodelica.org/apt/openmodelica.asc -O- | apt-key add -

# Update packages
apt-get update

# noninteractive tzdata ( https://stackoverflow.com/questions/44331836/apt-get-install-tzdata-noninteractive )
export DEBIAN_FRONTEND=noninteractive

# CI specific packages
apt-get install -y clang git

# Dependencies
apt-get install -y libgazebo9-dev openmodelica

# Script
cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake -DBUILD_TESTING:BOOL=ON -DUSES_SYSTEM_FMILIBRARY:BOOL=OFF -G"${TRAVIS_CMAKE_GENERATOR}" ..
cmake --build . --config $TRAVIS_BUILD_TYPE
ctest --output-on-failure --build-config ${TRAVIS_BUILD_TYPE}
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install
