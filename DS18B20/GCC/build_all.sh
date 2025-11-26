#!/bin/sh
# Debug
mkdir build-Debug && cd build-Debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
cd ..
# Release
mkdir build-Release && cd build-Release
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
cd ..
# RelWithDebInfo
mkdir build-RelWithDebInfo && cd build-RelWithDebInfo
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
cmake --build .
cd ..
# MinSizeRel
mkdir build-MinSizeRel && cd build-MinSizeRel
cmake -DCMAKE_BUILD_TYPE=MinSizeRel ..
cmake --build .
cd ..
