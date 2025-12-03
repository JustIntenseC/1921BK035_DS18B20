mkdir build-Debug
cd build-Debug
cmake -DCMAKE_BUILD_TYPE=Debug .. -G "MinGW Makefiles"
cmake --build .
cd ..
