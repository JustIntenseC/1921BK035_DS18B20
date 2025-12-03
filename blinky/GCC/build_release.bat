mkdir build-Release
cd build-Release
cmake -DCMAKE_BUILD_TYPE=Release .. -G "MinGW Makefiles"
cmake --build .
cd ..
