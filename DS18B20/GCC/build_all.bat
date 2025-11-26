mkdir build-Debug
cd build-Debug
cmake -DCMAKE_BUILD_TYPE=Debug .. -G "MinGW Makefiles"
cmake --build .
cd ..
mkdir build-Release
cd build-Release
cmake -DCMAKE_BUILD_TYPE=Release .. -G "MinGW Makefiles"
cmake --build .
cd ..
mkdir build-RelWithDebInfo
cd build-RelWithDebInfo
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. -G "MinGW Makefiles"
cmake --build .
cd ..
mkdir build-MinSizeRel
cd build-MinSizeRel
cmake -DCMAKE_BUILD_TYPE=MinSizeRel .. -G "MinGW Makefiles"
cmake --build .
cd ..
