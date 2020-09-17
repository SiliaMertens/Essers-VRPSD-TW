#!/bin/bash
if [ ! -d ${PWD}/build ]
then
    echo "Creating build"
    mkdir build/
elif [ $1 ] && [ $1 -eq 1 ]
then
    echo "Removing current build"
    rm -rf build/
    mkdir build/
else
    echo "Re-building"
fi
cd build
cmake ../
cmake --build .
cd ..
./build/heuristic
