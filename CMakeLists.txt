# run this, on windows or linux, as:
#make a directory build/ with `mkdir build`
#mkdir build/
#cd build
#cmake ../
#cmake --build .


cmake_minimum_required(VERSION 3.10)

# set the project name
project(Heuristic)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add the executable
add_executable(heuristic main.cpp Heuristic.cpp ProbabilityEstimator.cpp)
