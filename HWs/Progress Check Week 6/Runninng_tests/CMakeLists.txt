
cmake_minimum_required(VERSION 3.10)

# Project Name
project(Attempt_2)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include header file directory
include_directories(${PROJECT_SOURCE_DIR})

# Add the executable
add_executable(Attempt_2 Attempt_2.cpp)

cmake_minimum_required(VERSION 3.23)

# Set the project name
project(Week_6 VERSION 1.0 LANGUAGES CXX)

# Add the executable
add_executable(Week_6 Attempt_2.cpp)
