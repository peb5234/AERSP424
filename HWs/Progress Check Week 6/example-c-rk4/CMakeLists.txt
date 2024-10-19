cmake_minimum_required(VERSION 3.22) # Specify CMake version.
project(example) # A project name. This can be anything.
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED TRUE)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wpedantic -Wextra")

message(STATUS "CMakeLists.txt is at ${PROJECT_SOURCE_DIR}") # To print something.
option(DEBUG "Print 'debug' statements?" ON)
if(DEBUG)
    message(">> Will DEBUG print everything!")
    add_definitions(-DDEBUG=1)
else()
    message(">> DEBUGGING in silence!")
    add_definitions(-DDEBUG=0)
endif()
message(STATUS "OPTION: printing debug statements? " ${DEBUG})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)

file(GLOB all_SRCS
    "${PROJECT_SOURCE_DIR}/include/*.h"
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.c"
)

# Create a binary file. 
# The first argument is the name of the binary file.
# It can be anything. In this case, it is set to be the project name.
# The other arguments are source files.
# Header files are not needed in add_executable(), assuming they are properly included.
add_executable(${CMAKE_PROJECT_NAME} ${all_SRCS})
target_link_libraries(${PROJECT_NAME} m)

