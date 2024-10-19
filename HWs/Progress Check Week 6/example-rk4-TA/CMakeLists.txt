cmake_minimum_required(VERSION 3.2)

project(week6)

list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/third_party/eigen/install")
set(OpenCV_DIR "${PROJECT_SOURCE_DIR}/third_party/opencv/install")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED PATHS "${OpenCV_DIR}")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(main src/main.cpp src/plane.cpp)
target_link_libraries(main ${OpenCV_LIBS})



