#!/bin/bash
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=$current_directory

mkdir -p ${project_root_dir}/third_party
# compile eigen
cd ${project_root_dir}/third_party
if [ ! -d "eigen" ]; then
  git clone -b 3.3 --depth 1 https://gitlab.com/libeigen/eigen.git 

  cd ${project_root_dir}/third_party/eigen
  rm -rf build install
  mkdir -p build && mkdir -p install && cd build
  cmake -DCMAKE_INSTALL_PREFIX=${project_root_dir}/third_party/eigen/install -DCMAKE_BUILD_TYPE=Release ..
  cmake --build . -j 4
  cmake --install .
fi

# compile opencv
cd ${project_root_dir}/third_party
if [ ! -d "opencv" ]; then
  git clone --depth 1 https://github.com/opencv/opencv.git
  cd opencv
  git fetch --tags --depth 1
  git checkout -b 4.9.0  tags/4.9.0

  cd ${project_root_dir}/third_party/opencv
  if [ ! -d "install" ]; then
    mkdir -p build && mkdir -p install && cd build
    export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${project_root_dir}/third_party/eigen/install
    cmake -DCMAKE_INSTALL_PREFIX=${project_root_dir}/third_party/opencv/install -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DBUILD_LIST=core,highgui ..
    cmake --build . -j 4
    cmake --install .
  fi
fi

cd ${project_root_dir}
rm -rf build
mkdir -p build && cd build
cmake ..
cmake --build .