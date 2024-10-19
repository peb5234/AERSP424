#!/bin/sh
# if on ICDS ROAR you will need to load modules to make this work
# module load anaconda
# module load cmake
# rem module load gcc/9.1.0

# if on Windows and using MSYS, might need a couple of tools
# https://code.visualstudio.com/docs/cpp/config-mingw
# pacman -S cmake
# pacman -S git
# pacman -S --needed base-devel mingw-w64-ucrt-x86_64-toolchain
# pacman -S mingw-w64-ucrt-x86_64-python-numpy
# pacman -S mingw-w64-ucrt-x86_64-blas
# pacman -S mingw-w64-ucrt-x86_64-openblas
# pacman -S mingw-w64-ucrt-x86_64-vtk

# if on MacOS, you will need to install cmake, python, numpy, xcode-tools

echo "Hello Friend! Let's try and build this application together :D"
if [[ $# -ne 1 ]]; then
    echo 'Too many/few arguments, expecting one' >&2
    echo "Run this as 'sh build.sh 0' or 'sh build.sh 1'"
    echo "  0 will run without debug statements"
    echo "  1 will run with debug statements"

    exit 1
fi

case $1 in
    1|0)  # Ok
        ;;
    *)
        # The wrong first argument.
        echo 'Expected "0", "1"' >&2
        exit 1
esac

# remove the build directory that has the current code in it
echo "deleting the BUILD directory"
rm -rf build

echo "make a new BUILD directory to start the compiling process"
mkdir -p build
cd build

echo "cmake engage!"
cmake .. -DDEBUG=$1

echo "convert this to an executable application -- let's go!!"
cmake --build .
cd ..
echo "declare success -- hooray!"

echo "running the executible with some default parameters"
echo "./build/example > results.txt"
./build/example -t 60 -d 0.01 > results.txt


if [ $1 -eq "0" ]; then
    echo "Running the python plotting routine as the data is ready to go!"
    python plot.py
else
    echo "Check the 'results.txt' for helpful debugging information"
fi