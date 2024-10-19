---
title: Week 06 Participation
author: Xiangfu Li
email: xml5328@psu.edu
date: today
---
# Week 06 Participation

This project has been tested on MacOS with cmake. And it should work when use git console in Windows or terminal in Linux.

## Structure of the code

After you build the whole program, you will get several directories.

```
---- build <- where the build files and executables are stored
|
|
---- include <- header files
|
|
---- src <- main source files
|
|
---- third_party <- to include third_party dependencies and build/install the these dependencies
```

## How to build

Just run the bash file in the git console terminal in Windows or the default terminal in MacOS or Linux. Before running this script, make sure you have installed git, cmake and related build toolchains.

```
bash build.sh
```

In this shell script, we will download, compile and install the third_party dependencies (OpenCV and Eigen).

## How to run the final program


All the executables will be stored under the build folder.

Use the following command to run the executable.

```
./build/main
```

## Structure of the codes

In this project, I created a class named `Plane`. In this class, it will perform all the numerical integration and store the states to the corresponding vector. The only public method is `void fly(double dt);` which simulate the dynamics of the flight movement.

In the `main` file, I will try to invoke the `fly` method until reaches the time limit. Then I used the function `drawLineChart` to draw the line chart. For now only lines are shown in the plot, there is no any text inside. You can use `cv::putText` method to add the annotations or numbers inside the plot.
