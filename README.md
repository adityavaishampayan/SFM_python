[![Project Status: WIP – Initial development is in progress, but there has not yet been a stable, usable release suitable for the public.](https://www.repostatus.org/badges/latest/wip.svg)](https://www.repostatus.org/#wip)
[![Build Status](https://travis-ci.org/adityavaishampayan/SFM_python.svg?branch=master)](https://github.com/adityavaishampayan/SFM_python)
[![Coverage Status](https://coveralls.io/repos/github/adityavaishampayan/SFM_cpp/badge.svg?branch=master)](https://coveralls.io/github/adityavaishampayan/SFM_cpp?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


## Introduction

In this project we reconstruct a 3D scene and simultaneously obtain the camera poses of a monocular camera w.r.t. the given scene. This procedure is known as Structure from Motion (SfM). As the name suggests, you are creating the entire rigid structure from a set of images with different view points (or equivalently a camera in motion).

A few years ago, Agarwal et. al published Building Rome in a Day in which they reconstructed the entire city just by using a large collection of photos from the Internet.

There are a few steps that collectively form SfM:

  - Feature Matching and Outlier rejection using RANSAC
  - Estimating Fundamental Matrix
  - Estimating Essential Matrix from Fundamental Matrix
  - Estimate Camera Pose from Essential Matrix
  - Check for Cheirality Condition using Triangulation
  - Perspective-n-Point
     - Linear Camera Pose Estimation
     - PnP RANSAC
     - Nonlinear PnP
  - Bundle Adjustment
     - Visibility Matrix

## Authors
 - Aditya Vaishampayan
 - Amrish Baskaran

## License
License file can be found [here](https://github.com/adityavaishampayan/SFM_cpp/blob/master/LICENSE)

```
MIT License

Copyright (c) 2018 Aditya Vaishampayan, Amrish Bhaskaran

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Development Process
This module will be developed using the Solo Iterative Process(SIP), Test Driven Development and agile development in a 3 week sprint method.
The spreadsheet for the Product log, iteration backlog, work log and sprint details can be found in this link-[Agile Development Spreadsheet](https://docs.google.com/spreadsheets/d/1gTq1l9RCe6zuMW0L57cpaaCt8EEMOVuNtgeplXd2HEI/edit?usp=sharing)

Notes from the sprint review sessions can be found in the link-[Sprint review Doc](https://docs.google.com/document/d/1T2uCC2Ef_-l1TtSUzq2ocHxJed9Jsc1g0zjSTBZt-sQ/edit?usp=sharing)


## Structure
``` text
.
├── Dataset
│   ├── Data
│   │   ├── 1.jpg
│   │   ├── 2.jpg
│   │   ├── 3.jpg
│   │   ├── 4.jpg
│   │   ├── 5.jpg
│   │   ├── 6.jpg
│   │   ├── calibration.txt
│   │   ├── matching1.txt
│   │   ├── matching2.txt
│   │   ├── matching3.txt
│   │   ├── matching4.txt
│   │   └── matching5.txt
│   └── Data.zip
├── LICENSE
├── README.md
├── requirements
│   ├── requirements.txt
│   └── unittests.txt
├── scripts
│   ├── BuildVisibilityMatrix.py
│   ├── BundleAdjustment.py
│   ├── configure.py
│   ├── DisambiguateCameraPose.py
│   ├── EssentialMatrixFromFundamentalMatrix.py
│   ├── EstimateFundamentalMatrix.py
│   ├── ExtractCameraPose.py
│   ├── GetInliersRANSAC.py
│   ├── LinearTriangulation.py
│   ├── NonlinearPnP.py
│   ├── NonlinearTriangulation.py
│   ├── PnPRANSAC.py
│   └── Wrapper.py
├── setup.cfg
├── setup.py
├── tests
│   └── __init__.py
├── UML_diagrams
│   ├── activity_diagram.png
│   └── UML_ClassDiagram.pdf
└── Wrapper.py

```

Sources go in [src/](src/), header files in [include/](include/), main programs in [app/](app), and
tests go in [tests/](tests/) (compiled to `unit_tests` by default).

If you add a new executable, say `app/hello.cpp`, you only need to add the following two lines to [CMakeLists.txt](CMakeLists.txt):

``` cmake
add_executable(main app/main.cpp)   # Name of exec. and location of file.
target_link_libraries(main PRIVATE ${LIBRARY_NAME})  # Link the executable to lib built from src/*.cpp (if it uses it).
```

You can find the example source code that builds the `main` executable in [app/main.cpp](app/main.cpp) under the `Build` section in [CMakeLists.txt](CMakeLists.txt).
If the executable you made does not use the library in [src/](src), then only the first line is needed.

## Dependencies
Simple starter C++ project with:

- OpenCV - (OpenCV uses a BSD license and hence can be used for production with modification to the code.)
- googletest [Gtest](http://wiki.ros.org/gtest)
- Travis CI [Documentation](https://docs.travis-ci.com/user/for-beginners/)
- Coveralls [Documentation](https://docs.coveralls.io/about-coveralls)

## OpenCV installation
```
Update packages
sudo apt-get update
sudo apt-get upgrade
```
We will install required dependencies
```
sudo apt-get install build-essential checkinstall cmake pkg-config yasm
sudo apt-get install git gfortran
sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
 ```
If you are using Ubuntu 14.04
```
sudo apt-get install libtiff4-dev
```
If you are using Ubuntu 16.04
```
sudo apt-get install libtiff5-dev
```

```
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get install libxine2-dev libv4l-dev
sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install qt5-default libgtk2.0-dev libtbb-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install libvorbis-dev libxvidcore-dev
sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install x264 v4l-utils
 ```
Optional dependencies
```
sudo apt-get install libprotobuf-dev protobuf-compiler
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
```
Clone OpenCV and OpenCV_contrib
```
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.3.1
cd ..

git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.3.1
cd ..
```
Make build directory
```
cd opencv
mkdir build
cd build
```
Run Cmake
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
```
Find out number of CPU cores in your machine
```
nproc

# substitute 4 by output of nproc
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```
For installation related issues.

A complete OpenCV installation guide in Ubuntu can be found [here](http://www.codebind.com/cpp-tutorial/install-opencv-ubuntu-cpp/).


## Running tests-
Go to Catkin Workspace
```
cd ~/catkin_ws/
catkin_make run_tests intelli_bot_test
```

## Building

Build by making a build directory (i.e. `build/`), run `cmake` in that dir, and then use `make` to build the desired target.

Example:

``` bash
> mkdir build && cd build
> cmake .. -DCMAKE_BUILD_TYPE=[Debug | Coverage | Release]
> make
> ./main
> make test      # Makes and runs the tests.
> make coverage  # Generate a coverage report.
> make doc       # Generate html documentation.
```


## Setup
When starting a new project, you probably don't want the history of this repository. To start fresh you can use
the [setup script](setup.sh) as follows:
``` bash
> git clone https://github.com/bsamseth/cpp-project  # Or use ssh-link if you like.
> cd cpp-project
> sh setup.sh
```
The result is a fresh Git repository with one commit adding all files from the boiler plate.

## Doxygen Documentation

The doxygen documentation can be generated manually using the following commands
-

Installation
```
sudo apt-get install doxygen
sudo apt-get install doxygen-gui
```
Open Doxywizard and follow the instructions to generate the required files
```
doxywizard
```
Or one can also follow the steps given below:

Doxygen Documentation generation steps:
```
cd <path to repository>
mkdir Doxygen
cd Doxygen
doxygen -g <config_file_name>
```

Open configuration file and update the following:

```
PROJECT_NAME = 'your project name'
INPUT = ../LaneDetector ../include ../test
```

Run and generate the documents by running the next command:
```
doxygen <config_file_name>
```

## Plugins

- Google C++ Sytle

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter.
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml
