# fetch-it
## Autonomous Robot to collect tennis balls from the court 
[![Build Status](https://travis-ci.org/nalindas9/fetch-it.svg?branch=main)](https://travis-ci.org/github/nalindas9/fetch-it)
[![Coverage Status](https://coveralls.io/repos/github/nalindas9/fetch-it/badge.svg?branch=main)](https://coveralls.io/github/nalindas9/fetch-it?branch=main)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/nalindas9/fetch-it/blob/feature/nalindas9/initialize-repository/LICENSE)

## Overview
![Fetch-IT_REVISED_CLASS_DIAGRAM](https://github.com/nalindas9/fetch-it/assets/44141068/4b7379fc-5830-4648-9708-24319067da4b)

![FetchIT_ Quadchart_final](https://github.com/nalindas9/fetch-it/assets/44141068/57a943fb-f5d3-4505-bcef-ef068dde4dd8)

Robotics has long been an interesting part of the sports industry. Some of the examples are RoboCup, Basketball playing Robot CUE etc. Our group is committed to developing a robust autonomous collection robot for ACME’s tennis balls collection application. According to the studies, most of the time is spent on collecting the balls rather than the actual practice. Hence, we are planning to propose an autonomous robot to carry out the tennis ball collection activity inside a tennis court which will reduce the overall time spent on the collection activity. 

For the purpose of this project, we are only focusing on the ball identification and collection task. The robot will be deployed in a virtual tennis court environment using Gazebo and will roam around identifying and collecting the tennis balls. The identification task is addressed using a monocular camera mounted on the robot. We plan to leverage color detection (in the RGB or HSV space) to identify if the object is present in the frame from the incoming video stream. The idea is to utilize a simple obstacle avoidance algorithm similar to the Roomba Vacuum Cleaner.
The diagram depicting the overall framework process can be accessed from [here](https://drive.google.com/file/d/1x5PeSOjn5OzAIuLnxqx3R4edWLp_fohM/view?usp=sharing)

## Workflow Architecture

![Fetch-IT_UML_ACTIVITY_DIAGRAM](https://github.com/nalindas9/fetch-it/assets/44141068/441955a2-7cd3-4514-b633-5ca9c4baecec)

![YOLO_System](https://github.com/nalindas9/fetch-it/assets/44141068/b97f60bc-7e17-4506-949f-651289253ed9)

## Team Members
1. Nalin Das - nalindas9
2. Sukoon Sarin - sukoonsarin
3. Nidhi Bhojak - nbhojak07

## Presentation 
- The presentation slides can be viewed from [here](https://docs.google.com/presentation/d/1ziL8vnf1k-Nsx0coOIeDWfG2G3LviOGE_k53tatEt-w/edit?usp=sharing)
- Recorded Presentation Video can be accessed from [here](https://youtu.be/gi4kzVk1ybs)

## License 
- This project has been developed under the 3-Clause BSD License.
- Before cloning the repository, kindly go through the license [here](https://github.com/nbhojak07/fetch-it/blob/iteration-1/LICENSE)

## Technologies Used
- Operating System - Ubuntu 18.04
- Programming language - C++ 11/14
- Open Source Libraries - OpenCV (Apache License)
- Build System - CMake 
- ROS Version - Melodic
- Simulation Environment - Gazebo 
- Code Coverage - Coveralls
- Automated Unit Testing - Travic CI
- Version Control - Git & Github

## AIP and Sprint Documents
- To access the AIP sheets, click [here](https://docs.google.com/spreadsheets/d/1h1RDyUNMMq0FCVfPDFFzh-2Qu_SgeK83dBcB3OKtEOY/edit#gid=0)
- To access the Sprint Planning Notes, click [here](https://docs.google.com/document/d/13czpabipeWM1hxAIBa5MMN0HsTAbhcGeMKuGNPOWhuI/edit)

## Known Issues and Bugs
- For the ball identification task, we will need to explore available detection algorithms and choose the best option among them. 
- For the object collection task, we are figuring out a proper method that depicts the task. Like a vacuum cup or vanishing object on contact. 
- Multiple balls in the camera frame could be an issue with our current approach, since our approach takes into account one ball in the camera frame for collection. This can be mitigated by using LaserScanner from ROS to use distance to the nearest ball.

## Install Dependencies
#### Install Gazebo and Turtlebot Packages
- This Project is developed using ROS Melodic.
- Make sure that the ROS Melodic is properly installed on your System. Follow ROS Melodic installation instructions from [here](http://wiki.ros.org/melodic/Installation/Ubuntu)
- The Full-Desktop Version will install Gazebo as well. To install Gazebo separately follow instructions from [here](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- Ensure successful installation of Gazebo from the terminal by running the following command: 
```
gazebo
```
The above command will laumch an empty world of the Gazebo Simulator.
- Install necessary turtlebot packages by running the following command:
```
sudo apt install ros-melodic-turtlebot-gazebo ros-melodic-turtlebot-apps ros-melodic-turtlebot-rviz-launchers
```
#### Install OpenCV Dependencies 
```
sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install python3.5-dev python3-numpy libtbb2 libtbb-dev
sudo apt install libjpeg-dev libpng-dev libtiff5-dev libjasper-dev libdc1394-22-dev libeigen3-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common libtbb-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenexr-dev libgstreamer-plugins-base1.0-dev libavutil-dev libavfilter-dev libavresample-dev
```
#### Install OpenCV
```
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 3.3.0 
cd ..
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.3.0
cd ..
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=ON -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_QT=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D BUILD_EXAMPLES=ON ..
make -j$(nproc)
sudo make install
sudo /bin/bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig

```
## Build 
Switch to your src sub-directory of the ROS Workspace to clone this repository
```
<ROS Workspace>/src
```
Run the following commands to clone and build this project:
```
git clone --recursive https://github.com/nalindas9/fetch-it
cd ..
catkin_make
```
## Test 
Close and terminate everything including rosmaster. In a new terminal, switch to the ROS workspace and build the tests and type,
```
cd catkin_ws
source devel/setup.bash
catkin_make run_tests_fetch-it
```

## Run
We will use launch file to run the package. In a new terminal, enter:
```
cd catkin_ws
source devel/setup.bash
roslaunch fetch-it fetch-it.launch
```
## Accessing the UML Diagrams
- To access the UML Diagrams, go the UML Sub-Directory in the repository which contains folders for initial as well as the revised UML diagrams.

## Doxygen 
To install doxygen run the following command:
```
sudo apt-get install doxygen
```
Now from the cloned directory run:
```
doxygen Doxygen
```
Generated doxygen files are in html format and you can find them in ./docs folder with the following command,
```
firefox docs/html/index.html
```
