# PGEAS - UFSC - ROSI CHALLENGE - XIV SBAI

![rosi_banner](https://raw.githubusercontent.com/filRocha/rosiChallenge-sbai2019/master/resources/banner2.png)

This repository contains the ROS package solution from PGEAS´ group for the **ROSI CHALLENGE** competition that occurs on the **XIV SBAI**, to be held in Ouro Preto (Brazil) in october 2019. One may find more info about SBAI in www.sbai2019.com.br.

# Package description

This repository is structured as a ROS package. The folders organization is as follows:

- `resources` - General support files. See installation steps.

- `launch` - Contains ROS launch files for maping and localization. 

- `src` - Eventual nodes for specific tasks. 

- `maps` - Contains 3d point cloud database and 2d gridmap.

- `rviz` - Contains rviz config file used in tests.


# Installation

The simulator was conceived using **Ubuntu 18.4.2**, **ROS Melodic**, and **V-REP 3.6.2 (rev.0)**. Another software versions might work, but they are not recommended nor officially supported for the competition. THis solution uses Rtabmap for real time appearence based map, with lbpointmatcher and opencv with contrib modules, which should be all built from source.

## Installation advices

By covention on this installation steps, all boxes starting with a `$` mark means that you should run the command on the terminal. 

## Install eigen
```
sudo apt-get install libeigen3-dev
```
## Install libnabo
```
$ mkdir ~/Libraries/
$ cd ~/Libraries
$ git clone git://github.com/ethz-asl/libnabo.git
$ cd libnabo
$ SRC_DIR=`pwd`
$ BUILD_DIR=${SRC_DIR}/build
$ mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
$ cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
$ make
$ sudo make install
```
## Install libpoitmatcher in catkin_ws/devel folder (specify your catkin workspace if it has a different name)
```
$ cd ~/Libraries/
$ git clone git://github.com/ethz-asl/libpointmatcher.git
$ cd libpointmatcher
$ SRC_DIR=`pwd`
$ BUILD_DIR=${SRC_DIR}/build
$ mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
$ cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel ${SRC_DIR}
$ make
$ sudo make install
```
## Install OpenCV 3.4.4 with contrib (non-free modules)
```
$ cd ~/Libraries/
$ cvVersion="3.4.4"
$ mkdir installation
$ mkdir installation/OpenCV-"$cvVersion"
$ cwd=$(pwd)
$ sudo apt -y update
$ git clone https://github.com/opencv/opencv.git
$ cd opencv
$ git checkout 3.4
$ cd ..
$ git clone https://github.com/opencv/opencv_contrib.git
$ cd opencv_contrib
$ git checkout 3.4
$ cd ..
$ cd opencv
$ mkdir build
$ cd build
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
 	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_C_EXAMPLES=ON \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
	-D BUILD_EXAMPLES=ON \
	-D OPENCV_ENABLE_NONFREE=ON ..

$ make -j4
$ sudo make install
```
## Install RtabMap from source in catkin_ws/devel folder (specify your catkin workspace if it has a different name)
```
$ source /opt/ros/melodic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ sudo apt-get install ros-melodic-rtabmap ros-melodic-rtabmap-ros
$ sudo apt-get remove ros-melodic-rtabmap ros-melodic-rtabmap-ros
$ cd ~/Libraries/
$ git clone https://github.com/introlab/rtabmap.git rtabmap
$ cd rtabmap/build
$ rm -r *
$ cmake -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel ..
$ make
$ make install
```
After the cmake command, make sure that libpointmatcher and opencv with non-free modules are enabled.
## Install RtabMap_ros binaries
```
$ sudo apt-get install ros-melodic-rtabmap-ros
```
## Install rosi_challenge_ppgeas_ufsc package
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/feressalem/rosi_challenge_ppgeas_ufsc.git
$ cd ..
$ catkin build
```












