# PGEAS - UFSC - ROSI CHALLENGE - XIV SBAI

![rosi_banner](https://raw.githubusercontent.com/filRocha/rosiChallenge-sbai2019/master/resources/banner2.png)

This repository contains the ROS package solution from **PPGEAS-UFSC** group for the **ROSI CHALLENGE** competition that occurs on the **XIV SBAI**, to be held in Ouro Preto (Brazil) in october 2019. 

# Package description

This repository is structured as a ROS package. The folders organization is as follows:

- `launch` - Contains ROS launch files for maping and localization. 

- `src` - Eventual C++ nodes for specific tasks. 

- `script` - Eventual python nodes for specific tasks. 

- `config` - Configuration files.

- `maps` - Contains 3d point cloud database and 2d gridmap.

- `msg` - Generated message types.


# Installation

The simulator was conceived using **Ubuntu 18.4.2**, **ROS Melodic**, and **V-REP 3.6.2 (rev.0)**. Another software versions might work, but they are not recommended nor officially supported for the competition. 

## Dependencies installation advices

By covention on this installation steps, all boxes starting with a `$` mark means that you should run the command on the terminal. 

## Install move_base
```
sudo apt-get install ros-melodic-move-base
```
## Install rgbd_launch
```
sudo apt-get install ros-melodic-rgbd-launch
```
## Install eigen
```
sudo apt-get install libeigen3-dev
```
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












