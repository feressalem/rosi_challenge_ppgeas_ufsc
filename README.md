# PPGEAS - UFSC - ROSI CHALLENGE - XIV SBAI

![rosi_banner](https://raw.githubusercontent.com/filRocha/rosiChallenge-sbai2019/master/resources/banner2.png)

This repository contains the ROS package solution from **PPGEAS-UFSC** group for the **ROSI CHALLENGE** competition that occurs on the **XIV SBAI**, to be held in Ouro Preto (Brazil) in october 2019. 

# Package description

This repository is structured as a ROS package. The folders organization is as follows:

- `launch` - Contains ROS launch files for mapping and localization. 

- `src` - Eventual C++ nodes for specific tasks. 

- `script` - Eventual python nodes for specific tasks. 

- `config` - Configuration files.

- `maps` - Contains 3d point cloud database and 2d gridmap.

- `msg` - Generated custom message types.

- `srv` - Generated custom service types.


# Installation

The simulator was conceived using **Ubuntu 18.4.2**, **ROS Melodic**, and **V-REP 3.6.2 (rev.0)**. Another software versions might work, but they are not recommended nor officially supported for the competition. 

## Install rosi_challenge_ppgeas_ufsc package
By covention on this installation steps, all boxes starting with a `$` mark means that you should run the command on the terminal.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/feressalem/rosi_challenge_ppgeas_ufsc.git
$ cd ..
$ catkin build
```

## All Dependencies installation via Rosdep
After cloning our repository, it is possible to install all dependencies listed in the file package.xml by running the following commands:
```
$ cd <your_catkin_workspace>
$ rosdep install --from-paths src --ignore-src -r -y
```

## Install move_base, moveit, rgbd_launch, eigen, RtabMap_ros, dwa_local_planner and map_server
Alternatively, the dependencies may be installed individually by the following commands:
```
$ sudo apt-get install ros-melodic-move-base
$ sudo apt-get install ros-melodic-moveit
$ sudo apt-get install ros-melodic-rgbd-launch
$ sudo apt-get install libeigen3-dev
$ sudo apt-get install ros-melodic-rtabmap-ros
$ sudo apt-get install ros-melodic-dwa-local-planner
$ sudo apt-get install ros-melodic-map-server
```

## Running the solution
We made three options to run our solution:
Option 1:
```
$ roslaunch ppgeas mapping.launch rtabmap_args:="--delete_db_on_start"
```
Option 2:
```
$ roslaunch ppgeas mapping.launch rtabmap_args:="--delete_db_on_start" rtabmapviz:=true
```
Option 3:
```
$ roslaunch ppgeas mapping.launch rtabmap_args:="--delete_db_on_start" rviz:=true
```

## Fire detection
The fire detection may be seem by the image in the topic /fire_test

## Obs. 1: Bug detected in the timestamp
Due to a bug detected related to the images timestamp, our solution resulted in a very slow simulation time, what made difficult to run the touch tests from the UR-5, even when disabling the simulation rendering.

## Obs. 2: Issue with floating points in urdf
Rviz has some kind of issue with floating points in urdf, in order to load robot description correctly in Rviz append the following code to your .bashrc file.
```
$ echo 'export LC_NUMERIC='en_US.UTF-8'' >> ~/.bashrc 
```












