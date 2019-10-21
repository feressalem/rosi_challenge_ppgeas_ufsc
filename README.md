# PPGEAS - UFSC - ROSI CHALLENGE - XIV SBAI

![rosi_banner](https://raw.githubusercontent.com/filRocha/rosiChallenge-sbai2019/master/resources/banner2.png)

This repository contains the ROS package solution from **PPGEAS-UFSC** group for the **ROSI CHALLENGE** competition that occurs on the **XIV SBAI**, to be held in Ouro Preto (Brazil) in october 2019. 

# Package description

This repository is structured as a ROS stack. The folders organization of ppgeas is as follows:

- `launch` - Contains ROS launch files for mapping and localization. 

- `src` - Eventual C++ nodes for specific tasks. 

- `script` - Eventual python nodes for specific tasks. 

- `config` - Configuration files.

- `msg` - Generated custom message types.

- `srv` - Generated custom service types.

Besides this package we used a moveit config package and IKFAST pluging for planning and execution of UR-5 arm movements.

# List of C++ nodes used in the project

| C++ Node  | Description |
| ------------- | ------------- |
| clock.cpp  | Republishes simulation time in /clock topic.  |
| fake_controller_output.cpp  | Controls UR-5 arm through.   |
| kinect_converter.cpp  | Flips kinect rgb and republishes with camera info.  |
| kinect_converter_depth.cpp  | Flips kinect rgb and republishes.  |
| machine_state.cpp  | State Machine that brings everything together for autonomy.  |
| odometry.cpp  | Odometry /tf and topic obtained through IMU and GPS sensors.  |

# List of Python nodes used in the project

| Python Node  | Description |
| ------------- | ------------- |
| arm_controller.py  | Contains servvices servers for controlling UR-5 arm.  |
| base_controller.py  | Converts /cmd_vel messages to specific wheels angular velocities.  |
| control_arm_speed.py  | Controls tracking arms velocities.  |
| conveyorbelt_detection.py  | Detects roll center.  |
| fire_detection.py  | Detects fire.  |


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

## All Dependencies installation via Rosdep (preferred option)
After cloning our repository, it is possible to install all dependencies listed in the file package.xml by running the following commands:
```
$ cd <your_catkin_workspace>
$ rosdep install --from-paths src --ignore-src -r -y
```

## Install imutils (Python dependency)
Imutils is used for visualization of images.
```
$ sudo pip2 install imutils
```

## Install move_base, moveit, rgbd_launch, eigen, RtabMap_ros, teb_local_planner and map_server
Alternatively, the dependencies may be installed individually by the following commands:
```
$ sudo apt-get install ros-melodic-move-base
$ sudo apt-get install ros-melodic-moveit
$ sudo apt-get install ros-melodic-rgbd-launch
$ sudo apt-get install libeigen3-dev
$ sudo apt-get install ros-melodic-rtabmap-ros
$ sudo apt-get install ros-melodic-teb-local-planner
$ sudo apt-get install ros-melodic-map-server
$ sudo apt-get install ros-melodic-navigation
```

## Running the solution
We made two options to run our solution:
Option 1 (preferred option):
```
$ roslaunch ppgeas foo.launch
```
Option 2 (with 3d mapping):
```
$ roslaunch ppgeas foo-mapping.launch rtabmap_args:="--delete_db_on_start" rtabmapviz:=true
```


## Obs.: For evaluation purposes, please try with the "time_header_getSimTime" flag both to false and true in simulation_parameters.yaml of rosi_defy package and run the following command before starting simulation:
```
$ roslaunch rosi_defy load_parameters.launch
```

## Obs. 2: Issue with floating points in urdf.
Rviz has some kind of issue with floating points in urdf, in order to load robot description correctly in Rviz append the following code to your .bashrc file.
```
$ echo 'export LC_NUMERIC='en_US.UTF-8'' >> ~/.bashrc 
```
