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


