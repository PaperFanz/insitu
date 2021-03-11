# User Guide

## Useful Key Combinations

`Ctrl + Shift + m` New Mode
`Ctrl + Shift + v` New View

Alternatively, these functions can be activated through the toolbar under the `Edit` menu.

## Building from Source

This guide assumes that ROS and catkin tools have been installed.
If not, please refer to the installation guide at this link for an Ubuntu 18 installation of ROS and catkin tools respectively:

http://wiki.ros.org/melodic/Installation/Ubuntu

https://catkin-tools.readthedocs.io/en/latest/installing.html

Please note that InSitu has only been tested on ROS Melodic/Ubutnu 18.04 so far. See To-Do list in [CONTRIBUTING.md](CONTRIBUTING.md) for status on other setups.

1. Create and source a new catkin workspace
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
source devel/setup.bash
```

2. Git clone this repo into your source folder 
```sh
cd ~/catkin_ws/src
git clone https://github.com/PaperFanz/insitu.git
or
git clone git@github.com:PaperFanz/insitu.git
```
3. Install and initialize rosdep
```sh
sudo apt-get install python-rosdep
sudo rosdep init
```

4. Update ROS dependencies

```sh
rosdep update
```
 
4. Go to the top directory of your catkin workspace where the source code of the InSitu package is and install all dependencies required for InSitu (particularly QT)

```sh
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
    
5. Install clang to compile C++ code for InSitu
```sh
sudo apt-get update
sudo apt-get install clang-10
```

6. Build the packages
```sh
catkin build
```
7. Source the workspace
```sh
cd ~/catkin_ws/
source devel/setup.bash 
```

7. Open another terminal and launch roscore
```sh
roscore
```

8. Return to the first terminal (in step #7) and run InSitu 

```sh
rosrun insitu insitu
```
 
