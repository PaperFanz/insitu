# User Guide

## Useful Key Combinations

`Ctrl + Shift + m` New Mode
`Ctrl + Shift + v` New View

Alternatively, these functions can be activated through the toolbar under the `Edit` menu.

## Building from Source

This guide assumes you've installed ROS on your system.
 
If not, please refer to the installation guide at this link for an Ubuntu installation of ROS:

http://wiki.ros.org/melodic/Installation/Ubuntu

Note that InSitu has only been tested on ROS Melodic/Ubutnu 18.04 so far. See To-Do list in [CONTRIBUTING.md](CONTRIBUTING.md) for status on other setups.

1. Create and source a new catkin workspace
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
source devel/setup.bash
```

2. Git clone this repo into your workspace 
```sh
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
 
4. Go to the top directory of your catkin workspace where the source code of InSitu package is install all dependencies required for InSitu (particularly QT)

```sh
rosdep install --from-paths src --ignore-src -r -y
```
    
5. Install clang to compile C++ code for InSitu
```sh
sudo apt-get update
sudo apt-get install clang-10
```

6. Source the workspace
```sh
cd ~/catkin_ws/
source devel/setup.bash 
```

7. Launch roscore in one terminal
```sh
roscore
```

8. Open a new terminal and run InSitu 

```sh
rosrun insitu insitu
```
 
