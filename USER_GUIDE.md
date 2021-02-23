# User Guide

!! UNDER CONSTRUCTION !!

## Building from Source

This guide assumes you've installed ROS on your system.
 
If not, please refer to the installation guide at this link for an Ubuntu installation of ROS:

http://wiki.ros.org/melodic/Installation/Ubuntu

Also, inSitu has only been tested on ROS Melodic so far. 


1. Create and source a new catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
source devel/setup.bash
```

2. Git clone this repo into your workspace 
```
https://github.com/PaperFanz/insitu.git
```
3. Install and initialize rosdep
```
sudo apt-get install python-rosdep
sudo rosdep init
```

4. Update ROS dependencies

```
rosdep update
```
 
4. Go to the top directory of your catkin workspace where the source code of inSitu package is install all dependencies required for inSitu (particularly QT)

```
rosdep install --from-paths src --ignore-src -r -y
```
    
5. Install clang to compile C++ code for inSitu
```
sudo apt update
sudo apt-get install clang-10
```

6. Source the workspace
```
cd ~/catkin_ws/
source devel/setup.bash 
```

7. Launch roscore in one terminal
```
roscore
```

8. Open a new terminal and run inSitu 

```
rosrun insitu insitu
```
 
