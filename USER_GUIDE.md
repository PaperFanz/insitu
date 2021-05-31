# User Guide

## Useful Key Combinations
New Mode:

`Ctrl + Shift + m`  

New View:

`Ctrl + Shift + v`  

Increase/Decrease filter size:

`Alt + Click and drag left mouse button` on filter


Alternatively, these functions can be activated through the toolbar under the `Edit` menu.

## Building from Source

This guide assumes that ROS and catkin tools have been installed.
If not, please refer to the installation guide at this link for an Ubuntu 20 installation of ROS and catkin tools respectively:

http://wiki.ros.org/noetic/Installation/Ubuntu

http://wiki.ros.org/catkin 
OR 
https://catkin-tools.readthedocs.io/en/latest/installing.html


See To-Do list in [CONTRIBUTING.md](CONTRIBUTING.md) for status on other setups, bug fixes, new features, etc.

1. Create and source a new catkin workspace in a new terminal
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
source devel/setup.bash
```

2. Git clone this repo into your source folder 
```sh
cd ~/catkin_ws/src
git clone https://github.com/UTNuclearRobotics/inSitu_VB.git
```

3. Install and initialize rosdep
```sh
sudo apt install python3-rosdep 
sudo rosdep init
```

4. Update ROS dependencies

```sh
rosdep update
```
 
5. Go to the top directory of your catkin workspace where the source code of the InSitu package is and install all dependencies required for InSitu (particularly QT)

```sh
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
    
6. Install clang to compile C++ code for InSitu
```sh
sudo apt-get update
sudo apt-get install clang-format
```

7. Clone qt_ros repo into your src folder
```sh
cd ~/catkin_ws/src
git clone https://github.com/stonier/qt_ros.git

#delete qt_tutorials folder due to errors in build
rm -r qt_tutorials
```


8. Build the packages
```sh
catkin build
```

9. Source the workspace
```sh
cd ~/catkin_ws/
source devel/setup.bash 
```

10. Open another terminal and launch roscore
```sh
roscore
```

11. Return to the first terminal (in step #7) and run InSitu 

```sh
rosrun insitu insitu
```
 
Optional_1:
Add the source path to the .bashrc file to skip step 9 everytime you build the workspace
```sh
source ~</PATH_TO_WS_>/devel/setup.bash
```

Optional_2:
Create an alias to build only insitu pkgs after initial build with qt. Add this to .bashrc (or .bash_aliases, but remember to source .bash_aliases everytime)
```sh
alias build_insitu='catkin build insitu insitu_plugins -j4'
```

Optional_3
Create an alias to run insitu. Ensure Optional_1 is done for this option. 
```sh
alias insitu='rosrun insitu insitu'
```

Workflow:

Make/add changes to insitu or insitu_plugin folders

Run "build_insitu"

Run "roscore"

Run "insitu" 