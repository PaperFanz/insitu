# User Guide

!! UNDER CONSTRUCTION !!

## Building from Source

1. Create and source a new workspace, if you don’t already have one
2. Git clone this repo into your workspace 
3. Install and initialize rosdep if you haven’t already
4. Run

    ```rosdep update```
 
4. Go to the top directory of your catkin workspace where the source code of inSitu package is install all dependencies required for inSitu (particularly QT). Run:

    ```rosdep install --from-paths src --ignore-src -r -y```
    
5. Install clang to compile C++ during the catkin build if you haven’t already.
    
    ```
    sudo apt update
    sudo apt-get install clang-10
    ```

6. Source the workspace

7. Run inSitu: 

    ```rosrun insitu insitu```
