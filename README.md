# Insitu

![InSitu Logo](docs/insitu-icon-block-alt.png)

- [Insitu](#insitu)
  * [Description](#description)
  * [Quick Start](#quick-start)
    + [Building From Source](#building-from-source)
    + [Running InSitu](#running-insitu)
    + [User Guide](#user-guide)
  * [Available Filters](#available-filters)
  * [Contribute](#contribute)
    + [Creating Custom Filters](#creating-custom-filters)
    + [Modifying InSitu Core](#modifying-insitu-core)
  * [Related Work](#related-work)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

<!-- TODO add GIF of InSitu in action -->

## Description

InSitu is an extensible situational awareness platform that organizes ROS image
streams in a grid layout and applys overlays to create complex HUD interfaces
for robot teleoperation and supervision. These overlays are created through 
user-created plugins called `filters`, but first time users are encouraged to
check out the list of [Available Filters](##Available Filters) to get started 
without needing to write any code themselves. InSitu was inspired by [Guy 
Zaidner's work](https://www.youtube.com/watch?v=QCgzkMhAX68) at the 
[Nuclear and Applied Robotics Group](https://robotics.me.utexas.edu/) at the 
University of Texas at Austin and built on ROS pluginlib, Qt5, OpenCV, and 
jsoncpp.

## Quick Start

InSitu is not packaged for ROS yet so new users will have to build from
from source. 

### Building From Source
First, ensure that you've installed ROS Melodic and `rosdep` 
following the [ROS wiki](http://wiki.ros.org/melodic/Installation). Then, 
create a new catkin workspace and clone the repository into the `src` 
directory.

```sh
mkdir -p catkin_ws/src && cd catkin_ws
catkin build
source devel/setup.bash
cd src
git clone git@github.com:PaperFanz/insitu.git
```

Next, install any dependencies by running the following from the top directory
of your catkin workspace:

```sh
# in catkin_ws:
rosdep install --from-paths src --ignore-src -r -y
```

Finally, build the packages:

```sh
catkin build
```

### Running InSitu

To run InSitu, start `roscore` in a separate terminal and start some nodes that
publish image topics, such as [usb\_cam](http://wiki.ros.org/usb_cam). Then, 
run InSitu using `rosrun insitu insitu`. 

<!-- TODO gifs and basic demo guide -->

### User Guide

For a comprehensive list of InSitu functions and configuration options, please
refer to the [USER\_GUIDE.md](docs/USER_GUIDE.md).

## Available Filters

Curated list of tested, publicly available filter packages for InSitu.

- [insitu\_plugins](insitu_plugins): a set of filters included with InSitu 
    that provides some out-of-box functionality
- [iort\_filters](https://github.com/PaperFanz/iort_filters): overlays for IoT 
    data streamed using [iort\_lib](https://github.com/PaperFanz/iort_lib)

## Contribute

There are two ways to contribute to the InSitu project, creating new filters
to add custom functionality or modifying InSitu core to fix bugs and develop
new features.

### Creating Custom Filters

Those interested in creating their own filters should refer to 
[FILTERS.md](docs/FILTERS.md) to get started with the filter creation script
that generates the necessary boilerplate and explains the basic operation of
a filter.

### Modifying InSitu Core

Found a bug or want to see your custom feature merged into InSitu core? Fork
the repository, make your changes, and open a pull request. Please refer to
[CONTRIBUTING.md](docs/CONTRIBUTING.md) for an explanation of InSitu's program
structure and more detailed contribution guidelines.

## Related Work

While InSitu was functionally inspired by Guy Zaider's work referenced in the
description, its implementation was heavily based on RQT and RVIZ, two existing
giants in the ROS Visualization application sphere. InSitu's plugin architecture
is modeled after the great work that went into these related projects.

