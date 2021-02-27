# Insitu

Hardware agnostic situational awareness

## Quick Links

[User Guide](USER_GUIDE.md)

[Creating a filter](FILTERS.md)

[Contributing to Insitu](CONTRIBUTING.md)

## Description

<!-- TODO add images -->

Most ROS users are likely very familiar with RVIZ and RQT, and the plugin functionality that these two tools bring. Between RVIZ and RQT, most GUI requirements, 3D or 2D, can be satisfied with little effort other than selecting the appropriate plugins from the massive collection of GUI tools available.

However, as robotics development advances, the complexity of the interfaces used in teleoperation and monitoring increases as well. Guy Zaidner's work in situational awareness at the University of Texas at Austin's Nuclear and Applied Robotics Group demonstrated the utility of dynamic HUD overlays in robotic teleoperation. The HUD composed multiple camera streams and ROS sensor data into a unified user interface, a marked improvement over referring to dials and charts in another window (or raw data from a terminal).

Insitu is a continuation of Guy's work in overlays and aims to provide the same plugin support for camera overlays as RQT and RVIZ did for 2D and 3D GUIs. In essence, Insitu provides a common interface for composing multiple filters on a ROS image stream. The filters are implemented as ROS nodelets and can range in complexity from static crosshairs to full computer vision applications, as long as they implement the Insitu filter interface.

## Structure

Insitu has three levels of organization: modes, views, and filters. Modes are containers for related views. For example, the navigation mode might contain camera views from front and rear mounted cameras, whereas the manipulation mode might contain camera views from the end effectors of arms mounted on the robot. Views themselves are rather simple, and display a single ROS image stream. The real power of Insitu comes from the filter plugins, which can be loaded, applied, and dynamically configured for any view.

Common filters can be implemented to take configurations at runtime and shared with other users, drastically reducing the amount of time needed to create a complex HUD interface.
