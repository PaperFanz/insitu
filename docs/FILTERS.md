# Creating Filters

## Filter Creation Script

To create a filter template, simply run the filter creation script provided with InSitu inside your catkin workspace:

```
rosrun insitu new_insitu_filter
```

Simply follow the prompts and the appropriate folder structure and source/build files will be generated automatically.

e.g.
```
user@hostname:~/catkin_ws/src$ rosrun insitu new_insitu_filter 
Enter package name: myPackage
Enter filter name: myFilter
Enter filter description: desc
myPackage
├── CMakeLists.txt
├── inc
│   └── myFilter
│       ├── myFilter_dialog.hpp
│       └── myFilter.hpp
├── lib
├── package.xml
├── plugins.xml
└── src
    └── myFilter
        ├── myFilter.cpp
        └── myFilter_dialog.cpp

5 directories, 7 files
0

Done
```

## Filter Architecture

Under Construction!

