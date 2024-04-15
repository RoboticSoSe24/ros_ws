# TUBAF_Robotik_24
Repository for the summer semester 24 robotics course at the TU Freiberg 

+ [Requirements](#requirements)
+ [Packages](#packages)
+ [Build](#build)
+ [Run](#run)
+ [Parameters](#parameters)

## Requirements
The project has been set up to work with the 
following environment and dependencies installed: 

- Ubuntu 22.04.
- ROS 2 Humble Hawksbill
- Python 3
- OpenCV
- OpenCV contrib 

## Packages
### Perception
Package for nodes connected to turtlebot environment perception

### Planning
Package for nodes connected to turtlebot motion planning

## Build
In the workspace directory call:

```colcon build```

### Run
In the workspace directory call:

```
source install/setup.bash
ros2 run perception laneObserver
```

In another terminal navigate to the workspace directory and call:

```
source install/setup.bash
ros2 run planning defaultDriving
```

### Parameters

laneObserver:
- 'canny_threshold'  
- 'line_width'
- 'lane_width'

defaultDriving:
- 'scan_angle'
- 'boundary_left'
- 'boundary_right'
- 'speed_drive'
- 'speed_turn'
