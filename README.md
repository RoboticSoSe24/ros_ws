# TUBAF_Robotik_24
Repository for the summer semester 24 robotics course at the TU Freiberg 

- [TUBAF\_Robotik\_24](#tubaf_robotik_24)
  - [Requirements](#requirements)
  - [Packages](#packages)
    - [Perception](#perception)
    - [Planning](#planning)
  - [Build](#build)
    - [Run](#run)
    - [Parameters](#parameters)

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

```
colcon build
```

### Run
First in the workspace directory call:

```
source install/setup.bash
```

then launch according to the task via:
- ```ros2 launch launch_files/basic_driving.yaml``` 
  ...for basic lane following and stopping in front of obstructions
- ```ros2 launch launch_files/obstruction.yaml```
  ...for basic lane following and overtaking obstructions on the road

### Parameters

- **laneObserver**
  - *canny_threshold*   - threshold for edge detection, increase if contrast is low
  - *line_width*          - line width of relevant road markings
  - *line_tolerance*      - tolerance on width filtering, decrease for stricter filtering
  - *lane_width*          - width of the road lanes
  - *dot_line_length*     - segment length of dotted line
  - *dot_line_tolerance*  - tolerance on dotted line segment length filtreing, decrease for stricter filtering


- **stateController**
  - *distance_to_stop*    - laserscan distance below which normal driving will be stopped
  - *scan_angle*          - angle in front of the robot in which to search for obstructions


- **defaultDriving**
  - *boundary_left*       - threshold for when to correct turning, decrease to respond sooner 
  - *boundary_right*      - threshold for when to correct turning, decrease to respond sooner
  - *speed_drive*         - multiplier for linear drive speed
  - *speed_turn*          - multiplier for angular drive speed


- **obstruction**
  - *scan_angle*          - angle to the right of the robot in which to search for obstructions
  - *buffer*              - minimum number of values to recognize an object
  - *object_distance*     - laserscan distance below which an obstruction is considered adjacent
