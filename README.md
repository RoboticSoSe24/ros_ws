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

for configuring lane detection parameters before starting to drive call:
- ```ros2 run perception laneObserver``` then
- ```ros2 launch launch_files/no_lanes.yaml```


alternatively launch without additional configuration according to the task via:
- ```ros2 launch launch_files/basic_driving.yaml``` 
  ...for basic lane following, stopping in front of obstructions, and parking behind parking signs
- ```ros2 launch launch_files/obstruction.yaml```
  ...for basic lane following and overtaking obstructions on the road

### Parameters

- **laneObserver**
  - *canny_threshold*     - threshold for edge detection, increase if contrast is low
  - *line_width*          - line width of relevant road markings, decrease if you see black pixels within road markings in the image "line width filter", increase if road markings are blue but not cyan in the image "line width filter"
  - *line_tolerance*      - tolerance on width filtering, decrease for stricter filtering, increase if only a part of the road markings shows up cyan in the image "line width filter" 
  - *lane_width*          - width of the road lanes, increase if the green and blue areas in the image "countour 1, contour 2, centerline" are not overlapping
  - *dot_line_length*     - segment length of dotted line
  - *dot_line_tolerance*  - tolerance on dotted line segment length filtreing, decrease for stricter filtering


- **signObserver**
  - *min_sign_count*      - number of consecutive frames one type of traffic sign should be recognised in before publishing to the signs topic


- **stateController**
  - *distance_to_stop*    - laserscan distance below which normal driving will be stopped
  - *scan_angle*          - angle in front of the robot in which to search for obstructions


- **defaultDriving**
  - *boundary_left*       - threshold for when to correct turning, decrease to respond sooner 
  - *boundary_right*      - threshold for when to correct turning, decrease to respond sooner
  - *speed_drive*         - multiplier for linear drive speed
  - *speed_turn*          - multiplier for angular drive speed


- **obstruction**
  - *scan_angle*          - opening angle within which laserscan ranges will be observed during obstruction overtake
  - *scan_direction*      - directional offset to the scan angle range (counter-clockwise)


- **parking**
  - *scan_height*         - dimension of parking spot parallel to road
  - *scan_width*          - dimension of parking spot perpendicular to road
  - *idle_drive_time*     - seconds of idle driving after recognising a parking sign, time needed to reach first parking spot
  - *parking_time*        - seconds to wait once on the parking spot
  - *max_distance*        - maximum distance to follow the road before giving up on finding a free parking spot