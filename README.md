# SimplePlanner

This project implements a simple A* path planner using ROS and RViz.  
Given an image representing a maze or a gridmap, the planner computes a path from a start point to a goal using the A* search algorithm. The result is then visualized in RViz.
## How to Run

### 1. Open 3 separate terminals and run the following commands:

**Terminal 1:**
```bash
roscore

**Terminal 2:**
```bash
rosrun rviz rviz

**Terminal 3:**
```bash
rosrun ros_controller node_gridmap

Once you are in rviz you have to add by topic the map and the path, you can set the view to some values like x: 640, y:640, z:1280.

If roscore/rosrun doesn't work, use this command in each terminal:
source /opt/ros/noetic/setup.bash

If the package ros_controller is not found, use this commmand in the terminal when you are in the folder ros_controller:
source devel/setup.bash
