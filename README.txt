In order to run the project you have to do the following commands in 3 different terminals.
1. roscore
2. rosrun rviz rviz
3. rosrun ros_controller node_gridmap

Once you are in rviz you have to add by topic the map and the path, you can set the view to some values like x: 940, y:940, z:1920.

If roscore/rosrun doesn't work, use this command in each terminal:
source /opt/ros/noetic/setup.bash

If the package ros_controller is not found, use this commmand in the terminal when you are in the folder ros_controller:
source devel/setup.bash
