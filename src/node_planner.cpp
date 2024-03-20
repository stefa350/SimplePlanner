#include <ros/ros.h>
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "basis_stuff/grid_map.h"
#include <vector>
#include <iostream>


using namespace std;

bool gridMapFlag = false;
bool baseLinkFlag = false;
bool goalFlag = false;
bool baseLingAndGoalReceived = false;
bool RowsAndColsFlag = false;
GridMap gridMap;
pair<int, int> start, goal;
vector<uint8_t> gridMapMsg;
vector<int>RowsAndColsMsg;


void gridMapCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    if (gridMapFlag==false){
        ROS_INFO("grid_map received");
    }
    gridMapFlag=true;
    gridMapMsg = msg->data;
}

void rowsAndColsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (RowsAndColsFlag==false){
        ROS_INFO("rows_and_cols received");
    }
    RowsAndColsFlag = true;
    RowsAndColsMsg = msg->data;
}


int main(int argc, char** argv)
{
  std::cerr<<"node_planner starting"<<endl;

  ros::init(argc, argv, "node_planner");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // Create a subscriber for UInt8MultiArray messages
  ros::Subscriber sub = nh.subscribe("gridmap", 10, gridMapCallback);
  ros::Subscriber sub2 = nh.subscribe("rowscols", 10, rowsAndColsCallback);

  geometry_msgs::TransformStamped goalTransform;
  geometry_msgs::TransformStamped baseLinkTransform;

  ros::Rate rate(10.0);

  while (nh.ok()) {
    ros::spinOnce();
    
    
    try {
      goalTransform = tfBuffer.lookupTransform("map", "goal", ros::Time(0));
      goal = {goalTransform.transform.translation.x,goalTransform.transform.translation.y};
      goalFlag=true;
      
    }
    catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    try {
      baseLinkTransform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
      start = {baseLinkTransform.transform.translation.x,baseLinkTransform.transform.translation.y};
      baseLinkFlag = true;
    }
    catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    if (baseLinkFlag==true && goalFlag==true && baseLingAndGoalReceived==false){
        ROS_INFO("goal and baselink received");
        baseLingAndGoalReceived = true;};
    if (gridMapFlag==true && baseLinkFlag==true && goalFlag==true && RowsAndColsFlag){
        gridMap.loadFromVec(gridMapMsg,RowsAndColsMsg[0],RowsAndColsMsg[1]);
        gridMap.computeDistanceMap();
        vector<pair<int, int>> path;
        path = gridMap.findPath(start,goal);
        gridMap.displayPath(path);
        ROS_INFO("completed");
        return 0;
    }
    

    rate.sleep();
  }
  return 0;
}