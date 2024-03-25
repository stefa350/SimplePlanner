#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "basis_stuff/grid_map.h"
#include <vector>
#include <iostream>
#include <nav_msgs/Path.h> 

using namespace std;

bool gridMapFlag = false;
bool baseLinkFlag = false;
bool goalFlag = false;
bool baseLinkAndGoalReceived = false;
bool RowsAndColsFlag = false;
bool gridMapOccFlag = false;
GridMap gridMap;
pair<int, int> start;
pair<int, int>  goal;
vector<uint8_t> gridMapMsg;
vector<int>RowsAndColsMsg;
nav_msgs::OccupancyGrid gridmapocc;


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

void gridMapOccCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // Qui puoi accedere ai dati della mappa di occupazione
    // ad esempio, per copiare i dati in una variabile globale:
    if (gridMapOccFlag==false){
        ROS_INFO("grid_map_Occ received");
    }
    gridmapocc = *msg;
    gridMapOccFlag = true;

}



int main(int argc, char** argv)
{
  std::cerr<<"node_planner starting"<<endl;

  ros::init(argc, argv, "node_planner");
  ros::NodeHandle nh;

  //tf2_ros::Buffer tfBuffer;
  //tf2_ros::TransformListener tfListener(tfBuffer);
  
  // Create a subscriber for UInt8MultiArray messages
  ros::Subscriber sub = nh.subscribe("gridmap", 10, gridMapCallback);
  ros::Subscriber sub2 = nh.subscribe("rowscols", 10, rowsAndColsCallback);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);

  geometry_msgs::PoseStamped goalPose;
  geometry_msgs::PoseStamped baseLinkPose;

  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("goal",1, [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goalPose.pose = msg->pose;

    goalFlag = true;
  });

  ros::Subscriber base_link_sub = nh.subscribe<geometry_msgs::PoseStamped>("base_link", 1, [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
    baseLinkPose.pose = msg->pose;
    baseLinkFlag = true;
  });

  ros::Subscriber sub_occ = nh.subscribe("map", 10, gridMapOccCallback);
  


  ros::Rate rate(10.0);

  


  while (ros::ok()) {
    ros::spinOnce(); 
    gridMap.setStartGoal(start,goal,baseLinkPose, goalPose);
    start = std::make_pair(baseLinkPose.pose.position.x, baseLinkPose.pose.position.y);
    goal  = std::make_pair(goalPose.pose.position.x, goalPose.pose.position.y);


    
    if (baseLinkFlag==true && goalFlag==true && baseLinkAndGoalReceived==false){
        ROS_INFO("goal and baselink received");
        baseLinkAndGoalReceived = true;};
    if (gridMapFlag==true && baseLinkFlag==true && goalFlag==true && RowsAndColsFlag && gridMapOccFlag){
        //gridMap.loadFromVec(gridMapMsg,RowsAndColsMsg[0],RowsAndColsMsg[1]);
        gridMap.computeDistanceMap(gridmapocc);
        vector<pair<int, int>> path;
        
        path = gridMap.findPath(start,goal);
        //gridMap.displayPath(path);

        // Crea un messaggio Path e riempilo con il percorso
            nav_msgs::Path path_msg;
            path_msg.header.stamp = ros::Time::now();
            path_msg.header.frame_id = "map";
            for (const auto& point : path) {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";
                pose.pose.position.x = point.second;
                pose.pose.position.y = RowsAndColsMsg[0] - point.first;
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }

            // Pubblica il messaggio Path
            path_pub.publish(path_msg);


        ROS_INFO("completed");
        rate.sleep();
        
        
    }
    

    rate.sleep();
  }
  return 0;
}