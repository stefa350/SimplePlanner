#include <ros/ros.h>
#include "basis_stuff/grid_map.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
   
    GridMap gridMap;
    gridMap.loadImage("src/img_folder");

    vector<int> rowsAndCols(2);
    rowsAndCols[0]= gridMap.rows;
    rowsAndCols[1]= gridMap.cols;

    // Initialize the ROS node
    ros::init(argc, argv, "node_gridmap");
    ros::NodeHandle nh;

    // Define the topic name
    std::string topic_name = "gridmap";

    // Create a publisher for UInt8MultiArray messages
    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>(topic_name, 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Int32MultiArray>("rowscols", 10);

    // Set the publishing rate
    ros::Rate rate(1); // 1 Hz

    

    while (ros::ok())
    {   

        // Create the message object
        std_msgs::UInt8MultiArray matrix_msg;
        matrix_msg.data = gridMap.gridMapArray;

        // Create the message object
        std_msgs::Int32MultiArray rowsAndColsMsg;
        rowsAndColsMsg.data = rowsAndCols;
        

        // Publish the message
        pub.publish(matrix_msg);
        pub2.publish(rowsAndColsMsg);
        rate.sleep();
    }

    return 0;
}