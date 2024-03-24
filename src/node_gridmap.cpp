#include <ros/ros.h>
#include "basis_stuff/grid_map.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGrid imageToOccupancyGrid(const cv::Mat& img) {
    nav_msgs::OccupancyGrid grid;

    // Imposta le dimensioni della griglia
    grid.info.width = img.cols;
    grid.info.height = img.rows;

    // Imposta la risoluzione della griglia (ad esempio, in metri/cella)
    grid.info.resolution = 1.0;

    // Imposta l'origine della griglia
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.info.origin.orientation.w = 1.0;
    // Riempie la griglia con i dati dell'immagine
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            // Supponendo che i pixel bianchi siano spazi liberi e i pixel neri siano ostacoli
            int occupancy = (img.at<uchar>(i, j) == 255) ? 0 : 100;
            grid.data.push_back(occupancy);
        }
    }
    return grid;
}

int main(int argc, char** argv) {
   
    GridMap gridMap;
    gridMap.loadImage("/home/lattinone/catkin_ws/ws_rp/src/ros_controller/src/img_folder");

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


    ////////////////////////////////////////////
    cv::Mat img = cv::imread("/home/lattinone/catkin_ws/ws_rp/src/ros_controller/src/img_folder/labirinto.jpg", cv::IMREAD_GRAYSCALE);
    if(img.empty()){
        ROS_ERROR("Failer to load img");
        return -1;
    }else cout << "ok" << endl;


    nav_msgs::OccupancyGrid grid = imageToOccupancyGrid(img);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map",10);

    
    // Set the publishing rate
    ros::Rate rate(10); // 1 Hz

    

    while (ros::ok())
    {   
        map_pub.publish(grid);
        // Create the message object
        std_msgs::UInt8MultiArray matrix_msg;
        matrix_msg.data = gridMap.gridMapArray;

        // Create the message object
        std_msgs::Int32MultiArray rowsAndColsMsg;
        rowsAndColsMsg.data = rowsAndCols;
        pair<int, int> start, goal;
        vector<pair<int, int>> path;
        path = gridMap.findPath(start,goal);

        nav_msgs::OccupancyGrid grid_msg;
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "map";  // or any other frame in which the path is defined

        // Set the grid size
        grid_msg.info.width = rowsAndColsMsg.data[0];
        grid_msg.info.height = rowsAndColsMsg.data[1];

        // Set the grid resolution (e.g., in meters/cell)
        grid_msg.info.resolution = 1.0;

        // Set the grid origin
        grid_msg.info.origin.position.x = 0.0;
        grid_msg.info.origin.position.y = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height,0);

        // Fill the grid with the path data
        for (const auto& point : path) {
            // Assuming that 'point' is a pair (x, y)

            int index = point.first + point.second * rowsAndColsMsg.data[0];

            
            // Set the cell at the calculated index to occupied
            grid_msg.data[index] = 100;
            
        }

        // Publish the OccupancyGrid message
        map_pub.publish(grid_msg);
        
        

        // Publish the message
        pub.publish(matrix_msg);
        pub2.publish(rowsAndColsMsg);
        rate.sleep();
    }

    return 0;
}