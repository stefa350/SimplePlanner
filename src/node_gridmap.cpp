#include <ros/ros.h>
#include "basis_stuff/grid_map.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h> 


nav_msgs::Path convertiVectorToPath(const std::vector<std::pair<int, int>>& vettore) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now(); // Imposta il timestamp
    path_msg.header.frame_id = "map"; // Sostituisci "nome_frame" con il frame ID desiderato

    for (const auto& coppia : vettore) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = coppia.first; // Assegna la prima parte della coppia all'asse x
        pose_stamped.pose.position.y = coppia.second; // Assegna la seconda parte della coppia all'asse y
        pose_stamped.pose.position.z = 0.0; // Imposta z a zero o al valore desiderato
        pose_stamped.pose.orientation.x = 0.0; // Imposta l'orientamento se necessario
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.header.stamp = ros::Time::now(); // Imposta il timestamp per questa posa
        pose_stamped.header.frame_id = "map"; // Imposta il frame ID per questa posa

        path_msg.poses.push_back(pose_stamped); // Aggiunge la posa alla sequenza
    }

    return path_msg;
}

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

    pair<int, int> start = {127,0};  
    pair<int, int> goal = {560,950};
    gridMap.setStartGoal(start,goal);


    vector<int> rowsAndCols(2);
    rowsAndCols[0]= gridMap.rows;
    rowsAndCols[1]= gridMap.cols;

    // Initialize the ROS node
    ros::init(argc, argv, "node_gridmap");
    ros::NodeHandle nh;


    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map",10);
    vector<pair<int, int>> path_;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 10);

    
    cv::Mat img = cv::imread("/home/lattinone/catkin_ws/ws_rp/src/ros_controller/src/img_folder/labirinto.jpg", cv::IMREAD_GRAYSCALE);
    if(img.empty()){
        ROS_ERROR("Failed to load img");
        return -1;
    }else cout << "ok" << endl;

    
    nav_msgs::OccupancyGrid grid = imageToOccupancyGrid(img);
    gridMap.setOccupancy(grid);
    gridMap.computeDistanceMap();


    path_ = gridMap.findPath(start,goal);

    nav_msgs::Path path = convertiVectorToPath(path_);


    // Set the publishing rate
    ros::Rate rate(80); // 1 Hz

    

    while (ros::ok())
    {   
        map_pub.publish(grid);
        
        

        /*
        nav_msgs::OccupancyGrid grid_msg;
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "map"; 

        // Set the grid size
        grid_msg.info.width = rowsAndCols[0];
        grid_msg.info.height = rowsAndCols[1];

        // Set the grid resolution (e.g., in meters/cell)
        grid_msg.info.resolution = 1.0;

        // Set the grid origin
        grid_msg.info.origin.position.x = 0.0;
        grid_msg.info.origin.position.y = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height,0);

        // Fill the grid with the path data
        /*for (const auto& point : path.poses) {
            // Assuming that 'point' is a pair (x, y)
            

            const auto& pose = point.pose.position;
            int index = pose.x + pose.y * rowsAndCols[0];


            
            // Set the cell at the calculated index to occupied
            grid_msg.data[index] = 100;
            
        }

        // Publish the OccupancyGrid message
        map_pub.publish(grid_msg);
        */
        
        nav_msgs::Path path_msg;
            path_msg.header.stamp = ros::Time::now();
            path_msg.header.frame_id = "map";
            for (const auto& point : path.poses) {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";
                pose.pose.position.y = point.pose.position.y;
                pose.pose.position.x = rowsAndCols[0] - point.pose.position.x;
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }

            // Pubblica il messaggio Path
            path_pub.publish(path_msg);


        //ROS_INFO("completed");
        
        

        // Publish the message
        //pub.publish(matrix_msg);
        //pub2.publish(rowsAndColsMsg);
        rate.sleep();
    }

    return 0;
}