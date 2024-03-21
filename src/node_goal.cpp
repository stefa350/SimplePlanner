#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

int main(int argc, char** argv){

    if (argc != 3) {
        cerr << "Usage: rosrun <package_name> node_goal <x_coordinate> <y_coordinate>" << endl;
        return 1;
    }

    int x = atoi(argv[1]);
    int y = atoi(argv[2]);

    cout << "node_goal is now starting..." << endl;
    ros::init(argc,argv,"node_goal");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("goal", 10);

    ros::Rate rate(10);

    cout << "Goal: (" << x << ";" << y << ")" << endl;
    

    while (ros::ok()){
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = "map";
        //poseStamped.child_frame_id = "goal";
        poseStamped.pose.position.x = x;
        poseStamped.pose.position.y =y;
        poseStamped.pose.position.z = 0.0;
        poseStamped.pose.orientation.x = 0.0;
        poseStamped.pose.orientation.y = 0.0;
        poseStamped.pose.orientation.z = 0.0;
        poseStamped.pose.orientation.w = 1.0;

        pub.publish(poseStamped);
        
        rate.sleep();
    }
    return 0;

}