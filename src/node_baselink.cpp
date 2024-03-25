#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

int main(int argc, char** argv){
/*
    if (argc != 3) {
        cerr << "Usage: rosrun <package_name> node_baselink <x_coordinate> <y_coordinate>" << endl;
        return 1;
    }

    int x = atoi(argv[1]);
    int y = atoi(argv[2]);

    cout << "node_baselink is now starting..." << endl;
    ros::init(argc,argv,"node_baselink");
    ros::NodeHandle nh;

   // tf2_ros::TransformBroadcaster br;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("base_link",10);
    //geometry_msgs::TransformStamped transformStamped;


    ros::Rate rate(10);

    cout << "Base Link: (" << x << ";" << y << ")" << endl;
    

    while (nh.ok()){
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = "map";
        //poseStamped.child_frame_id = "base_link";
        poseStamped.pose.position.x = x;
        poseStamped.pose.position.y = y;
        poseStamped.pose.position.z = 0.0;
        poseStamped.pose.orientation.x = 0.0;
        poseStamped.pose.orientation.y = 0.0;
        poseStamped.pose.orientation.z = 0.0;
        poseStamped.pose.orientation.w = 1.0;

        //br.sendTransform(transformStamped);
        pub.publish(poseStamped);
        rate.sleep();
    }*/
    return 0;

}
