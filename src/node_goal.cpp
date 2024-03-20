#include <iostream>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
using namespace std;

int main(int argc, char** argv){

    if (argc != 3) {
        cerr << "Usage: rosrun <package_name> node_goal <x_coordinate> <y_coordinate>" << endl;
        return 1;
    }

    int x = atoi(argv[1]);
    int y = atoi(argv[2]);

    cout << "node_baselink is now starting..." << endl;
    ros::init(argc,argv,"node_goal");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;


    ros::Rate rate(10);

    cout << "Base Link: (" << x << ";" << y << ")" << endl;
    

    while (nh.ok()){
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = y;
        transformStamped.transform.translation.y = x;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        br.sendTransform(transformStamped);
        rate.sleep();
    }
    return 0;

}