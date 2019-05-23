#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
geometry_msgs::PoseArray pose_data;
geometry_msgs::Pose point_1, point_2, temp;
int main(int argc, char **argv) {
    ros::init(argc, argv, "generatedata");
    ros::NodeHandle nh;
    ros::Publisher data_pub = nh.advertise<geometry_msgs::PoseArray>("pose_data", 1000);
    ros::Rate loop(1);

    point_1.position.x = -0.3;point_1.position.y = 0.4;point_1.position.z = 0.6;
    point_2.position.x = 0.3;point_2.position.y = 0.6;point_2.position.z = 0.6;
    int range = 10;
    double x__ = (point_2.position.x - point_1.position.x) / range;
    double y__ = (point_2.position.y - point_1.position.y) / range;
    double z__ = (point_2.position.z - point_1.position.z) / range;

    while (ros::ok()) {
        pose_data.poses.clear();
        pose_data.poses.push_back(point_1);
        temp=point_1;
        for(int i=0 ; i<= range ; i++)
        {
            temp.position.x += x__;
            temp.position.y += y__;
            temp.position.z += z__;
            pose_data.poses.push_back(temp);
        }

        data_pub.publish(pose_data);
        loop.sleep();
    }
}