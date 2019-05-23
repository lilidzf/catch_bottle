#include <ros/ros.h>
#include "catch_bottle.h"




int main(int argc, char** argv)
{
    ros::init(argc, argv, "catch_control");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();

    std::string chain_start, chain_end, urdf_param;
    double timeout;
    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("UR5e/robot_description"));
    std::string UR_name_prefix,hand_name_prefix;
    nh.getParam("UR_name_prefix", UR_name_prefix);
    nh.getParam("hand_name_prefix", hand_name_prefix);
    ROS_INFO_STREAM("UR_name_prefix: "<<UR_name_prefix<<"   hand_name_prefix:"<<hand_name_prefix );
    UR UR5e(nh, urdf_param, "base_link", "tool0", timeout, UR_name_prefix,hand_name_prefix);

    ros::Duration(0.5).sleep();

    ros::spinOnce();
    KDL::JntArray tmp(6);
    std::vector<double> gravity_up_joints;
    nh.getParam("gravity_up_joints", gravity_up_joints);
    for (int j = 0; j <6; ++j) {
       tmp(j) = gravity_up_joints[j]/180*M_PI;
    }
    UR5e.servoj_sin_test();

    //UR5e.moveto_target(0.3,0.4,0.8);

//    UR5e.draw_straight(-0.3,0.6,0.6,20);
//    ros::Duration(2).sleep();
//      UR5e.moveTO(0.3,0.6,0.8,10);

//    ros::Duration(5).sleep();
//    int i=0;
//    ROS_INFO_STREAM("WAITING...");
//    geometry_msgs::Pose targetPose;
//    ros::Rate loop(1);
//    while(ros::ok())
//    {
//       std::cout << "ok";
//      ros::spinOnce();
//
//       targetPose = UR5e.poseData.poses[i];
//
//       UR5e.draw_straight(targetPose.position.x,targetPose.position.y,targetPose.position.z,10);
//
//       i++;
//        if(i>10) {
//            break;
//        }
//        loop.sleep();
//    }



    return  1;


}

