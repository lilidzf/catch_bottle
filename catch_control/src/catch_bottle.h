//
// Created by zh on 18-11-3.
//

#ifndef PROJECT_CATCH_BOTTLE_H
#define PROJECT_CATCH_BOTTLE_H


#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include<sensor_msgs/JointState.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <fstream>
#include <iostream>

#include "robotiq_85_msgs/GripperCmd.h"
#include "robotiq_85_msgs/GripperStat.h"

#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <geometry_msgs/PoseArray.h>


class UR{

    typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  Client;

public:
    geometry_msgs::PoseArray poseData;
    UR(ros::NodeHandle _nh, const std::string & _urdf_param, const std::string & _chain_start, const std::string & _chain_end, double _timeout,std::string UR_name_prefix, std::string hand_name_prefix){
        double eps = 1e-5;
        nh_ = _nh;
        p_tracik_solver_ = new TRAC_IK::TRAC_IK(_chain_start, _chain_end, _urdf_param, _timeout, eps); //反解
        KDL::Chain chain;
        bool valid = p_tracik_solver_->getKDLChain(chain);
        p_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain);  //正解
        //init
        joint_names_.push_back("shoulder_pan_joint");
        joint_names_.push_back("shoulder_lift_joint");
        joint_names_.push_back("elbow_joint");
        joint_names_.push_back("wrist_1_joint");
        joint_names_.push_back("wrist_2_joint");
        joint_names_.push_back("wrist_3_joint");
        current_JntArr_.resize(6);
        for(int i=0;i<6;++i)
        {
            if (i==1||i==3)
                current_JntArr_(i) = -M_PI/2;
            else if(i==0)
                current_JntArr_(i) = M_PI;
            else
                current_JntArr_(i) = 0;
        }
        bsub_ = false;
        bstart_cal_ = false;

        sub_=nh_.subscribe(UR_name_prefix+"/joint_states", 1, &UR::subJointStatesCB, this);
       // sub_hand_ = nh_.subscribe("gripper/stat",1 , &UR::subHandStatesCB,this);
       // pub_hand_cmd_ = nh_.advertise<robotiq_85_msgs::GripperCmd>("gripper/cmd",1);
        pose_data_ = nh_.subscribe("pose_data",1000,&UR::poseArrayCB,this);


        //sub_markers_ = nh_.subscribe("vis_markers", 1, &UR::subMotionCaptureCB, this);

        ROS_INFO("sub done");

        //action client
//        client_ = new Client(UR_name_prefix+"/follow_joint_trajectory", true);
//        client_->waitForServer(ros::Duration());
        // servoj client

        client_servoj_ = new Client("pos_based_pos_traj_controller/follow_joint_trajectory",true);
        ROS_INFO("wait for servoj client");
        client_servoj_->waitForServer(ros::Duration());
        ROS_INFO("servoj clent connect!");
    }

    ~UR(){
        delete p_tracik_solver_;
        delete client_;
        delete client_servoj_;
    }
    void curren_frame(){
        for(int i = 0; i < 3; i++){
            ROS_INFO_STREAM(frame_wrist3_base_.p.data[i]);
        }
    }
    void moveTO(double x, double y, double z,double time){
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(bur_sub_==false);
        KDL::Frame end_frame;
        end_frame.M.RPY(0,0,0);
        end_frame.p.data[0] = x;
        end_frame.p.data[1] = y;
        end_frame.p.data[2] = z;
        KDL::JntArray start_Jnt = current_JntArr_;
        KDL::JntArray end_Jnt;
        p_tracik_solver_->CartToJnt(current_JntArr_, end_frame, end_Jnt);
        std::vector<KDL::JntArray> result_array;
        KDL::JntArray delta_array(6);
        std::vector<int> sizeArr;
        delta_array(0) = end_Jnt(0) - start_Jnt(0);
        for(int i = 0; i < 6; ++i){
            delta_array(i) = end_Jnt(i) - start_Jnt(i);
            int dsize = 1+int(std::abs(delta_array(i)*180/M_PI));
            sizeArr.push_back((dsize));
        }
        int c = 0;
        for(int i = 0; i < sizeArr.size(); ++i){
            c=(sizeArr[i]>c)?sizeArr[i]:c;
        }
        double per_time;
        if( c > 0 )
            per_time = time / c;
        else
            per_time = time;
        for(int i = 0; i < c; ++i){
            KDL::JntArray tmp(6);
            for (int x = 0; x < 6; ++x)
            {
                tmp(x) = start_Jnt(x) + i * delta_array(x) / (c-1);
            }

            moveto_joints(tmp,per_time);
//            result_array.push_back(tmp);
        }
    }
    void draw_straight(double x, double y, double z, double time){
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(bur_sub_==false);
        KDL::Frame end_frame;
        end_frame.M.RPY(0,0,0);
        end_frame.p.data[0] = x;
        end_frame.p.data[1] = y;
        end_frame.p.data[2] = z;
        KDL::JntArray start_Jnt = current_JntArr_;
        KDL::Frame start_frame = frame_wrist3_base_;
        KDL::JntArray end_Jnt;
        p_tracik_solver_->CartToJnt(start_Jnt, end_frame, end_Jnt);
        std::vector<KDL::JntArray> result_array;
        std::ofstream outFile;
        std::ostringstream oss;
        int name_suffix;
        std::string filename = oss.str() + "_Joint_ARR_1";
        std::string file = "/home/dzf/catch_bottle-master" + filename + ".txt";
        outFile.open(file.c_str(), std::ios::app);
        int range = 1000;
        double x_ = (end_frame.p.data[0] - start_frame.p.data[0])/range;
        double y_ = (end_frame.p.data[1] - start_frame.p.data[1])/range;
        double z_ = (end_frame.p.data[2] - start_frame.p.data[2])/range;
        double per_time;
        per_time = time / range;
        for(int i = 0; i < range; ++i) {
            KDL::JntArray tmp(6);
            start_frame.p.data[0] += x_;
            start_frame.p.data[1] += y_;
            start_frame.p.data[2] += z_;
            p_tracik_solver_->CartToJnt(start_Jnt, start_frame, tmp);
            outFile <<tmp(0)<<" "<<tmp(1)<<" "<<tmp(2)<<" "<< tmp(3)<<" "<< tmp(4)<<" "<<tmp(5)<<" "
            <<start_frame.p.data[0]<<" "<<start_frame.p.data[1]<<" "<<start_frame.p.data[2]<< "\n";
            moveto_joints(tmp, per_time);
            start_Jnt = tmp;
        }
        outFile.close();
    }




    void moveto_target(double Ax, double Ay, double Az){
          KDL::Frame target_pose;
          target_pose.M.RPY(0,0,0);
          target_pose.p.data[0] = Ax;
          target_pose.p.data[1] = Ay;
          target_pose.p.data[2] = Az;
          KDL::JntArray result;
          p_tracik_solver_->CartToJnt(current_JntArr_, target_pose, result);
          moveto_joints(result,5);
//        ROS_INFO("DRAW STRASIGHT FROM A TO B");
//        ros::Time start_time = ros::Time::now();
//        force_update_all_state();
//        KDL::Chain chain;
//        bool vaild = p_tracik_solver_->getKDLChain(chain);
//        uint nbr = chain.getNrOfJoints();//关节数
//        KDL::JntArray p(nbr);
//        p = current_JntArr_;
//        KDL::Frame frame_wrist3_base;
//        p_fk_solver_->JntToCart(current_JntArr_,frame_wrist3_base);
//
//        for(int i = 0; i < nbr ; ++i)
//        {
//            ROS_INFO_STREAM("name: " << chain.getSegment(i).getJoint().getName() << " pos: " << p(i) << "\n");
//        }
//        int range = 100;
//        KDL::Frame next_pose;
//
//        double x_ = (Ax - frame_wrist3_base.p.data[0])/range;
//        double y_ = (Ay - frame_wrist3_base.p.data[1])/range;
//        double z_ = (Az - frame_wrist3_base.p.data[2])/range;
//        next_pose.p.data[0] = frame_wrist3_base.p.data[0];
//        next_pose.p.data[1] = frame_wrist3_base.p.data[1];
//        next_pose.p.data[2] = frame_wrist3_base.p.data[2];
//        std::vector<KDL::JntArray> vResult;
//        KDL::JntArray result;
////        std::ofstream outFile;
////        std::ostringstream oss;
////        int name_suffix;
////        std::string filename = oss.str() + "_Joint_ARR";
////        std::string file = "/home/dzf/catch_bottle-master" + filename + ".txt";
////        outFile.open(file.c_str(), std::ios::app);
//        for(int i = 0; i < range; ++i )
//        {
//            next_pose.p.data[0] += x_;
//            next_pose.p.data[1] += y_;
//            next_pose.p.data[2] += z_;
////         next_pose.M = KDL::Rotation::RPY(0,0,i*M_PI/range);
//            p_tracik_solver_->CartToJnt(p, next_pose, result);
//            p = result;
////            outFile <<result(0)<<" "<<result(1)<<" "<<result(2)<<" "<< result(3)<<" "<< result(4)<<" "<<
////            result(5)<<" "<<next_pose.p.data[0]<<" "<<next_pose.p.data[1]<<" "<<next_pose.p.data[2]<< "\n";
//            vResult.push_back(result);
//        }
////        outFile.close();
//        control_msgs::FollowJointTrajectoryGoal g;
//        //head
//        g.trajectory.header.stamp = ros::Time::now();
//        //names
//        for(int i=0;i<nbr;++i)
//        {
//            std::string name = chain.getSegment(i).getJoint().getName();
//            g.trajectory.joint_names.push_back(name);
//            ROS_INFO_STREAM("joint name : "<<name);
//        }
//
//        //loop result
//        KDL::JntArray jntArr;
//        double dur = 0;
//        for(int i=0;i<vResult.size();++i)
//        {
//            jntArr = vResult[i];
//            //pt
//            trajectory_msgs::JointTrajectoryPoint pt;
//            int n=jntArr.rows()*jntArr.columns();
//            for(int x=0;x<n;++x)
//            {
//                pt.positions.push_back(jntArr(x));
//                pt.velocities.push_back(0);
//            }
//
//            pt.time_from_start = ros::Duration(dur);
//            dur += 0.05;
//            g.trajectory.points.push_back(pt);
////            int num;
////            num = i % 10;
////
////            while (num == 0) {
//////            ROS_INFO_STREAM(dur);
//////            ROS_INFO_STREAM(pt);
////            num ++;
////            }
//        }
//        ROS_INFO_STREAM("end_time_sed goal before "<<ros::Time::now() - start_time);
//        client_->sendGoal(g);
//        ROS_INFO_STREAM("end_time middle send "<<ros::Time::now() - start_time);
////        client_->waitForResult(ros::Duration());
//        ROS_INFO_STREAM("end_time after send "<<ros::Time::now() - start_time);
    }






    void draw_sin(bool bLeftArm)
    {
        int dir = bLeftArm?-1:1;
        ROS_INFO("draw sin left");
        KDL::Chain chain;
        bool valid = p_tracik_solver_->getKDLChain(chain);

        //start jntArr
        uint nbr = chain.getNrOfJoints();
        KDL::JntArray p(nbr);
        p = current_JntArr_;

//   }
        for(int i=0;i<nbr;++i)
        {
            ROS_INFO_STREAM("name: "<<chain.getSegment(i).getJoint().getName()<<"   pos: "<<p(i)<<"\n");
        }

        //ee pose
        KDL::Frame end_effector_pose;
        end_effector_pose.M.data[0] = 1;
        end_effector_pose.M.data[1] = 0;
        end_effector_pose.M.data[2] = 0;
        end_effector_pose.M.data[3] = 0;
        end_effector_pose.M.data[4] = 1;
        end_effector_pose.M.data[5] = 0;
        end_effector_pose.M.data[6] = 0;
        end_effector_pose.M.data[7] = 0;
        end_effector_pose.M.data[8] = 1;
        

        //cal
        std::vector<KDL::JntArray>  vResult;
        KDL::JntArray result;
        int i=0, range=50;
        for(i=-range;i<range;++i)
        {
            //     ROS_INFO("Pt %d ", i);
            end_effector_pose.p.data[0] = dir*(i*M_PI/(30*range)+0.1);
            end_effector_pose.p.data[1] = 0.325;
            end_effector_pose.p.data[2] = 0.05*sin(i*M_PI/range)+0.435;
//            end_effector_pose.M.RPY(0,0,i*M_PI/(30*range)+0.1);
            end_effector_pose.M = KDL::Rotation::RPY(0,0,i*M_PI/range);
            for(unsigned int i=0;i<9;i++)
                ROS_INFO_STREAM(end_effector_pose.M.data[i]);
            p_tracik_solver_->CartToJnt(p, end_effector_pose, result);
            p = result;
            vResult.push_back(result);
        }
        //reverse
        for(i=range;i>-range;--i)
        {
            //     ROS_INFO("Pt %d ", i);
            end_effector_pose.p.data[0] = dir*(i*M_PI/(30*range)+0.1);
            end_effector_pose.p.data[1] = 0.325;
            end_effector_pose.p.data[2] = -0.05*sin(i*M_PI/range)+0.435;
            end_effector_pose.M = KDL::Rotation::RPY(0,0,i*M_PI/range);
            //根据end_effector_pose逆解获取关节
            p_tracik_solver_->CartToJnt(p, end_effector_pose, result);
            p = result;
            vResult.push_back(result);

        }
        //规划轨迹为sin的运动，轨迹分成30分分别

        //trajectory goal
        control_msgs::FollowJointTrajectoryGoal g;
        //head
//    g.trajectory.header.frame_id = "base_link";
        g.trajectory.header.stamp = ros::Time::now();
        //names
        for(i=0;i<nbr;++i)
        {
            std::string name = chain.getSegment(i).getJoint().getName();
            g.trajectory.joint_names.push_back(name);
            ROS_INFO_STREAM("joint name : "<<name);
        }

        //loop result
        KDL::JntArray jntArr;
        double dur = 5.0;
        for(i=0;i<vResult.size();++i)
        {
            jntArr = vResult[i];
            //pt
            trajectory_msgs::JointTrajectoryPoint pt;
            int n=jntArr.rows()*jntArr.columns();
            for(int x=0;x<n;++x)
            {
                pt.positions.push_back(jntArr(x));
                pt.velocities.push_back(0);
            }

            pt.time_from_start = ros::Duration(dur);
            dur += 0.05;
            g.trajectory.points.push_back(pt);
//            ROS_INFO_STREAM("  joint pos 1: "<<pt.positions[0]<<" joint pos 2: "<<pt.positions[1]<<" joint pos 3: "<<pt.positions[2]<<"  joint pos 4: "<<pt.positions[3]<<"  joint pos 5: "<<pt.positions[4]<<"  joint pos 6: "<<pt.positions[5]);
//      break;//test
        }

        client_->sendGoal(g);
        client_->waitForResult(ros::Duration());

    }



    void moveto_joints(KDL::JntArray target_jnt, double time){
//        ROS_INFO("move to joints");
        //运行前需要ros::spinOnce 来更新当前关节角
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(bur_sub_==false);

        trajectory_msgs::JointTrajectoryPoint p0;
        trajectory_msgs::JointTrajectoryPoint p1;
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        g.trajectory.joint_names.push_back("shoulder_pan_joint");
        g.trajectory.joint_names.push_back("shoulder_lift_joint");
        g.trajectory.joint_names.push_back("elbow_joint");
        g.trajectory.joint_names.push_back("wrist_1_joint");
        g.trajectory.joint_names.push_back("wrist_2_joint");
        g.trajectory.joint_names.push_back("wrist_3_joint");

        for(int x=0;x<6;++x)
        {
            p0.positions.push_back(current_JntArr_(x));
            p0.velocities.push_back(0);
        }
        p0.time_from_start = ros::Duration(0);
        g.trajectory.points.push_back(p0);

        for(int x=0;x<6;++x)
        {
            p1.positions.push_back(target_jnt(x));
            p1.velocities.push_back(0);
        }
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_->sendGoal(g);
        client_->waitForResult(ros::Duration());

    }

    void servoj_sin_test(){
        std::vector<double> gravity_down_joints;
        nh_.getParam("gravity_down_joints", gravity_down_joints);
        KDL::JntArray gravity_down_jnt(6);
        for (int i = 0; i < 6; ++i) {
            gravity_down_jnt(i) = gravity_down_joints[i] * M_PI / 180;
        }
        ROS_INFO_STREAM("t0 ");
        servoj_moveto(gravity_down_jnt, 5, true);

        ROS_INFO_STREAM("t1 ");
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(!bur_sub_);
        KDL::Frame original_P = frame_wrist3_base_;

        ros::Rate loop_rate(500);
        uint64_t nsec0 = ros::Time::now().toNSec();
        while(ros::ok()){
            ros::spinOnce();

            uint64_t time_now = ros::Time::now().toNSec() - nsec0;
            double time_now_d = time_now / 1e9;
            KDL::Frame p1 = original_P;
            KDL::JntArray q1;
            q1.resize(6);
            p1.p.data[2] = original_P.p.data[2]+ 0.05*sin(2*M_PI/5*time_now_d);
            p_tracik_solver_->CartToJnt(current_JntArr_,p1,q1);
            servoj_moveto(q1,0.002,false);

            //record
            std::ofstream file3;
            std::ostringstream oss;
            int name_suffix = 1;
            oss << name_suffix;
            std::string filename = oss.str() + "_servoj_tracking_real_robot";
            std::string file = "/home/zh/catkin_ws/src/catch_control/record_servoj_sim/" + filename + ".txt";
            file3.open(file.c_str(), std::ios::app);
            file3 << time_now_d << " ";
            file3 << p1.p.data[0]<< " " <<p1.p.data[1] << " " << p1.p.data[2]<< " "
                  << end_point_.data[0]<< " " << end_point_.data[1] << " " <<end_point_.data[2];
            file3 << std::endl;
            file3.close();
            loop_rate.sleep();
            if (time_now_d>20)
                break;
        }



    }

    void servoj_moveto(KDL::JntArray target, double time,bool wait_for_D) {
        trajectory_msgs::JointTrajectoryPoint p0;
        trajectory_msgs::JointTrajectoryPoint p1;
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        g.trajectory.joint_names.push_back("shoulder_pan_joint");
        g.trajectory.joint_names.push_back("shoulder_lift_joint");
        g.trajectory.joint_names.push_back("elbow_joint");
        g.trajectory.joint_names.push_back("wrist_1_joint");
        g.trajectory.joint_names.push_back("wrist_2_joint");
        g.trajectory.joint_names.push_back("wrist_3_joint");

        for(int x=0;x<6;++x)
        {
            p0.positions.push_back(current_JntArr_(x));
            p0.velocities.push_back(0);
        }
        p0.time_from_start = ros::Duration(0);
//        g.trajectory.points.push_back(p0);

        for(int x=0;x<6;++x)
        {
            p1.positions.push_back(target(x));
            p1.velocities.push_back(0);
        }
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_servoj_->sendGoal(g);
        if(wait_for_D)
            ros::Duration(time).sleep();

    }

// control Robotiq in [mm]
    void robotiq_hand_move(float position,float vel, float force){
        ROS_INFO_STREAM("hand move, position="<<position<<"  vel="<<vel<<"  force="<<force);
        robotiq_85_msgs::GripperCmd hand_cmd;
        position = position/1000;
        vel = vel/1000;
        force = force/1000;
        if (position < 0 || position>0.086 || vel>0.02 ||vel<-0.02 || force>255 ||force<0){
            ROS_ERROR("The command of the Robotiq is out of range");
            return;
        }
        force_update_all_state();
        if(true ){
            hand_cmd.position = position;   // scale 0-0.085mm
            hand_cmd.speed = vel;
            hand_cmd.force = force;
            pub_hand_cmd_.publish(hand_cmd);

        }

    }

    void catch_bottle(){
        ROS_INFO_STREAM("begin to catch the bottle.");
        robotiq_hand_move(85,10,50);   // open hand
        move_to_xyz(-0.059, 0.341, 0.433, 3, false);//home点
        //竖直的瓶子
        do{
        force_update_all_state();
        }while(check_markers_fine(first_body_markers_[0]));

        KDL::Vector A0 = first_body_markers_[0];  //Motion capture

        KDL::Frame bottle_target;
        KDL::JntArray bottle_target_jnt(6);
        bottle_target.M = KDL::Rotation::RPY(-M_PI/2,-M_PI/2,0);  //base_link的z轴旋转90度
        bottle_target.p = A0 + KDL::Vector(0,-0.1-0.505,-0.13);


        p_tracik_solver_->CartToJnt(current_JntArr_,bottle_target,bottle_target_jnt);
        moveto_joints(bottle_target_jnt,3);
        move_to_xyz(0, 0.05, 0, 2, true);
        ros::Duration(3).sleep();
        do{
            force_update_all_state();
        }while(check_markers_fine(first_body_markers_[0]));
        double bottle_move = 0.03;


        while (bottle_move>0.02){
            robotiq_hand_move(85,10,50);   // open hand
            do{
                force_update_all_state();
            }while(check_markers_fine(first_body_markers_[0]));
            KDL::Vector A1 = first_body_markers_[0];
            A1 = first_body_markers_[0];
            bottle_move = sqrt(pow(A1.data[0]-A0.data[0],2)+  pow(A1.data[1]-A0.data[1],2)+pow(A1.data[2]-A0.data[2],2) );
            A0 = A1;
            move_to_xyz(0, -0.1-0.05, 0.2, 2, true);
            bottle_target.p = A1 + KDL::Vector(0,-0.15-0.05,-0.13);
            p_tracik_solver_->CartToJnt(current_JntArr_,bottle_target,bottle_target_jnt);
            moveto_joints(bottle_target_jnt,3);
            robotiq_hand_move(85,10,50);   // open hand

            move_to_xyz(0, 0.05, 0, 2, true);
            robotiq_hand_move(56,5,50);   //catch  close hand 56mm
            ros::Duration(2).sleep();
            do{
                force_update_all_state();
            }while(check_markers_fine(first_body_markers_[0]));
            A1 = first_body_markers_[0];
            bottle_move = sqrt(pow(A1.data[0]-A0.data[0],2)+  pow(A1.data[1]-A0.data[1],2)+pow(A1.data[2]-A0.data[2],2) );
        }


        //向上并运送到指定位置
        force_update_all_state();
        do{
            ros::spinOnce();
        }while(!b_hand_sub_);
        if (hand_state_.is_moving){
            //向上 200mm
            ros::Duration(1).sleep();
        }
        ros::Duration(2).sleep();
        move_to_xyz(0, 0, 0.2, 3, true);
        // 放到固定位置 -0.2064 0.402 bottle_target.data[2];
        move_to_xyz(-0.2064,0.402,bottle_target.p.data[2]+0.2, 3, false);
        //向下0.2
        move_to_xyz(0, 0, -0.2, 3, true);
        robotiq_hand_move(85,5,50);   // put
        ros::Duration(1).sleep();
        move_to_xyz(0, -0.1, 0, 2, true);
        move_to_xyz(0, 0, 0.2, 3, true);
        move_to_xyz(-0.059, 0.341, 0.433, 2, false);//home点

    }

    void catch_bottle_to_fixed_pos(){

    }

    void calibrate_motioncapture_to_UR(std::string path){

        ROS_INFO_STREAM("cal begin");
        bstart_cal_ = true;
        ros::spin();


    }

    void force_update_all_state(){
//        b_hand_sub_ = false;
        bur_sub_ = false;
//        bmotion_capture_sub_ = false;
//        b_hand_sub_ = false;
        do{
            ros::spinOnce();
        }while(!bur_sub_); //&& !bmotion_capture_sub_&&!b_hand_sub_
    }
    bool check_markers_fine(KDL::Vector point){
        if (fabs(point.data[0])<0.4&& point.data[0]>0.46&&point.data[1]< 0.85 &&point.data[1]>0.1 &&point.data[2]< 0.7)
            //绝对值 Absolute value
            return true;
        else
            return false;

    }

    void move_to_xyz(double x,double y, double z,double time,bool b_relative){
        force_update_all_state();
        KDL::Frame next_frame = frame_wrist3_base_;
        KDL::JntArray next_jnts(6);
        if(b_relative){
            ROS_INFO_STREAM("move to position relative to current point   x="<<x<<"   y="<<y<<"  z"<<z);
            next_frame.p.data[0] += x;
            next_frame.p.data[1] += y;
            next_frame.p.data[2] += z;

        } else{
            ROS_INFO_STREAM("move to fixed position     x="<<x<<"   y="<<y<<"  z"<<z);
            next_frame.p.data[0] = x;
            next_frame.p.data[1] = y;
            next_frame.p.data[2] = z;
        }
        p_tracik_solver_->CartToJnt(current_JntArr_,next_frame,next_jnts);
        moveto_joints(next_jnts,time);
    }



private:
    void poseArrayCB(geometry_msgs::PoseArray msg)
    {
        poseData = msg;
    }

    void subMotionCaptureCB(visualization_msgs::MarkerArray state) {
//        ROS_INFO_STREAM("size"<<state.markers.size());
        first_body_markers_.clear();
        std::vector<KDL::Vector> first_body_markers_in_nokov ;

        std::vector<KDL::Vector> hole_markers ;
        std::vector<geometry_msgs::Point> markers_position;

        for (int i = 0; i < state.markers.size(); ++i) {
            markers_position.push_back(state.markers.at(i).pose.position);
            std::string hole_tmp = "m1";  // first body name
            if (state.markers.at(i).ns.substr(0,2)==hole_tmp) {
                KDL::Vector m_tmp  = KDL::Vector(markers_position[i].x,markers_position[i].y,markers_position[i].z);
                hole_markers.push_back(m_tmp);
                first_body_markers_in_nokov.push_back(m_tmp);
            }
        }

        // start calibration with three markers in the end-effector ,A2A0 is x-axis, A0A1 is y-axis.
        if (bstart_cal_) {
            KDL::Frame T_p2m, T_p2b, T_m2b;
            //求 P坐标系到基座坐标系变换
            KDL::Frame T_p2tool0;
            double offset_tmp1 = 0.025 * sqrt(2) / 2;
            T_p2tool0.M = KDL::Rotation::Identity();
            T_p2tool0.p = KDL::Vector(offset_tmp1, -offset_tmp1, 0.006);
            T_p2b = frame_wrist3_base_ * T_p2tool0;

            // 求P坐标系到MotionCapture变换
            KDL::Vector Px = hole_markers[0] - hole_markers[2];
            Px.Normalize();
            KDL::Vector Py = hole_markers[1] - hole_markers[0];
            Py.Normalize();
            Eigen::Vector3d Py_, Pz_, Px_;
            for (int j = 0; j < 3; ++j) {
                Px_(j) = Px[j];
                Py_(j) = Py[j];
            }
            Pz_ = Px_.cross(Py_);   //内积
            Pz_.normalize();
            Py_ = Pz_.cross(Px_);
            T_p2m.p = hole_markers[0];
            for (int k = 0; k < 3; ++k) {
                T_p2m.M(k, 0) = Px_(k);
                T_p2m.M(k, 1) = Py_(k);
                T_p2m.M(k, 2) = Pz_(k);
            }

            T_m2b = T_p2b * T_p2m.Inverse();

            double tmp_x, tmp_y, tmp_z, tmp_w;
            T_m2b.M.GetQuaternion(tmp_x, tmp_y, tmp_z, tmp_w);

            ROS_INFO_STREAM("T_m2b .p " << T_m2b.p.data[0] << " " << T_m2b.p.data[1] << " " << T_m2b.p.data[2] << " ");
            ROS_INFO_STREAM("T_m2b .Qua " << tmp_x << " " << tmp_y << " " << tmp_z << " " << tmp_w);
        } else{
            KDL::Frame T_m2b;
            T_m2b.p = KDL::Vector(-0.452872, 0.331446, -0.0393143);
            T_m2b.M = KDL::Rotation::Quaternion(0.00222333, -0.00330297, 0.00157233, 0.999991);
            for (int i = 0; i < first_body_markers_in_nokov.size(); ++i) {
                KDL::Vector tmp1 = T_m2b.M*first_body_markers_in_nokov[i] + T_m2b.p;   // 点在bask_link下的坐标
                first_body_markers_.push_back(tmp1);
            }
        }

        bmotion_capture_sub_ = true;
    }

    void subHandStatesCB(robotiq_85_msgs::GripperStat state){
        hand_state_ = state;
        b_hand_sub_ = true;
    }

    void subJointStatesCB(sensor_msgs::JointState state)
    {
        KDL::JntArray jntArr;
        KDL::JntArray jntSpeed;
        KDL::JntArray jntCur;
        jntArr.resize(6);
        jntSpeed.resize(6);
        jntCur.resize(6);
        int n = state.name.size();
        for (int i = 0; i < 6; ++i)//joint_names_
        {
            int x = 0;
            for (; x < n; ++x)//state
            {
                if (state.name[x] == ( joint_names_[i])) {
                    jntArr(i) = state.position[x];
                    jntSpeed(i) = state.velocity[x];
                    jntCur(i) = state.effort[x];
                    break;
                }
            }

            if (x == n) {
                ROS_ERROR_STREAM("Error,  joint name : " << joint_names_[i] << " , not found.  ");
                return;
            }
        }
        current_JntArr_ = jntArr;
//      current_JntSpeed = jntSpeed;
//      current_JntCur_ = jntCur;

        //fk
        p_fk_solver_->JntToCart(current_JntArr_, frame_wrist3_base_);    //frame_wrist3_base_ 为正运动学计算出的位姿
        end_point_.data[0] = frame_wrist3_base_.p.data[0];
        end_point_.data[1] = frame_wrist3_base_.p.data[1];
        end_point_.data[2] = frame_wrist3_base_.p.data[2];
        bur_sub_ = true;
    }

private:
    ros::NodeHandle nh_;
    TRAC_IK::TRAC_IK *p_tracik_solver_;
    KDL::ChainFkSolverPos_recursive *p_fk_solver_;
    ros::Subscriber sub_;   //UR5e sub
    KDL::JntArray current_JntArr_;
    Client *client_;
    Client *client_servoj_;
    bool bsub_;
    bool bur_sub_;
    std::vector<std::string> joint_names_;
    KDL::Frame frame_wrist3_base_;//fk result
    KDL::Vector end_point_;

    // Robotiq Hand
    ros::Subscriber sub_hand_;
    ros::Publisher pub_hand_cmd_;
    ros::Subscriber pose_data_;

    //Motion capture
    ros::Subscriber sub_markers_;
    bool bmotion_capture_sub_;
    bool bstart_cal_;
    std::vector<KDL::Vector> first_body_markers_ ;



    robotiq_85_msgs::GripperStat hand_state_;
    bool b_hand_sub_;
};








#endif //PROJECT_CATCH_BOTTLE_H
