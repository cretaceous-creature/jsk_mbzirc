/*
 jsk_mbzirc_task
 */

// Author: Chen

#include <ros/ros.h>

//msg headers.
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <jsk_mbzirc_msgs/ProjectionMatrix.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
//sys lib
#include <iostream>
#include <string>
//srv
#include <std_srvs/Empty.h>
#include <gazebo_msgs/GetModelState.h>

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <ctime>




class uav_move
{

private:

    ros::NodeHandle nh_;
    //subscriber
    ros::Subscriber aim_pose_sub_;
    ros::Subscriber uav_odom_sub_;
    ros::Subscriber p_matrix_sub_;
    //ros::Subscriber pick_state;
    //publisher
    ros::Publisher cmd_pub_;

    //data field
    nav_msgs::Odometry uav_odom;
    geometry_msgs::Pose aim_pose;
    geometry_msgs::Pose camera_center_coords;
    const float obj_plane_ = 0.2;//1 meter high
    //cmd_vel
    geometry_msgs::Twist obj_vel;
    geometry_msgs::Twist vel_world_uav;
    geometry_msgs::Twist vel_cmd_uav;
    geometry_msgs::Twist diff_twist, d_diff_twist, i_diff_twist;

public:
    void init()
    {
        //publish pointcloud msgs:
        std::string topic_cmd = nh_.resolveName("/cmd_vel");
        float Kp = 0.2;
        float Kd = 0.5;
        float Ki = 0.0001;
        nh_.setParam("Kp",Kp);
        nh_.setParam("Kd",Kd);
        nh_.setParam("Kd",Ki);

        //subscriber
        aim_pose_sub_ = nh_.subscribe("/aimpose",
                                         1,&uav_move::AimPoseCallback,this);
        uav_odom_sub_ = nh_.subscribe("/ground_truth/state",
                                      10,&uav_move::OdomCallback,this);
        p_matrix_sub_ = nh_.subscribe("/projection_matrix",
                                      1,&uav_move::ProjectionMatrixCallback,this);
        //publisher  publisher the velocity
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    }


    void AimPoseCallback(const geometry_msgs::Pose aimpose)
    {
        //update the aim_pose
        //if the aimpose is very different with the last one, reservew
//        if(aim_pose_vector.poses.size())
//        {
//            if(abs())
//        }
        obj_vel.linear.x = aimpose.position.x - aim_pose.position.x;
        obj_vel.linear.y = aimpose.position.y - aim_pose.position.y;
        obj_vel.linear.z = aimpose.position.z - aim_pose.position.z; //z is not necessary for following
        aim_pose = aimpose; //renew the aimpose..

        //get the twist
        geometry_msgs::Twist diff;
        diff.linear.x = aim_pose.position.x - camera_center_coords.position.x;
        diff.linear.y = aim_pose.position.y - camera_center_coords.position.y;
        diff.linear.z = aim_pose.position.z - camera_center_coords.position.z;

        //delta diff twist...
        d_diff_twist.linear.x = diff.linear.x - diff_twist.linear.x;
        d_diff_twist.linear.y = diff.linear.y - diff_twist.linear.y;
        d_diff_twist.linear.z = diff.linear.z - diff_twist.linear.z;
        //intergrate twist
        i_diff_twist.linear.x += diff.linear.x;
        i_diff_twist.linear.y += diff.linear.y;
        i_diff_twist.linear.z += diff.linear.z;
        i_diff_twist.linear.x = i_diff_twist.linear.x>500.0?500.0:i_diff_twist.linear.x;
        i_diff_twist.linear.y = i_diff_twist.linear.y>500.0?500.0:i_diff_twist.linear.y;
        i_diff_twist.linear.z = i_diff_twist.linear.z>50.0?50.0:i_diff_twist.linear.z;

        //renew the diff twist
        diff_twist = diff;
    }
    void ProjectionMatrixCallback(
            const jsk_mbzirc_msgs::ProjectionMatrix projection_matrix)
    {

            float A[2][2];
            float bv[2];
            int i = 240;
            int j = 320;
            A[0][0] = j * projection_matrix.data.at(8) -
                    projection_matrix.data.at(0);
            A[0][1] = j * projection_matrix.data.at(9) -
                    projection_matrix.data.at(1);
            A[1][0] = i * projection_matrix.data.at(8) -
                    projection_matrix.data.at(4);
            A[1][1] = i * projection_matrix.data.at(9) -
                    projection_matrix.data.at(5);
            bv[0] = projection_matrix.data.at(2)*obj_plane_  +
                    projection_matrix.data.at(3) - j*projection_matrix.data.at(
                        10)*obj_plane_  - j*projection_matrix.data.at(11);
            bv[1] = projection_matrix.data.at(4)*obj_plane_  +
                    projection_matrix.data.at(7) - i*projection_matrix.data.at(
                        10)*obj_plane_ - i*projection_matrix.data.at(11);
            float dominator = A[1][1] * A[0][0] - A[0][1] * A[1][0];

            camera_center_coords.position.x = (A[1][1]*bv[0]-A[0][1]*bv[1]) / dominator;
            camera_center_coords.position.y = (A[0][0]*bv[1]-A[1][0]*bv[0]) / dominator;
            camera_center_coords.position.z = this->obj_plane_;
    }

    void OdomCallback(const nav_msgs::Odometry odom)
    {
        uav_odom = odom;
        //PID control the diff
        vel_world_uav = obj_vel;  //send the uav to the same speed of the truck.
        float Kp, Kd, Ki;
        //get param
        nh_.getParam("Kp",Kp);
        nh_.getParam("Kd",Kd);
        nh_.getParam("Kd",Ki);
        //diff_twist need to be 0,0,0   d_diff_twist is the differential of diff_twist
        vel_world_uav.linear.x += Kp * diff_twist.linear.x - Kd * d_diff_twist.linear.x
                +Ki * i_diff_twist.linear.x;
        vel_world_uav.linear.y += Kp * diff_twist.linear.y - Kd * d_diff_twist.linear.y
                +Ki * i_diff_twist.linear.y;
        //disable z control
  //        vel_cmd_uav.linear.z += Kp * diff_twist.linear.z - Kd * d_diff_twist.linear.z;

        vel_world_uav.linear.x = vel_world_uav.linear.x>4.0?4.0:vel_world_uav.linear.x;
        vel_world_uav.linear.y = vel_world_uav.linear.y>4.0?4.0:vel_world_uav.linear.y;
        vel_world_uav.linear.z = vel_world_uav.linear.z>1.0?1.0:vel_world_uav.linear.z;

        vel_world_uav.linear.x = vel_world_uav.linear.x<-4.0?-4.0:vel_world_uav.linear.x;
        vel_world_uav.linear.y = vel_world_uav.linear.y<-4.0?-4.0:vel_world_uav.linear.y;
        vel_world_uav.linear.z = vel_world_uav.linear.z<-1.0?-1.0:vel_world_uav.linear.z;

        vel_world_uav.linear.z = 0;

        tf::Pose tfpose;
        tf::Vector3 tftwist;
        //transfer world vel to cmd vel..
        tf::poseMsgToTF(odom.pose.pose,tfpose);
        tf::vector3MsgToTF(vel_world_uav.linear,tftwist);
        //tf::vector3MsgToTF(odom.twist.twist.linear,tftwist);
        tf::Quaternion rot = tfpose.getRotation().inverse();
        tf::Vector3 axis = rot.getAxis();
        tfScalar angle = rot.getAngle();
        tftwist = tftwist.rotate(axis,angle);
        ROS_WARN("speed is %f,%f,%f",tftwist.getX(),tftwist.getY(),
                 tftwist.getZ());
        vel_cmd_uav.linear.x = tftwist.getX();
        vel_cmd_uav.linear.y = tftwist.getY();
        vel_cmd_uav.linear.z = tftwist.getZ();
        cmd_pub_.publish(vel_cmd_uav);


    }

    ~uav_move()
    {    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_move");
    uav_move t_p;
    t_p.init();

    ros::spin();

}
