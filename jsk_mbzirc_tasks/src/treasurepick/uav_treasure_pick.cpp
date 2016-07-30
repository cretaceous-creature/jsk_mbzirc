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
#include <std_msgs/Bool.h>
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
//the store the detected objects
typedef struct
{
    geometry_msgs::Pose pose;
    //if this is large enough, publish the pose
    //if is minus too small, remove it from the vector
    int visible_times;
}Treasure;

class treasure_pick
{

private:

    ros::NodeHandle nh_;
    //subscriber
    ros::Subscriber object_pose_sub_;
    ros::Subscriber uav_odom_sub_;
    ros::Subscriber pick_state_sub_;
    //publisher
    ros::Publisher aim_pose_pub_;
    //data field
    geometry_msgs::Pose aim_pose;
    geometry_msgs::Pose box_pose;
    geometry_msgs::Pose search_pose;
    std_msgs::Bool pick_state;
    nav_msgs::Odometry uav_odom;
    std::vector<Treasure> treasure_vec;
    const double distthreshold = 0.25;
    enum State_Machine{Picking, Placing, Searching}uav_task_state;


public:
    void init()
    {
       //subscriber
        object_pose_sub_ = nh_.subscribe("/cluster_decomposer/centroid_pose_array",
                                         1,&treasure_pick::ObjPoseCallback,this);
        uav_odom_sub_ = nh_.subscribe("/ground_truth/state",10,&treasure_pick::OdomCallback,this);
        pick_state_sub_ = nh_.subscribe("/gazebo/magnetget",1,&treasure_pick::PickCallback,this);
        aim_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("aimpose",1);
        pick_state.data = false;
        //box pose, when picked, go to the box...
        box_pose.position.x = -65;
        box_pose.position.y = 25;
        box_pose.position.z = 2.5;
        uav_task_state = Picking;

    }
    inline bool DistLessThanThre(geometry_msgs::Point P1, geometry_msgs::Point P2, double threshold)
    {
        double dist = (P1.x-P2.x)*(P1.x-P2.x)+(P1.y-P2.y)*(P1.y-P2.y)+
                (P1.z-P2.z)*(P1.z-P2.z);
        if(dist>(threshold*threshold))
            return false;
        else
            return true;
    }

    //renew the data of the gazebo objects
    void ObjPoseCallback(const geometry_msgs::PoseArray posearray)
    {
        //update the aim_pose

        if(!posearray.poses.size())
            return;

        if(!treasure_vec.size())
        {
            Treasure tmp_t;
            tmp_t.pose = posearray.poses.at(0);
            tmp_t.visible_times = 0;
            treasure_vec.push_back(tmp_t);
        }
        for(int i = 0; i < posearray.poses.size(); i++)
        {
            for(int j = 0; j < treasure_vec.size(); j++)
            {
                treasure_vec.at(j).visible_times -= 1; //every object minus 1
                if(DistLessThanThre(treasure_vec.at(j).pose.position
                                    ,posearray.poses.at(i).position
                                    ,distthreshold))
                {
                    treasure_vec.at(j).visible_times += 5; //plus 5
                    //if they are the same one, update the pose
                    treasure_vec.at(j).pose = posearray.poses.at(i);
                    treasure_vec.at(j).visible_times
                            = treasure_vec.at(j).visible_times>
                            50?50:treasure_vec.at(j).visible_times;
                    break;
                }
                //larger than threshold
                else
                {
                    if(j == treasure_vec.size()-1)//the last one
                    {
                        Treasure tmp_t;
                        tmp_t.pose = posearray.poses.at(i);
                        tmp_t.visible_times = 0;
                        treasure_vec.push_back(tmp_t);
                        break;
                    }
                }
                //remove the one that can't be seen for more than 15 times.
                if(treasure_vec.at(j).visible_times < -10)
                    treasure_vec.erase(treasure_vec.begin() + j);
                //remove the one near the box
                //-65,25,
                if((treasure_vec.at(j).pose.position.x>-70&&
                        treasure_vec.at(j).pose.position.x<-60&&
                        treasure_vec.at(j).pose.position.y>20&&
                        treasure_vec.at(j).pose.position.y<30)
                        ||(treasure_vec.at(j).pose.position.y>-0.1&&
                        treasure_vec.at(j).pose.position.y<0.1))
                    treasure_vec.erase(treasure_vec.begin() + j);
            }
        }
        //publish the pose, maybe we should use the same one, now use the first one
        for(int i=0;i<treasure_vec.size();i++)
        {
            if(treasure_vec.at(i).visible_times>30)
            {
                aim_pose = treasure_vec.at(i).pose;
                //aim_pose_pub_.publish(aim_pose);
                uav_task_state = Picking;
                break;
            }
        }
    }
    void OdomCallback(const nav_msgs::Odometry odom)
    {
        uav_odom = odom; //update
        //publish by status of the state machine
        //can be set to a fixed frequency..
        if(uav_task_state==Placing)
            aim_pose_pub_.publish(box_pose);
        else if(uav_task_state==Searching)
            aim_pose_pub_.publish(search_pose);
        else
            aim_pose_pub_.publish(aim_pose);
    }
    void PickCallback(const std_msgs::Bool pickstate)
    {

        if(!pick_state.data&&pickstate.data) //from false to true
        {
            object_pose_sub_.shutdown(); //unsubscribe
            uav_task_state = Placing;
        }
        else if(pick_state.data&&!pickstate.data) //frome true to false
        {
            object_pose_sub_ = nh_.subscribe("/cluster_decomposer/centroid_pose_array",
                                             1,&treasure_pick::ObjPoseCallback,this);
            uav_task_state = Searching;
        }
        pick_state = pickstate;
    }

    ~treasure_pick()
    {    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_treasure_pick");
    treasure_pick t_p;
    t_p.init();

    ros::spin();

}
