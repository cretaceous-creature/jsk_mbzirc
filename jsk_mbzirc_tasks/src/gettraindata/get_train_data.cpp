/*
 jsk_mbzirc_task
 */

// Author: Chen

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
//msg headers.
#include <gazebo_msgs/ModelStates.h>  //this stores all the model states of the objects..
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <string>
//srv
#include <std_srvs/Empty.h>
#include <gazebo_msgs/GetModelState.h>
//for test
#include <tf/transform_broadcaster.h>
//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <ctime>


typedef struct
{
    std::string str;
    std::vector<int> index;
}ModelString;

class get_train_data
{

private:

    ros::NodeHandle nh_;
    ros::Subscriber img_sub_;
    ros::Subscriber gazebo_model_sub_;

    ros::Publisher gazebo_model_pub_;

    //gazebo message
    gazebo_msgs::ModelStates gazebo_models;
    gazebo_msgs::ModelState tmpstate;
    //model name string
    std::vector<ModelString> gazebo_model_types;
    //srv
    std_srvs::Empty pause_srv;
    gazebo_msgs::GetModelState model_to_get;
    ros::ServiceClient gazebo_pause;
    ros::ServiceClient gazebo_unpause;
    ros::ServiceClient gazebo_getstate;

    long frame_counter = 0;

public:
    void init()
    {
        //publish pointcloud msgs:
        std::string topic = nh_.resolveName("gettraindata");
        //for test the image
        cv::namedWindow("view");
        cv::namedWindow("view2");
        cv::startWindowThread();
        //subscriber
        img_sub_ = nh_.subscribe("/downward_cam/camera/image",1,&get_train_data::imageCallback,this);
        gazebo_model_sub_ = nh_.subscribe("/gazebo/model_states",10,&get_train_data::gazebocallback,this);
        //services
        gazebo_pause = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
        gazebo_unpause = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
        gazebo_getstate = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        //publisher
        gazebo_model_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);

        //initial the model names
        ModelString tmpstr;
        tmpstr.str = "small_cylinder";
        gazebo_model_types.push_back(tmpstr);
        tmpstr.str  = "small_rectangle";
        gazebo_model_types.push_back(tmpstr);
        tmpstr.str = "large_object";
        gazebo_model_types.push_back(tmpstr);
        tmpstr.str = "uav";
        gazebo_model_types.push_back(tmpstr);
    }
    //call back, for processing
    void imageCallback(const sensor_msgs::ImageConstPtr& img);
    void setmodelstatus(bool movemodel);

    //renew the data of the gazebo objects
    void gazebocallback(const gazebo_msgs::ModelStates gazebo_model_states);
    // class's family...
    ~get_train_data()
    {    }
};

void get_train_data::setmodelstatus(bool movemodel)
{
    //process the points for objects and projecting to image space
    for(int i = 0; i < this->gazebo_models.name.size(); i++)   //only if when data are received
    {
        bool invimodel = true;
        for(int j = 0; j < gazebo_model_types.size(); j++)
        {
            int k = gazebo_models.name.at(i).find(gazebo_model_types.at(j).str);
            if(k>=0) //if find points
            {
                gazebo_model_types.at(j).index.push_back(i);
                invimodel = false;
                break;
            }
        }
        if(invimodel)
        {

            tmpstate.model_name = gazebo_models.name.at(i);
            tmpstate.pose = gazebo_models.pose.at(i);
            tmpstate.twist = gazebo_models.twist.at(i);
            if(movemodel)
                tmpstate.pose.position.z += 1000;
            else
                tmpstate.pose.position.z -= 1000;

            gazebo_model_pub_.publish(tmpstate);
        }
    }

    for(int j = 0; j < gazebo_model_types.size(); j++)
    {
        gazebo_model_types.at(j).index.clear();
    }

}


void get_train_data::imageCallback(const sensor_msgs::ImageConstPtr& img)
{
    try
    {
        if(!(frame_counter%2))   //ordinary image   pause the physics
        {
            gazebo_pause.call(pause_srv);
            cv::imshow("view", cv_bridge::toCvShare(img,"bgr8")->image);
            setmodelstatus(true);
            this->nh_.setParam("/gazebo/gravity/z",0);
        }
        else
        {
            gazebo_pause.call(pause_srv);
            cv::imshow("view2", cv_bridge::toCvShare(img,"bgr8")->image);
            setmodelstatus(false);
            this->nh_.setParam("/gazebo/gravity/z",-9.8);
        }

        //two count one frame, two images, one input one output
        frame_counter++;
        cv::waitKey(10);
        gazebo_unpause.call(pause_srv);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }
}

void get_train_data::gazebocallback(const gazebo_msgs::ModelStates gazebo_model_states)
{
    this->gazebo_models = gazebo_model_states;   //get the models
    this->gazebo_models.name.size();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_train_data");
    get_train_data getdata;
    getdata.init();

    ros::spin();
    cv::destroyWindow("view");
    cv::destroyWindow("view2");
}
