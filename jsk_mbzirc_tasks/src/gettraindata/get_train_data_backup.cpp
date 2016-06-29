/*
 jsk_mbzirc_task
 */

// Author: Chen

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
//msg headers.
#include <gazebo_msgs/ModelStates.h>  //this stores all the model states of the objects..
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <string>
//srv
#include <std_srvs/Empty.h>
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
    std::vector<int> visible;
}ModelString;

typedef struct tagST_POINT {
    float x;
    float y;
} ST_POINT;

//cuda
float  process_in_cuda(double *_a, double *_b, double *_c, cv::Mat *dev_img,
                                                   pcl::PointCloud<pcl::PointXYZRGB> *PC);

class get_train_data
{
typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::Image,
    sensor_msgs::CameraInfo,
    nav_msgs::Odometry> MySyncPolicy;

private:
    ros::Publisher pointcloud_pub_;
    sensor_msgs::PointCloud2 cloud_msg;
    message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    ros::Subscriber gazebo_model_sub_;
    ros::NodeHandle nh_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    tf::Transform BaseToCamera;
    //gazebo message
    gazebo_msgs::ModelStates gazebo_models;
    //model name string
    std::vector<ModelString> gazebo_model_types;

    //srv
    std_srvs::Empty pause_srv;
    ros::ServiceClient gazebo_pause;
    ros::ServiceClient gazebo_unpause;

#define Ground_Z 0.0  //object is 0.2m high
   //test
    tf::TransformBroadcaster br;
public:
    void init()
    {
        //publish pointcloud msgs:
        std::string topic = nh_.resolveName("imagetoground");
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 1);
        //for test the image
        cv::namedWindow("view");
        cv::startWindowThread();
        //subscriber
        img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/downward_cam/camera/image",2);
        camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,"/downward_cam/camera/camera_info", 2);
        uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/ground_truth/state",2);
        gazebo_model_sub_ = nh_.subscribe("/gazebo/model_states",10,&get_train_data::gazebocallback,this);
        //services
        gazebo_pause = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
        gazebo_unpause = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");


        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(30), *img_sub_, *camera_info_sub_, *uav_odom_sub_);
        sync->registerCallback(boost::bind(&get_train_data::imageCallback,this,_1,_2,_3));

        /*
        if(!nh_.getParam("enableGPU",GPUFLAG))
            std::cout<<"fail to load the param enableGPU, Using CPU instead"<<std::endl;
        else
            std::cout<<"With GPU support flag = " << GPUFLAG<<std::endl;
            */
        //initialize base_link to camera optical link
        BaseToCamera.setOrigin(tf::Vector3(0.0,0.0,-0.2));
        BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000));

        //initial the model names
        ModelString tmpstr;
        tmpstr.str = "small_cylinder";
        gazebo_model_types.push_back(tmpstr);
        tmpstr.str  = "small_rectangle";
        gazebo_model_types.push_back(tmpstr);
        tmpstr.str = "large_object";
        gazebo_model_types.push_back(tmpstr);
    }
    //call back, for processing
    void imageCallback(const sensor_msgs::ImageConstPtr& img,
                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                       const nav_msgs::OdometryConstPtr& odom);
    //process to pointcloud
    void p2p(const sensor_msgs::ImageConstPtr& img,
             const sensor_msgs::CameraInfoConstPtr& cam_info,
             const nav_msgs::OdometryConstPtr& odom);
    //renew the data of the gazebo objects
    void gazebocallback(const gazebo_msgs::ModelStates gazebo_model_states);
    bool PtInPolygon(ST_POINT p, ST_POINT* ptPolygon, int nCount);
    // class's family...
    ~get_train_data()
    {    }
};

void get_train_data::p2p(const sensor_msgs::ImageConstPtr& img,
                             const sensor_msgs::CameraInfoConstPtr& cam_info,
                             const nav_msgs::OdometryConstPtr& odom)
{
    tf::Transform extrisic;
    cv::Mat P(3,4,CV_64FC1);
    cv::Mat P_Mat_G(3,4,CV_64FC1);
    tf::Pose tfpose;
    tfScalar extrisic_data[4*4];
    pcl::PointCloud<pcl::PointXYZRGB> Pointcloud;
    ST_POINT fov_points[4];

    Pointcloud.header.frame_id = "/world";
//    Pointcloud.height = img->height; Pointcloud.width = img->width;
//    Pointcloud.resize(img->height*img->width);
    Pointcloud.height = 2; Pointcloud.width = 2;
    Pointcloud.resize(2*2);
//    Pointcloud.is_dense = true;
    cv::Mat cvimg = cv_bridge::toCvShare(img,"bgr8")->image.clone();
    tf::poseMsgToTF(odom->pose.pose,tfpose);
    extrisic = BaseToCamera*tfpose.inverse();
    //pinv of projection matrix...
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 4; j++)
            P.at<double>(i,j) = cam_info->P.at(i*4+j);
    // however, this P is in camera coordinate..
    extrisic.getOpenGLMatrix(extrisic_data);
    cv::Mat E_MAT(4, 4, CV_64FC1, extrisic_data);
    P_Mat_G = P*(E_MAT.t());
    // now is the ground, namely, world coordinate
    double a[4], b[4], c[4];
    a[0] = P_Mat_G.at<double>(0, 0);
    a[1] = P_Mat_G.at<double>(0, 1);
    a[2] = P_Mat_G.at<double>(0, 2);
    a[3] = P_Mat_G.at<double>(0, 3);
    b[0] = P_Mat_G.at<double>(1, 0);
    b[1] = P_Mat_G.at<double>(1, 1);
    b[2] = P_Mat_G.at<double>(1, 2);
    b[3] = P_Mat_G.at<double>(1, 3);
    c[0] = P_Mat_G.at<double>(2, 0);
    c[1] = P_Mat_G.at<double>(2, 1);
    c[2] = P_Mat_G.at<double>(2, 2);
    c[3] = P_Mat_G.at<double>(2, 3);
    std::clock_t start;
    double duration;
    start = std::clock();
    
    //gpu

#if defined(GPU_EN)
    process_in_cuda(a, b, c, &cvimg, &Pointcloud);
#else
    //cpu
    int k = 0;
    for(int i=0;i<img->height;i+=(img->height-1))
        for(int j=0;j<img->width;j+=(img->width-1))
        {
            float A[2][2],bv[2];
            A[0][0] = j*c[0] - a[0]; A[0][1] = j*c[1] - a[1];
            A[1][0] = i*c[0] - b[0]; A[1][1] = i*c[1] - b[1];
            bv[0]= a[2]*Ground_Z + a[3] - j*c[2]*Ground_Z - j*c[3];
            bv[1] = b[2]*Ground_Z + b[3] - i*c[2]*Ground_Z - i*c[3];
            float DomA = A[1][1]*A[0][0]-A[0][1]*A[1][0];
            Pointcloud.points[k].x = (A[1][1]*bv[0]-A[0][1]*bv[1])/DomA;
            Pointcloud.points[k].y = (A[0][0]*bv[1]-A[1][0]*bv[0])/DomA;
            Pointcloud.points[k].z = (float)Ground_Z;
            fov_points[k].x = Pointcloud.points[k].x;
            fov_points[k].y = Pointcloud.points[k].y;

            //fill the color info
            uint8_t rgb[4];
            rgb[0] = cvimg.at<cv::Vec3b>(i,j)[0];
            rgb[1] = cvimg.at<cv::Vec3b>(i,j)[1];
            rgb[2] = cvimg.at<cv::Vec3b>(i,j)[2];
            rgb[3] = 0;
            Pointcloud.points[k].rgb = *(float *)(rgb);
            k++;
        }
#endif
    //process the points for objects and projecting to image space
    for(int i = 0; i < this->gazebo_models.name.size(); i++)   //only if when data are received
    {
        for(int j = 0; j < gazebo_model_types.size(); j++)
        {
            int k = gazebo_models.name.at(i).find(gazebo_model_types.at(j).str);
            if(k>=0) //if find points
            {
                gazebo_model_types.at(j).index.push_back(i);
                ST_POINT model_point;
                model_point.x = gazebo_models.pose.at(i).position.x;
                model_point.y = gazebo_models.pose.at(i).position.y;
                if(PtInPolygon(model_point,fov_points,4))
                {
                    gazebo_model_types.at(j).visible.push_back(i);
                    std::cout<<gazebo_model_types.at(j).visible.size()<<std::endl;
                }
            }

        }
    }


    for(int j = 0; j < gazebo_model_types.size(); j++)
    {
        gazebo_model_types.at(j).index.clear();
        gazebo_model_types.at(j).visible.clear();
    }

    //get the duration....
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"process_time is "<< duration << " second" <<'\n';
    //publish pointcloud
    pcl::toROSMsg(Pointcloud,cloud_msg);
    pointcloud_pub_.publish(cloud_msg);

}



void get_train_data::imageCallback(const sensor_msgs::ImageConstPtr& img,
                                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                                       const nav_msgs::OdometryConstPtr& odom)
{
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(img,"bgr8")->image);
        //process to pointcloud
        p2p(img,cam_info,odom);
        cv::waitKey(10);
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


/**
 * 功能：判断点是否在多边形内
 * 方法：求解通过该点的水平线（射线）与多边形各边的交点
 * 结论：单边交点为奇数，成立!
 * 参数：p 指定的某个点
         ptPolygon 多边形的各个顶点坐标（首末点可以不一致）
         nCount 多边形定点的个数
 * 说明：
 */
bool get_train_data::PtInPolygon(ST_POINT p, ST_POINT* ptPolygon, int nCount)
{
    int nCross = 0, i;
    float x;
    ST_POINT p1, p2;

    for (i = 0; i < nCount; i++)
    {
        p1 = ptPolygon[i];
        p2 = ptPolygon[(i + 1) % nCount];
        // 求解 y=p.y 与 p1p2 的交点
        if ( p1.y == p2.y ) // p1p2 与 y=p.y平行
            continue;
        if ( p.y < std::min(p1.y, p2.y) ) // 交点在p1p2延长线上
            continue;
        if ( p.y >= std::max(p1.y, p2.y) ) // 交点在p1p2延长线上
            continue;
        // 求交点的 X 坐标 --------------------------------------------------------------
        x = (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
        if ( x > p.x )
        {
            nCross++; // 只统计单边交点
        }
    }
    // 单边交点为偶数，点在多边形之外 ---
    return (nCross % 2 == 1);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_train_data");
    get_train_data u_i2p;
    u_i2p.init();

    ros::spin();
    cv::destroyWindow("view");
}
