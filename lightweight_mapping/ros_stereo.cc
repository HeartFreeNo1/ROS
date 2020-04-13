/******************************************************
*Description:ROS下的双目相机接口,相机数据来源于gazebo仿真环境
*Time:2020.4.11
*Author:wangxinxin
******************************************************/
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<memory>
#include<functional>
#include<list>
#include<vector>
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include"boost/thread.hpp"
#include<opencv2/core/core.hpp>
#include<rosbag/bag.h>
#include<rosbag/chunked_file.h>
#include<rosbag/view.h>
#include<rosbag/query.h>
#include"src/System.h"
#include<stdio.h>
#include<X11/Xlib.h>
#include<boost/filesystem.hpp>
#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){;}
    ~ImageGrabber(){;}
    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify = false;
    cv::Mat M1l,M2l,M1r,M2r;
}igb;

std::mutex mMutexImg0;
std::mutex mMutexImg1;

int main(int argc, char **argv)
{
    XInitThreads();

    ros::init(argc, argv, "ROS_Stereo_node");
    ros::start();
    ros::NodeHandle nh;
    string packagePath = ros::package::getPath("light_mapping");

    string dictPath = packagePath + "//Vocabulary//ORBvoc.bin" ;
    string configPath = packagePath + "//config//user_setting.yaml";

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(dictPath,configPath,ORB_SLAM2::System::STEREO, true);
    SLAM.setPublisher(nh);
    igb.mpSLAM = &SLAM ;

    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }
    int preRectified = fsSettings["preRectified"] ;
    if ( preRectified > 0 ){
        igb.do_rectify = false ;
    }
    else{
        igb.do_rectify = true ;
    }
    if(igb.do_rectify)
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);

        igb.mpSLAM->mpLocalMesher->M1l = igb.M1l.clone();
        igb.mpSLAM->mpLocalMesher->M1r = igb.M1r.clone();
        igb.mpSLAM->mpLocalMesher->M2l = igb.M2l.clone();
        igb.mpSLAM->mpLocalMesher->M2r = igb.M2r.clone();
    }
    fsSettings.release();
    cv::Mat curImg(480, 640, CV_8U ) ;
    curImg.setTo(0) ;
    cv::imshow("Current Keyframe", curImg) ;
    cv::moveWindow("Current Keyframe", 0, 700) ;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();
    //std::getchar();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    //ros::shutdown();

    return 0;
}

// 双目图像获取类 函数
void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);// ros image message to cv::Mat.
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);// ros image message to cv::Mat.
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)//矫正
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        cv::Mat left_image,right_image,gray_left_image,gray_right_image;
        left_image=cv_ptrLeft->image;
        right_image=cv_ptrRight->image;
        cv::cvtColor(left_image, gray_left_image, CV_RGB2GRAY);//将图像转换为灰度图
        cv::cvtColor(right_image, gray_right_image, CV_RGB2GRAY);//将图像转换为灰度图
        mpSLAM->TrackStereo(gray_left_image,gray_right_image,cv_ptrLeft->header.stamp.toSec());
        //mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    // while (mpSLAM->mpLocalMesher->onView && ros::ok() ){
    //      usleep(1000);
    // }
    // while (mpSLAM->mpLocalMesher->mlpKeyFrameQueue.size() > 1 && ros::ok() ){
    //      usleep(1000);
    // }
    // while (mpSLAM->mpLoopCloser->mbRunningGBA && ros::ok() ){
    //      usleep(1000);
//    }
}
