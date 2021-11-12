/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM,ros::NodeHandle& nh):mpSLAM(pSLAM){
        pub_pose_current_frame_ = nh.advertise<nav_msgs::Odometry>("/tesse/odom",10);
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight, const sensor_msgs::ImageConstPtr& msgM);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    ros::Publisher pub_pose_current_frame_;
    tf2_ros::TransformBroadcaster br_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_mask_remove");
    ros::start();

    ros::NodeHandle nh;

    std::string ORBvoc_address, yaml_address, im_left_topic, im_right_topic, im_left_mask_topic;
    bool do_rectification = false, show_orbslam_UI = false;

    if(nh.getParam("ORBvoc_address",ORBvoc_address)){
        ROS_INFO("get ros vocabulary address %s", ORBvoc_address.c_str());
    }
    else{
        ROS_ERROR("no vocabulary address, ERROR!");
        ros::shutdown();
    }

    if(nh.getParam("yaml_address",yaml_address)){
        ROS_INFO("get sensor setting address %s", yaml_address.c_str());
    }
    else{
        ROS_ERROR("no sensor configuration address, ERROR!");
        ros::shutdown();
    }

    if(nh.getParam("im_left_topic",im_left_topic)){
        ROS_INFO("left image topic %s", im_left_topic.c_str());
    }
    else{
        im_left_topic = "/gray_image0";
        ROS_WARN("no left image topic, default %s", im_left_topic.c_str());
    }

    if(nh.getParam("im_right_topic",im_right_topic)){
        ROS_INFO("right image topic %s", im_right_topic.c_str());
    }
    else{
        im_right_topic = "/gray_image1";
        ROS_WARN("no right image topic, default %s", im_right_topic.c_str());
    }

    if(nh.getParam("im_left_mask_topic",im_left_mask_topic)){
        ROS_INFO("left image topic %s", im_left_mask_topic.c_str());
    }
    else{
        im_left_mask_topic = "/image_seg0";
        ROS_WARN("no left image topic, default %s", im_left_mask_topic.c_str());
    }

    if(nh.getParam("do_rectification",do_rectification)){
        ROS_INFO("Do rectification?  %d", do_rectification);
    }
    else{
        ROS_WARN("Not set do rectification value default False");
    }

    if(nh.getParam("show_orbslam_UI",show_orbslam_UI)){
        ROS_INFO("Show orbslam UI? %d", show_orbslam_UI);
    }
    else{
        ROS_WARN("no show orbslam UI value, default False");
    }  
   

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(ORBvoc_address,yaml_address,ORB_SLAM3::System::STEREO,show_orbslam_UI);

    ImageGrabber igb(&SLAM,nh);

    igb.do_rectify = do_rectification;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(yaml_address, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

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
    }

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, im_left_topic, 10);//
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, im_right_topic, 10);//
    message_filters::Subscriber<sensor_msgs::Image> mask_sub(nh, im_left_mask_topic, 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub,mask_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2,_3));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight,const sensor_msgs::ImageConstPtr& msgM)
{
    // Copy the ros image message to cv::Mat.
    // cv_bridge::CvImageConstPtr cv_ptrLeft;
    // try
    // {
    //     cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // cv_bridge::CvImageConstPtr cv_ptrRight;
    // try
    // {
    //     cv_ptrRight = cv_bridge::toCvShare(msgRight);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    cv_bridge::CvImagePtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvCopy(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrM;
    try
    {
        cv_ptrM = cv_bridge::toCvShare(msgM);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Turn rgb in the mask to white
    for(unsigned int i = 0; i<msgLeft->width; i++)
        for(unsigned int j = 0; j<msgLeft->height; j++){
            if(cv_ptrM->image.ptr<uchar>(j)[i] == 255){
                //cv_ptrRGB->image.at<uchar>(j,i,0) = 255;
                if(cv_ptrLeft->encoding == "bgr8" || cv_ptrLeft->encoding == "rgb8" || cv_ptrLeft->encoding == "8UC3"){
                    cv_ptrLeft->image.ptr<uchar>(j)[i*3 + 0] = 255;
                    cv_ptrLeft->image.ptr<uchar>(j)[i*3 + 1] = 255;
                    cv_ptrLeft->image.ptr<uchar>(j)[i*3 + 2] = 255;
                }
                else if(cv_ptrLeft->encoding == "mono8" || cv_ptrLeft->encoding == "8UC1")
                    cv_ptrLeft->image.ptr<uchar>(j)[i] = 255;
                else
                    ROS_ERROR("image encoding not support!");
                    return;
            }
        }


    cv::Mat Tcw;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    if(Tcw.empty())
        return;

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);

    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    msg.header.stamp = msgLeft->header.stamp;
    msg.child_frame_id = "base_link_gt";
    msg.pose.pose.orientation.w = q[3];
    msg.pose.pose.orientation.x = q[0];
    msg.pose.pose.orientation.y = q[1];
    msg.pose.pose.orientation.z = q[2];
    msg.pose.pose.position.x = twc.ptr<float>(0)[0];
    msg.pose.pose.position.y = twc.ptr<float>(0)[1];
    msg.pose.pose.position.z = twc.ptr<float>(0)[2];
    pub_pose_current_frame_.publish(msg);

    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link_gt";
    transform.header.stamp = msgLeft->header.stamp;
    transform.transform.rotation = msg.pose.pose.orientation;
    transform.transform.translation.x = msg.pose.pose.position.x;
    transform.transform.translation.y = msg.pose.pose.position.y;
    transform.transform.translation.z = msg.pose.pose.position.z;
    br_.sendTransform(transform);
}


