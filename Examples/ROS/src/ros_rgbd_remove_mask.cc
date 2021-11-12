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
#include <geometry_msgs/PoseStamped.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"
#include <mutex>
#include <queue>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ros::NodeHandle& nh):mpSLAM(pSLAM){
        pub_pose_current_frame_ = nh.advertise<nav_msgs::Odometry>("/tesse/odom",10);
        pub_pose_current_frame_geo_ = nh.advertise<geometry_msgs::PoseStamped>("/orbslam3/odom",10);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabMask(const sensor_msgs::ImageConstPtr& msgM);

    ORB_SLAM3::System* mpSLAM;
    ros::Publisher pub_pose_current_frame_;
    tf2_ros::TransformBroadcaster br_;
    ros::Publisher pub_pose_current_frame_geo_;
    std::mutex mask_q_locker_;
    std::mutex rgbd_q_locker_;
    std::queue<sensor_msgs::ImageConstPtr> mask_q_;
    std::queue<std::pair<sensor_msgs::ImageConstPtr,sensor_msgs::ImageConstPtr> > rgbd_q_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_remove_mask");
    ros::start();

    ros::NodeHandle nh;

    std::string ORBvoc_address, yaml_address, rgb_image_topic, depth_image_topic, im_left_mask_topic;
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

    if(nh.getParam("rgb_image_topic",rgb_image_topic)){
        ROS_INFO("left image topic %s", rgb_image_topic.c_str());
    }
    else{
        rgb_image_topic = "/gray_image0";
        ROS_WARN("no rgb image topic, default %s", rgb_image_topic.c_str());
    }

    if(nh.getParam("depth_image_topic",depth_image_topic)){
        ROS_INFO("depth_image_topic image topic %s", depth_image_topic.c_str());
    }
    else{
        depth_image_topic = "/depth";
        ROS_WARN("no depth image topic, default %s", depth_image_topic.c_str());
    }

    if(nh.getParam("im_left_mask_topic",im_left_mask_topic)){
        ROS_INFO("left image topic %s", im_left_mask_topic.c_str());
    }
    else{
        im_left_mask_topic = "/image_seg0";
        ROS_WARN("no left image topic, default %s", im_left_mask_topic.c_str());
    }


    if(nh.getParam("show_orbslam_UI",show_orbslam_UI)){
        ROS_INFO("Show orbslam UI? %d", show_orbslam_UI);
    }
    else{
        ROS_WARN("no show orbslam UI value, default False");
    }  

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(ORBvoc_address,yaml_address,ORB_SLAM3::System::RGBD,show_orbslam_UI);//false: no viewer

    ImageGrabber igb(&SLAM, nh);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_image_topic, 100);//"/camera/rgb/image_raw"
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_image_topic, 100);//"camera/depth_registered/image_raw"
    ros::Subscriber sub = nh.subscribe(im_left_mask_topic, 10, &ImageGrabber::GrabMask,&igb);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

//Mask may be late because we need to put into the segmentation system
void ImageGrabber::GrabMask(const sensor_msgs::ImageConstPtr& msgM){
    std::lock_guard<std::mutex> lock(mask_q_locker_);
    mask_q_.push(msgM);
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    rgbd_q_locker_.lock();
    rgbd_q_.push(make_pair(msgRGB,msgD));
    rgbd_q_locker_.unlock();


    auto im_rgbd = rgbd_q_.front();
    sensor_msgs::ImageConstPtr im_mask;
    if(!mask_q_.empty()){
        im_mask = mask_q_.front();
    }
    else{
        ROS_INFO("wait for mask....");
        return;
    }

    ROS_INFO("get the image ......");

    double time_im_rgbd = im_rgbd.first->header.stamp.toSec();
    double time_im_mask = im_mask->header.stamp.toSec();

    //cout<<"time is "<<std::setprecision(18)<<time_im_rgbd<<","<<time_im_mask<<", difference "<<time_im_rgbd-time_im_mask<<endl;
    double th = 0.033;//use th as thershold to decide if the stereo image and mask are pairs

    //Check image pairs
    if(abs(time_im_rgbd - time_im_mask) <= th){
        std::lock_guard<std::mutex> lock0(mask_q_locker_);
        std::lock_guard<std::mutex> lock1(rgbd_q_locker_);
        rgbd_q_.pop();
        mask_q_.pop();
    }
    else if(time_im_rgbd - time_im_mask > th){
        ROS_INFO("rgbd pair is in front of the mask. Should not often happen");
        while(time_im_rgbd - time_im_mask > th && !mask_q_.empty()){
            mask_q_locker_.lock();
            mask_q_.pop();
            mask_q_locker_.unlock();
            if(!mask_q_.empty())
                im_mask = mask_q_.front();
            else{
                ROS_WARN("Mask lags too much behind the rgbd");
                return;                
            }
            time_im_mask = im_mask->header.stamp.toSec();
        }

        //if the sequence is switched, it means the image is not well synchronized
        if(time_im_mask - time_im_rgbd > th){
            ROS_WARN("Image is not well synchronized. The localization result may not be good");
        }
    }
    else if(time_im_mask - time_im_rgbd > th){
        ROS_INFO("mask is in front of the rgbd pair");
        while(time_im_mask - time_im_rgbd > th && !rgbd_q_.empty()){
            rgbd_q_locker_.lock();
            rgbd_q_.pop();
            rgbd_q_locker_.unlock();
            if(!rgbd_q_.empty())
                im_rgbd = rgbd_q_.front();
            else{
                ROS_WARN("rgbd lags too much behind the mask");
                return;                
            }
            time_im_rgbd = im_rgbd.first->header.stamp.toSec();;
        }

        if(time_im_rgbd - time_im_mask > th){
            ROS_WARN("Image is not well synchronized. The localization result may not be good");
        }
    }

    //cout<<"final time is "<<std::setprecision(18)<<time_im_rgbd<<","<<time_im_mask<<", difference "<<time_im_rgbd-time_im_mask<<endl;

    cv_bridge::CvImagePtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(im_rgbd.first);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(im_rgbd.second);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrM;
    try
    {
        cv_ptrM = cv_bridge::toCvCopy(im_mask);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Turn rgb in the mask to white
    int counter = 0;
    for(int i = 0; i<msgRGB->width; i++)
        for(int j = 0; j<msgRGB->height; j++){
            if(cv_ptrM->image.ptr<uchar>(j)[i] == 255){
                counter++;
                //cv_ptrRGB->image.at<uchar>(j,i,0) = 255;
                if(cv_ptrRGB->encoding == "bgr8" || cv_ptrRGB->encoding == "rgb8" || cv_ptrRGB->encoding == "8UC3"){
                    cv_ptrRGB->image.ptr<uchar>(j)[i*3 + 0] = 255;
                    cv_ptrRGB->image.ptr<uchar>(j)[i*3 + 1] = 255;
                    cv_ptrRGB->image.ptr<uchar>(j)[i*3 + 2] = 255;
                }
                else if(cv_ptrRGB->encoding == "mono8" || cv_ptrRGB->encoding == "8UC1")
                    cv_ptrRGB->image.ptr<uchar>(j)[i] = 255;
                else{
                    ROS_ERROR("image encoding not support!");
                    return;
                }
            }
        }

    
    // std::cout<<"we have "<<counter<<" points in the mask"<<std::endl;
    // cv::imshow("Display window", cv_ptrM->image);
    // cv::waitKey(100);

    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    

    if(Tcw.empty())
        return;

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);

    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    msg.header.stamp = msgRGB->header.stamp;
    msg.child_frame_id = "base_link_gt";
    msg.pose.pose.orientation.w = q[3];
    msg.pose.pose.orientation.x = q[0];
    msg.pose.pose.orientation.y = q[1];
    msg.pose.pose.orientation.z = q[2];
    msg.pose.pose.position.x = twc.ptr<float>(0)[0];
    msg.pose.pose.position.y = twc.ptr<float>(0)[1];
    msg.pose.pose.position.z = twc.ptr<float>(0)[2];
    pub_pose_current_frame_.publish(msg);

    ros::Time rostImLeft;
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
    p.header.stamp = msgRGB->header.stamp;
    p.pose.orientation.w = q[3];
    p.pose.orientation.x = q[0];
    p.pose.orientation.y = q[1];
    p.pose.orientation.z = q[2];
    p.pose.position.x = twc.ptr<float>(0)[0];
    p.pose.position.y = twc.ptr<float>(0)[1];
    p.pose.position.z = twc.ptr<float>(0)[2];
    pub_pose_current_frame_geo_.publish(p);

    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link_gt";
    transform.header.stamp = msgRGB->header.stamp;
    transform.transform.rotation = msg.pose.pose.orientation;
    transform.transform.translation.x = msg.pose.pose.position.x;
    transform.transform.translation.y = msg.pose.pose.position.y;
    transform.transform.translation.z = msg.pose.pose.position.z;
    br_.sendTransform(transform);
}



