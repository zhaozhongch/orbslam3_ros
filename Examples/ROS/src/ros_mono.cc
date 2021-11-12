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

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_remove_mask");
    ros::start();

    ros::NodeHandle nh;

    std::string ORBvoc_address, yaml_address, image_topic, depth_image_topic, im_left_mask_topic;
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

    if(nh.getParam("image_topic",image_topic)){
        ROS_INFO("left image topic %s", image_topic.c_str());
    }
    else{
        image_topic = "/gray_image0";
        ROS_WARN("no rgb image topic, default %s", image_topic.c_str());
    }


    if(nh.getParam("show_orbslam_UI",show_orbslam_UI)){
        ROS_INFO("Show orbslam UI? %d", show_orbslam_UI);
    }
    else{
        ROS_WARN("no show orbslam UI value, default False");
    }  

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(ORBvoc_address,yaml_address,ORB_SLAM3::System::MONOCULAR,show_orbslam_UI);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(image_topic, 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


