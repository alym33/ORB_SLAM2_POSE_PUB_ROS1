/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

//Ali
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <Converter.h>
//using ORB_SLAM2::System;
//using ORB_SLAM2::Converter;
//using ORB_SLAM2::MapPoint;
//using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    
    
    void publishPose(const cv::Mat &Tcw, const std_msgs::Header &image_header); //Ali
    std::string p_ref_frame_, p_parent_frame_, p_child_frame_;
    ros::Publisher pub_pose_, pub_pose2d_, pub_pc_;
    bool getTransform(tf::StampedTransform &transform, std::string from, std::string to, ros::Time stamp);
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    const double planar_tol_ = 0.1;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

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

   cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec()); //Ali
   
    publishPose(Tcw, cv_ptr->header); //Ali
}

void ImageGrabber::publishPose(const cv::Mat &Tcw, const std_msgs::Header &image_header)
{
    // Publish 3D pose
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    // x right, y down, z forward

    Eigen::Matrix<double,3,3> eigm = ORB_SLAM2::Converter::toMatrix3d(Rwc);
    Eigen::Quaterniond eigq(eigm);

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = p_ref_frame_;
    msg.header.stamp = image_header.stamp;
    msg.pose.position.x = twc.at<float>(0);
    msg.pose.position.y = twc.at<float>(1);
    msg.pose.position.z = twc.at<float>(2);
    msg.pose.orientation.x = eigq.x();
    msg.pose.orientation.y = eigq.y();
    msg.pose.orientation.z = eigq.z();
    msg.pose.orientation.w = eigq.w();
    pub_pose_.publish(msg);

    // Publish tf
    tf::Transform ref_to_image;
    tf::StampedTransform parent_to_ref, child_to_image;
    tf::poseMsgToTF(msg.pose, ref_to_image);
    std::string image_frame = image_header.frame_id;
    std::string parent_frame = p_parent_frame_, child_frame = p_child_frame_;
    bool get_tf_succeed = true;
    if (p_parent_frame_.empty()) {
        parent_frame = p_ref_frame_;
        parent_to_ref.setIdentity();
    } else {
        get_tf_succeed &= getTransform(parent_to_ref, parent_frame, p_ref_frame_, msg.header.stamp);
    }
    if (p_child_frame_.empty()) {
        child_frame = image_frame;
        child_to_image.setIdentity();
    } else {
        get_tf_succeed &= getTransform(child_to_image, child_frame, image_frame, msg.header.stamp);
    }
    if (get_tf_succeed) {
        tf::Transform parent_to_child = tf::Transform(parent_to_ref * ref_to_image * child_to_image.inverse());
        tf_broadcaster_.sendTransform(tf::StampedTransform(parent_to_child, msg.header.stamp, parent_frame, child_frame));
    }

    // Publish 2D pose
    if (pub_pose2d_.getNumSubscribers() > 0) {
        // 1. transform into the parent frame (usually "map")
        geometry_msgs::PoseStamped pose2d;
        tf_listener_.transformPose(parent_frame, msg, pose2d);
        // 2. rotate from camera coordinate (right-down-forward) to ROS coordinate (forward-left-up)
        tf::Quaternion q2d(pose2d.pose.orientation.x, pose2d.pose.orientation.y, pose2d.pose.orientation.z, pose2d.pose.orientation.w);
        tf::Quaternion cam2map(0.5, -0.5, 0.5, 0.5);
        q2d *= cam2map;
        // 3. warn if the actual pose is not in the x-y plane
        if (std::abs(pose2d.pose.position.z) > planar_tol_)
            ROS_WARN("Non-planar position: (%lf, %lf, %lf)", pose2d.pose.position.x, pose2d.pose.position.y, pose2d.pose.position.z);
        if (std::abs(q2d[0]) > planar_tol_ || std::abs(q2d[1]) > planar_tol_)
            ROS_WARN("Non-planar orientation: (%lf, %lf, %lf, %lf)", q2d[0], q2d[1], q2d[2], q2d[3]);
        // 4. make the pose strictly in the x-y plane and publish it
        double norm_factor = 1. / std::sqrt(q2d[2] * q2d[2] + q2d[3] * q2d[3]);
        pose2d.pose.position.z = 0;
        pose2d.pose.orientation.x = 0;
        pose2d.pose.orientation.y = 0;
        pose2d.pose.orientation.z = q2d[2] * norm_factor;
        pose2d.pose.orientation.w = q2d[3] * norm_factor;
        pub_pose2d_.publish(pose2d);
    }}

bool ImageGrabber::getTransform(tf::StampedTransform &transform, std::string from, std::string to, ros::Time stamp)
{
    try {
        //tf_listener_.waitForTransform(from, to, stamp, ros::Duration(0.5));
        tf_listener_.lookupTransform(from, to, stamp, transform);
        return true;
    } catch (tf::TransformException &e) {
        ROS_ERROR("Failed to get transform from %s to %s: %s",
            from.c_str(), to.c_str(), e.what());
        return false;
    }
}
