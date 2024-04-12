/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2021-09-08 09:35
#
# Filename: image_undistort.h
#
# Description: 
#
************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <map>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

typedef std::map<std::string,Eigen::Isometry3d>::iterator iterTransform;
typedef std::map<std::string,Eigen::Isometry3d>::const_iterator const_iterTransform;

class TagLocalization
{
public:

	TagLocalization();

	~TagLocalization()
	{
	}

	void run()
	{
	}

	void publishPose();

private:

	void TagCallBack_right(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
	void TagCallBack_front(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
	void TagCallBack_left(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
	void TagCallBack_rear(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

private:

	ros::NodeHandle nh_;

	ros::Subscriber tag_sub_right_;
	ros::Subscriber tag_sub_front_;
	ros::Subscriber tag_sub_left_;
	ros::Subscriber tag_sub_rear_;

	ros::Publisher tag_pub_right_;
	ros::Publisher tag_pub_front_;
	ros::Publisher tag_pub_left_;
	ros::Publisher tag_pub_rear_;

	geometry_msgs::PoseStamped pose_msg_right_;
	geometry_msgs::PoseStamped pose_msg_front_;
	geometry_msgs::PoseStamped pose_msg_left_;
	geometry_msgs::PoseStamped pose_msg_rear_;

//	Eigen::Isometry3d Ttag;

	std::map<std::string,Eigen::Isometry3d> T_bi_gt_;

//	Eigen::Isometry3d T_cb_;

//	std::vector<bool> flag_right_,flag_front_,flag_left_,flag_rear_;
//	const int size_right_,size_front_,size_left_,size_rear_;

	std::map<std::string,Eigen::Isometry3d> T_ci_right_; // [all,32,48,64,80]
	std::map<std::string,Eigen::Isometry3d> T_ci_front_; // [all,0,16,32]
	std::map<std::string,Eigen::Isometry3d> T_ci_left_;  // [all,0,112,128,144]
	std::map<std::string,Eigen::Isometry3d> T_ci_rear_;  // [all,80,96,112]
};
