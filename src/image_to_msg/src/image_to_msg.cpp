/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2019-06-21 09:45
#
# Filename: imgPublisher.cpp
#
# Description: 
#
************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <time.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
//#include <ros/spinner.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

class ImgPublisher
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
//	ros::Publisher img_right_pub;
//	ros::Publisher img_front_pub;
//	ros::Publisher img_left_pub;
//	ros::Publisher img_rear_pub;
	image_transport::CameraPublisher image_pub_right_;
	image_transport::CameraPublisher image_pub_front_;
	image_transport::CameraPublisher image_pub_left_;
	image_transport::CameraPublisher image_pub_rear_;

	std::string folder_name;
	std::vector<std::string> image_filenames;

	sensor_msgs::CameraInfoPtr camera_info_right_;
	sensor_msgs::CameraInfoPtr camera_info_front_;
	sensor_msgs::CameraInfoPtr camera_info_left_;
	sensor_msgs::CameraInfoPtr camera_info_rear_;

public:
	ImgPublisher(const std::string &folder): it_(nh_),
		camera_info_right_(new sensor_msgs::CameraInfo()),
		camera_info_front_(new sensor_msgs::CameraInfo()),
		camera_info_left_(new sensor_msgs::CameraInfo()),
		camera_info_rear_(new sensor_msgs::CameraInfo())
	{
		folder_name=folder;

		image_pub_right_= it_.advertiseCamera("/cv_camera_right/image_raw",7);
		image_pub_front_= it_.advertiseCamera("/cv_camera_front/image_raw",7);
		image_pub_left_ = it_.advertiseCamera("/cv_camera_left/image_raw",7);
		image_pub_rear_ = it_.advertiseCamera("/cv_camera_rear/image_raw",7);

		std::string yaml_path=folder_name+"calib_right.yaml";
		YAML::Node cam_yaml=YAML::LoadFile(yaml_path.c_str());
		std::vector<double> K_matrix=cam_yaml["camera_matrix"]["data"].as<std::vector<double>>();
		std::vector<double> D_matrix=cam_yaml["distortion_coefficients"]["data"].as<std::vector<double>>();
		for(int i=0;i<K_matrix.size();i++) camera_info_right_->K[i]=K_matrix[i];
		camera_info_right_->D=D_matrix;

		yaml_path=folder_name+"calib_front.yaml";
		cam_yaml=YAML::LoadFile(yaml_path.c_str());
		K_matrix=cam_yaml["camera_matrix"]["data"].as<std::vector<double>>();
		D_matrix=cam_yaml["distortion_coefficients"]["data"].as<std::vector<double>>();
		for(int i=0;i<K_matrix.size();i++) camera_info_front_->K[i]=K_matrix[i];
		camera_info_front_->D=D_matrix;

		yaml_path=folder_name+"calib_left.yaml";
		cam_yaml=YAML::LoadFile(yaml_path.c_str());
		K_matrix=cam_yaml["camera_matrix"]["data"].as<std::vector<double>>();
		D_matrix=cam_yaml["distortion_coefficients"]["data"].as<std::vector<double>>();
		for(int i=0;i<K_matrix.size();i++) camera_info_left_->K[i]=K_matrix[i];
		camera_info_left_->D=D_matrix;

		yaml_path=folder_name+"calib_rear.yaml";
		cam_yaml=YAML::LoadFile(yaml_path.c_str());
		K_matrix=cam_yaml["camera_matrix"]["data"].as<std::vector<double>>();
		D_matrix=cam_yaml["distortion_coefficients"]["data"].as<std::vector<double>>();
		for(int i=0;i<K_matrix.size();i++) camera_info_rear_->K[i]=K_matrix[i];
		camera_info_rear_->D=D_matrix;

//		// K=[429.7445911405171, 0, 619.2259664343836;
//		//  0, 429.8380306301192, 401.9287812132077;
//		//  0, 0, 1]
//		// D=[0.299383368920503;
//		//  0.07355700835564347;
//		//  -0.06920002447924962;
//		//  0.01045030304436501]
//		// rms=0.265354
//		camera_info_right_->K[0]=429.7445911405171;
//		camera_info_right_->K[2]=619.2259664343836;
//		camera_info_right_->K[4]=429.8380306301192;
//		camera_info_right_->K[5]=401.9287812132077;
//		camera_info_right_->K[8]=1.0;
//		camera_info_right_->D.resize(4);
//		camera_info_right_->D[0]=0.299383368920503;
//		camera_info_right_->D[1]=0.07355700835564347;
//		camera_info_right_->D[2]=-0.06920002447924962;
//		camera_info_right_->D[3]=0.01045030304436501;
//
//		// K=[433.1639126633538, 0, 595.3000629743157;
//		//  0, 432.7563491855097, 386.1423482509578;
//		//  0, 0, 1]
//		// D=[0.3090410462199386;
//		//  0.06266451770686568;
//		//  -0.0402330843147774;
//		//  -0.01233306309777425]
//		// rms=0.283606
//		camera_info_front_->K[0]=433.1639126633538;
//		camera_info_front_->K[2]=595.3000629743157;
//		camera_info_front_->K[4]=432.7563491855097;
//		camera_info_front_->K[5]=386.1423482509578;
//		camera_info_front_->K[8]=1.0;
//		camera_info_front_->D.resize(4);
//		camera_info_front_->D[0]=0.3090410462199386;
//		camera_info_front_->D[1]=0.06266451770686568;
//		camera_info_front_->D[2]=-0.0402330843147774;
//		camera_info_front_->D[3]=-0.01233306309777425;
//
//		// K=[431.1064501485133, 0, 614.0835353823536;
//		//  0, 431.8317575360658, 399.3146912097641;
//		//  0, 0, 1]
//		// D=[0.3039910607171842;
//		//  0.1157562545882771;
//		//  -0.1361880951399535;
//		//  0.04323683896923074]
//		// rms=0.206466
//		camera_info_left_->K[0]=431.1064501485133;
//		camera_info_left_->K[2]=614.0835353823536;
//		camera_info_left_->K[4]=431.8317575360658;
//		camera_info_left_->K[5]=399.3146912097641;
//		camera_info_left_->K[8]=1.0;
//		camera_info_left_->D.resize(4);
//		camera_info_left_->D[0]=0.3039910607171842;
//		camera_info_left_->D[1]=0.1157562545882771;
//		camera_info_left_->D[2]=-0.1361880951399535;
//		camera_info_left_->D[3]=0.04323683896923074;
//
//		// K=[431.7134961165484, 0, 654.4164575106633;
//		//  0, 431.3275788330303, 389.0474069629062;
//		//  0, 0, 1]
//		// D=[0.29981361155376;
//		//  0.08490468424384201;
//		//  -0.0906486389453772;
//		//  0.02267462825560619]
//		// rms=0.219896
//		camera_info_rear_->K[0]=431.7134961165484;
//		camera_info_rear_->K[2]=654.4164575106633;
//		camera_info_rear_->K[4]=431.3275788330303;
//		camera_info_rear_->K[5]=389.0474069629062;
//		camera_info_rear_->K[8]=1.0;
//		camera_info_rear_->D.resize(4);
//		camera_info_rear_->D[0]=0.29981361155376;
//		camera_info_rear_->D[1]=0.08490468424384201;
//		camera_info_rear_->D[2]=-0.0906486389453772;
//		camera_info_rear_->D[3]=0.02267462825560619;
	}

	void setFolderName(const std::string &name) { folder_name=name; }

	~ImgPublisher()
	{
	}

	void run()
	{
		loadImage(folder_name);
		ros::Rate loop_rate(10);
		while(ros::ok())
		{
			for(int i=0;i<image_filenames.size();i++)
			{
				std::cout<<image_filenames[i].c_str()<<std::endl;
				publishImage(image_filenames[i].c_str());

				ros::spinOnce();
				loop_rate.sleep();
			}
		}
	}

private:

	void publishImage(const std::string &file_name)
	{
		std_msgs::Header header;
		header.stamp=ros::Time::now();
//		for(int i=0;i<image_filenames.size();i++)
//		{
		header.frame_id="camera_right";
		std::string file_right=folder_name+"img_right/"+file_name;
		cv::Mat img_right=cv::imread(file_right.c_str(),1);
		sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(header,"bgr8",img_right).toImageMsg();
		camera_info_right_->header=header;
		image_pub_right_.publish(msg_right,camera_info_right_);

		header.frame_id="camera_front";
		std::string file_front=folder_name+"img_front/"+file_name;
		cv::Mat img_front=cv::imread(file_front.c_str(),1);
		sensor_msgs::ImagePtr msg_front = cv_bridge::CvImage(header,"bgr8",img_front).toImageMsg();
		camera_info_front_->header=header;
		image_pub_front_.publish(msg_front,camera_info_front_);

		header.frame_id="camera_left";
		std::string file_left =folder_name+"img_left/" +file_name;
		cv::Mat img_left=cv::imread(file_left.c_str(),1);
		sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(header,"bgr8",img_left).toImageMsg();
		camera_info_left_->header=header;
		image_pub_left_.publish(msg_left,camera_info_left_);

		header.frame_id="camera_rear";
		std::string file_rear =folder_name+"img_rear/" +file_name;
		cv::Mat img_rear=cv::imread(file_rear.c_str(),1);
		sensor_msgs::ImagePtr msg_rear = cv_bridge::CvImage(header,"bgr8",img_rear).toImageMsg();
		camera_info_rear_->header=header;
		image_pub_rear_.publish(msg_rear,camera_info_rear_);
//			ros::Rate loop_rate(1);
//			while(nh_.ok())
//			{
//				ros::spinOnce();
//				loop_rate.sleep();
//			}
//		}
	}

	void loadImage(const std::string &folder)
	{
		std::string file_name=folder+"img.txt";
		std::cout<<file_name<<std::endl;
		
		std::ifstream fp;
		fp.open(file_name.c_str());
		image_filenames.clear();
		while(!fp.eof())
		{
			std::string s; getline(fp,s);
			if(!s.empty())
			{
				image_filenames.push_back(s);
			}
		}
		fp.close();
	}

}; 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imgPublisher", ros::init_options::AnonymousName);

	if(!ros::ok())
	{
		return 0;
	}

	std::string folder_name="./data/img/";

	ImgPublisher img_publisher(folder_name);

	img_publisher.run();

	ros::shutdown();
	return 0;
}

