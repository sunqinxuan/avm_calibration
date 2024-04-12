/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2019-06-21 09:45
#
# Filename: readBag.cpp
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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <cv_bridge/cv_bridge.h>

class BagReader
{
private:

  const size_t queueSize;

  ros::NodeHandle nh;
  //ros::AsyncSpinner spinner;
  std::string folder_name;

public:
  BagReader(const std::string &folder): queueSize(5), nh("~")//, spinner(0)
  {
	folder_name=folder;
	std::cout<<"folder_name="<<folder_name.c_str()<<std::endl;
	std::string mkdir_img="mkdir -p "+folder_name+"/img";
	system(mkdir_img.c_str());
  }

  void setFolderName(const std::string &name) { folder_name=name; }

  ~BagReader()
  {
  }

  void run()
  {
    start();
    stop();
  }

private:
  void start()
  {
	std::string topicCam="/mipi_four_cameras";
	ros::Subscriber sub_cam = nh.subscribe(topicCam, queueSize, &BagReader::callback_cam, this);

    //spinner.start();
	ros::spin();

  }

  void stop()
  {
    //spinner.stop();
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

	void callback_cam(const sensor_msgs::Image::ConstPtr& msgImg)
	{
		std::string file_name=folder_name+"/img.txt";
		std::ofstream fp;
		fp.open(file_name.c_str(),std::ios::app);

		cv::Mat img;
		readImage(msgImg,img);

		std::ostringstream name;
//		name.precision(10);
		name.setf(std::ios::fixed);

		double sec=msgImg->header.stamp.sec;
		double nsec=msgImg->header.stamp.nsec;
		double time_stamp=sec+nsec*1e-9;
		fp<<std::fixed<<time_stamp<<" img/"<<std::fixed<<time_stamp<<".png"<<std::endl;
		name<<folder_name<<"/img/"<<time_stamp<<".png";
		cv::imwrite(name.str().c_str(),img);
		std::cout<<name.str().c_str()<<std::endl;

		fp.close();
	}

}; 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bag_to_image", ros::init_options::AnonymousName);

	if(!ros::ok())
	{
		return 0;
	}

	std::string folder_name;

	time_t tt = time(NULL);
	struct tm* t= localtime(&tt);
	std::ostringstream ostr;
	ostr<<t->tm_year + 1900<<"-"<<t->tm_mon + 1<<"-" <<t->tm_mday<<"-"
		<<t->tm_hour<<"-" <<t->tm_min<<"-" <<t->tm_sec;
	folder_name="./"+ostr.str();
	std::cout<<"save images to "<<folder_name<<std::endl;

//	// give folder_name in the command line;
//	for(size_t i = 1; i < (size_t)argc; ++i)
//	{
//		std::string param(argv[i]);
//		if(param == "-O") folder_name="./"+std::string(argv[i+1]);
//	}

	BagReader bag_reader(folder_name);

	std::cout<<"awaiting rosbag play ..."<<std::endl;
	bag_reader.run();

	ros::shutdown();
	return 0;
}

