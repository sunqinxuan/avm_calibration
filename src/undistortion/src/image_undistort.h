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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include <yaml-cpp/yaml.h>
#include <vector>

#include "ocam_functions.h"

class ImageUndistortion
{
public:

	ImageUndistortion():it_(nh_)
	{
		image_pub_=it_.advertiseCamera("/cv_camera_undistort/image_raw",7);
		image_sub_=it_.subscribe("/cv_camera/image_raw",7,&ImageUndistortion::ImageCallBack,this);
//		cv::cvNamedWindow("cv_camera",CV_WINDOW_AUTOSIZE);
//		cv::cvNamedWindow("cv_camera_undistorted",CV_WINDOW_AUTOSIZE);
	}

	~ImageUndistortion()
	{
//		cv::destroyWindow("cv_camera");
//		cv::destroyWindow("cv_camera_undistorted");
	}

	void ImageCallBack(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;

		try 
		{
			cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception is %s", e.what());
			return;
		}

		get_ocam_model(&o_, "./calib_results.txt");
//		int i;
//		printf("pol =\n"); for(i=0; i<o_.length_pol; i++) {printf("\t%e\n",o_.pol[i]);}; printf("\n");
//		printf("invpol =\n"); for (i=0; i<o_.length_invpol; i++){ printf("\t%e\n",o_.invpol[i]); }; printf("\n");  
//		printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",o_.xc,o_.yc,o_.width,o_.height);

		IplImage *src=cvCreateImage(cvSize(o_.width,o_.height),8,3);
		IplImage src_tmp=cv_ptr->image;
		cvCopy(&src_tmp,src);
//		*src=cv_ptr->image;
//		IplImage* src = new IplImage(cv_ptr->image);
//		cvNamedWindow("cv_camera_src", 1 );
//		cvShowImage("cv_camera_src", src);
//		cv::waitKey(0);

		IplImage *dst=cvCreateImage(cvGetSize(src),8,3);
		CvMat* mapx_persp = cvCreateMat(src->height, src->width, CV_32FC1);
		CvMat* mapy_persp = cvCreateMat(src->height, src->width, CV_32FC1);

		float sf = 10;
		create_perspecive_undistortion_LUT( mapx_persp, mapy_persp, &o_, sf);

		cvRemap(src, dst, mapx_persp, mapy_persp, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );

//		cvNamedWindow("cv_camera_dst", 1 );
//		cvShowImage("cv_camera_dst", dst);
//		cv::waitKey(0);
//		cvShowImage("cv_camera_undistorted", dst);
//		cvSaveImage("undistorted.jpg",dst);
//		printf("\nImage %s saved\n","undistorted.jpg");

		cv::Mat dst_mat=cv::cvarrToMat(dst);
//		cvShowImage("cv_camera_dst", dst_mat);
//		cv::imshow("cv_camera_dst",dst_mat);
//		cv::waitKey(0);
		sensor_msgs::ImagePtr msg_pub=cv_bridge::CvImage(msg->header,"bgr8",dst_mat).toImageMsg();

//		boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo;
//		cinfo.reset(new camera_info_manager::CameraInfoManager(nh_,
		sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo());
		ci->header.frame_id=msg_pub->header.frame_id;
		ci->header.stamp=msg_pub->header.stamp;

		YAML::Node cam_yaml=YAML::LoadFile("camera_undistort.yaml");
		std::vector<double> K_matrix=cam_yaml["camera_matrix"]["data"].as<std::vector<double>>();
		std::vector<double> D_matrix=cam_yaml["distortion_coefficients"]["data"].as<std::vector<double>>();
		std::vector<double> P_matrix=cam_yaml["projection_matrix"]["data"].as<std::vector<double>>();
		ci->distortion_model=cam_yaml["distortion_model"].as<std::string>();
		for(int i=0;i<K_matrix.size();i++) ci->K[i]=K_matrix[i];
		for(int i=0;i<P_matrix.size();i++) ci->P[i]=P_matrix[i];
		ci->D=D_matrix;

//		double fx=600,fy=600,cx=640,cy=360;
//		ci->K[0] = fx;
//		ci->K[2] = cx;
//		ci->K[4] = fy;
//		ci->K[5] = cy;
//		ci->K[8] = 1.0;
//		ci->P[0] = fx;
//		ci->P[2] = cx;
//		ci->P[5] = fy;
//		ci->P[6] = cy;
//		ci->P[10] = 1.0;

		image_pub_.publish(msg_pub,ci);
//		try 
//		{
//			image_pub_.publish(bridge_.cvToImgMsg(dst,"bgr8"));
//		}
//		catch (sensor_msgs::CvBridgeException error)
//		{
//			ROS_ERROR("publish error");
//		}

		cvReleaseImage(&src);
		cvReleaseImage(&dst);
		cvReleaseMat(&mapx_persp);
		cvReleaseMat(&mapy_persp);  
	}

private:

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::CameraPublisher image_pub_;
//	sensor_msgs::CvBridge bridge_;

	struct ocam_model o_;
};
