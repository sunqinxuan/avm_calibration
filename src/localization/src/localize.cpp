/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2021-09-14 14:36
#
# Filename: localize.cpp
#
# Description: 
#
************************************************/
#include "localize.h"

TagLocalization::TagLocalization()//:size_right_(5),size_front_(4),size_left_(5),size_rear_(4)
{
	tag_pub_right_=nh_.advertise<geometry_msgs::PoseStamped>("/pose_wheel_camera_right",3);
	tag_pub_front_=nh_.advertise<geometry_msgs::PoseStamped>("/pose_wheel_camera_front",3);
	tag_pub_left_ =nh_.advertise<geometry_msgs::PoseStamped>("/pose_wheel_camera_left",3);
	tag_pub_rear_ =nh_.advertise<geometry_msgs::PoseStamped>("/pose_wheel_camera_rear",3);

	pose_msg_right_.header.frame_id="";
	pose_msg_front_.header.frame_id="";
	pose_msg_left_.header.frame_id="";
	pose_msg_rear_.header.frame_id="";

	tag_sub_right_=nh_.subscribe("/tag_detections_right",3,&TagLocalization::TagCallBack_right,this);
	tag_sub_front_=nh_.subscribe("/tag_detections_front",3,&TagLocalization::TagCallBack_front,this);
	tag_sub_left_ =nh_.subscribe("/tag_detections_left", 3,&TagLocalization::TagCallBack_left,this);
	tag_sub_rear_ =nh_.subscribe("/tag_detections_rear", 3,&TagLocalization::TagCallBack_rear,this);

	Eigen::Quaterniond q(1.0,0.0,0.0,0.0);
	Eigen::Isometry3d T;
	T.matrix().topLeftCorner(3,3)=q.toRotationMatrix();

//	T_bi_gt_.resize(10);

//	T_ci_right_.resize(5); flag_right_.resize(5);
//	T_ci_front_.resize(4); flag_front_.resize(4);
//	T_ci_left_.resize(5);  flag_left_.resize(5);
//	T_ci_rear_.resize(4);  flag_rear_.resize(4);

	T.translation()(0)=4.7;
	T.translation()(1)=1.25;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_0",T));

	T.translation()(0)=4.7;
	T.translation()(1)=-0.75;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_16",T));

	T.translation()(0)=4.7;
	T.translation()(1)=-2.75;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_32",T));

	T.translation()(0)=2.4;
	T.translation()(1)=-2.75;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_48",T));

	T.translation()(0)=0.0;
	T.translation()(1)=-2.75;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_64",T));

	T.translation()(0)=-2.3;
	T.translation()(1)=-2.75;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_80",T));

	T.translation()(0)=-2.3;
	T.translation()(1)=-0.75;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_96",T));

	T.translation()(0)=-2.3;
	T.translation()(1)=1.25;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_112",T));

	T.translation()(0)=0.0;
	T.translation()(1)=1.25;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_128",T));

	T.translation()(0)=2.4;
	T.translation()(1)=1.25;
	T.translation()(2)=0;
	T_bi_gt_.insert(std::pair<std::string,Eigen::Isometry3d>("bundle_144",T));
}

void TagLocalization::publishPose()
{
	if(!pose_msg_right_.header.frame_id.empty())
	{
		tag_pub_right_.publish(pose_msg_right_);
	}
	if(!pose_msg_front_.header.frame_id.empty())
	{
		tag_pub_front_.publish(pose_msg_front_);
	}
	if(!pose_msg_left_.header.frame_id.empty())
	{
		tag_pub_left_.publish(pose_msg_left_);
	}
	if(!pose_msg_rear_.header.frame_id.empty())
	{
		tag_pub_rear_.publish(pose_msg_rear_);
	}

	YAML::Node node;
	node["frame_id"]="bundle_all";

	YAML::Node pose=YAML::Load("[]");
	pose.push_back(pose_msg_right_.pose.position.x);
	pose.push_back(pose_msg_right_.pose.position.y);
	pose.push_back(pose_msg_right_.pose.position.z);
	pose.push_back(pose_msg_right_.pose.orientation.w);
	pose.push_back(pose_msg_right_.pose.orientation.x);
	pose.push_back(pose_msg_right_.pose.orientation.y);
	pose.push_back(pose_msg_right_.pose.orientation.z);
	node["pose_wheel_camera_right"]=pose;

	pose=YAML::Load("[]");
	pose.push_back(pose_msg_front_.pose.position.x);
	pose.push_back(pose_msg_front_.pose.position.y);
	pose.push_back(pose_msg_front_.pose.position.z);
	pose.push_back(pose_msg_front_.pose.orientation.w);
	pose.push_back(pose_msg_front_.pose.orientation.x);
	pose.push_back(pose_msg_front_.pose.orientation.y);
	pose.push_back(pose_msg_front_.pose.orientation.z);
	node["pose_wheel_camera_front"]=pose;

	pose=YAML::Load("[]");
	pose.push_back(pose_msg_left_.pose.position.x);
	pose.push_back(pose_msg_left_.pose.position.y);
	pose.push_back(pose_msg_left_.pose.position.z);
	pose.push_back(pose_msg_left_.pose.orientation.w);
	pose.push_back(pose_msg_left_.pose.orientation.x);
	pose.push_back(pose_msg_left_.pose.orientation.y);
	pose.push_back(pose_msg_left_.pose.orientation.z);
	node["pose_wheel_camera_left"]=pose;

	pose=YAML::Load("[]");
	pose.push_back(pose_msg_rear_.pose.position.x);
	pose.push_back(pose_msg_rear_.pose.position.y);
	pose.push_back(pose_msg_rear_.pose.position.z);
	pose.push_back(pose_msg_rear_.pose.orientation.w);
	pose.push_back(pose_msg_rear_.pose.orientation.x);
	pose.push_back(pose_msg_rear_.pose.orientation.y);
	pose.push_back(pose_msg_rear_.pose.orientation.z);
	node["pose_wheel_camera_rear"]=pose;

	std::ofstream yaml_file("calib_extrinsic.yaml");
	yaml_file<<node;
	yaml_file.close();
}

void TagLocalization::TagCallBack_right(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
	if(msg->detections.size()==0) return;

	Eigen::Vector3d t;
	Eigen::Quaterniond q;

	// bundle_all
	// T_ci_right_[0]
	for(int i=0;i<msg->detections.size();i++)
	{
		t(0)=msg->detections[i].pose.pose.pose.position.x;
		t(1)=msg->detections[i].pose.pose.pose.position.y;
		t(2)=msg->detections[i].pose.pose.pose.position.z;
		q.w()=msg->detections[i].pose.pose.pose.orientation.w;
		q.x()=msg->detections[i].pose.pose.pose.orientation.x;
		q.y()=msg->detections[i].pose.pose.pose.orientation.y;
		q.z()=msg->detections[i].pose.pose.pose.orientation.z;
		q.normalize();

		Eigen::Isometry3d T;
		T.translation()=t;
		T.matrix().topLeftCorner(3,3)=q.toRotationMatrix();

		std::string bundle_name;
		if(msg->detections[i].id.size()==160) 
		{
			bundle_name="bundle_all";
		}
		else
		{
			bundle_name="bundle_"+std::to_string(msg->detections[i].id[0]);
		}

		T_ci_right_.insert(std::pair<std::string,Eigen::Isometry3d>(bundle_name,T));
	}

	if(T_ci_right_.find("bundle_all")==T_ci_right_.end()) return;

	Eigen::Isometry3d T=T_ci_right_.find("bundle_all")->second.inverse();
	Eigen::Matrix3d R=T.matrix().topLeftCorner(3,3);
	Eigen::Quaterniond qq(R);

	// msg to publish: /pose_wheel_camera_right 
	pose_msg_right_.header.stamp=ros::Time::now();
	pose_msg_right_.header.frame_id="bundle_all";
	pose_msg_right_.pose.position.x=T.translation()(0);
	pose_msg_right_.pose.position.y=T.translation()(1);
	pose_msg_right_.pose.position.z=T.translation()(2);
	pose_msg_right_.pose.orientation.w=qq.w();
	pose_msg_right_.pose.orientation.x=qq.x();
	pose_msg_right_.pose.orientation.y=qq.y();
	pose_msg_right_.pose.orientation.z=qq.z();

	// bundlea_all is observed by camera_right;
	double rmse_rot=0, rmse_trn=0, mean_ratio=0;
	std::cout<<std::endl<<"*****************************************"<<std::endl;
	std::cout<<"camera_right:"<<std::endl;
	for(iterTransform it=T_ci_right_.begin();it!=T_ci_right_.end();++it)
	{
		if(it->first=="bundle_all") continue;
		// T_bi=T_cb^{-1}*T_ci;
		Eigen::Isometry3d T_bi=T_ci_right_.find("bundle_all")->second.inverse()*it->second;

		Eigen::Isometry3d T_delta=T_bi.inverse()*T_bi_gt_.find(it->first)->second;
		Eigen::Matrix3d R_delta=T_delta.rotation();
		Eigen::Vector3d t_delta=T_delta.translation();

		double error_rot=acos((R_delta.trace()-1.0)/2.0)*180.0/M_PI;
		double error_trn=t_delta.norm();

		double error_trn_ratio=error_trn/T_bi_gt_.find(it->first)->second.translation().norm();

		std::cout<<it->first<<"\t"<<error_trn<<"\t"<<error_rot<<"\t"<<error_trn_ratio<<std::endl;

		rmse_rot+=error_rot*error_rot;
		rmse_trn+=error_trn*error_trn;
		mean_ratio+=error_trn_ratio;
	}

	rmse_rot/=T_ci_right_.size(); rmse_rot=sqrt(rmse_rot);
	rmse_trn/=T_ci_right_.size(); rmse_trn=sqrt(rmse_trn);
	mean_ratio/=T_ci_right_.size();

	std::cout<<std::endl;
	std::cout<<"rmse:\t"<<rmse_trn<<"\t"<<rmse_rot<<"\t"<<mean_ratio<<std::endl;
	std::cout<<std::endl;
}

void TagLocalization::TagCallBack_front(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
	if(msg->detections.size()==0) return;

	Eigen::Vector3d t;
	Eigen::Quaterniond q;

	// bundle_all
	// T_ci_front_[0]
	for(int i=0;i<msg->detections.size();i++)
	{
		t(0)=msg->detections[i].pose.pose.pose.position.x;
		t(1)=msg->detections[i].pose.pose.pose.position.y;
		t(2)=msg->detections[i].pose.pose.pose.position.z;
		q.w()=msg->detections[i].pose.pose.pose.orientation.w;
		q.x()=msg->detections[i].pose.pose.pose.orientation.x;
		q.y()=msg->detections[i].pose.pose.pose.orientation.y;
		q.z()=msg->detections[i].pose.pose.pose.orientation.z;
		q.normalize();

		Eigen::Isometry3d T;
		T.translation()=t;
		T.matrix().topLeftCorner(3,3)=q.toRotationMatrix();

		std::string bundle_name;
		if(msg->detections[i].id.size()==160) 
		{
			bundle_name="bundle_all";
		}
		else
		{
			bundle_name="bundle_"+std::to_string(msg->detections[i].id[0]);
		}

		T_ci_front_.insert(std::pair<std::string,Eigen::Isometry3d>(bundle_name,T));
	}

	if(T_ci_front_.find("bundle_all")==T_ci_front_.end()) return;

	Eigen::Isometry3d T=T_ci_front_.find("bundle_all")->second.inverse();
	Eigen::Matrix3d R=T.matrix().topLeftCorner(3,3);
	Eigen::Quaterniond qq(R);

	// msg to publish: /pose_wheel_camera_front 
	pose_msg_front_.header.stamp=ros::Time::now();
	pose_msg_front_.header.frame_id="bundle_all";
	pose_msg_front_.pose.position.x=T.translation()(0);
	pose_msg_front_.pose.position.y=T.translation()(1);
	pose_msg_front_.pose.position.z=T.translation()(2);
	pose_msg_front_.pose.orientation.w=qq.w();
	pose_msg_front_.pose.orientation.x=qq.x();
	pose_msg_front_.pose.orientation.y=qq.y();
	pose_msg_front_.pose.orientation.z=qq.z();

	// bundlea_all is observed by camera_front;
	double rmse_rot=0, rmse_trn=0, mean_ratio=0;
	std::cout<<std::endl<<"*****************************************"<<std::endl;
	std::cout<<"camera_front:"<<std::endl;
	for(iterTransform it=T_ci_front_.begin();it!=T_ci_front_.end();++it)
	{
		if(it->first=="bundle_all") continue;
		// T_bi=T_cb^{-1}*T_ci;
		Eigen::Isometry3d T_bi=T_ci_front_.find("bundle_all")->second.inverse()*it->second;

		Eigen::Isometry3d T_delta=T_bi.inverse()*T_bi_gt_.find(it->first)->second;
		Eigen::Matrix3d R_delta=T_delta.rotation();
		Eigen::Vector3d t_delta=T_delta.translation();

		double error_rot=acos((R_delta.trace()-1.0)/2.0)*180.0/M_PI;
		double error_trn=t_delta.norm();

		double error_trn_ratio=error_trn/T_bi_gt_.find(it->first)->second.translation().norm();

		std::cout<<it->first<<"\t"<<error_trn<<"\t"<<error_rot<<"\t"<<error_trn_ratio<<std::endl;

		rmse_rot+=error_rot*error_rot;
		rmse_trn+=error_trn*error_trn;
		mean_ratio+=error_trn_ratio;
	}

	rmse_rot/=T_ci_front_.size(); rmse_rot=sqrt(rmse_rot);
	rmse_trn/=T_ci_front_.size(); rmse_trn=sqrt(rmse_trn);
	mean_ratio/=T_ci_front_.size();

	std::cout<<std::endl;
	std::cout<<"rmse:\t"<<rmse_trn<<"\t"<<rmse_rot<<"\t"<<mean_ratio<<std::endl;
	std::cout<<std::endl;
}

void TagLocalization::TagCallBack_left(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
	if(msg->detections.size()==0) return;

	Eigen::Vector3d t;
	Eigen::Quaterniond q;

	// bundle_all
	// T_ci_left_[0]
	for(int i=0;i<msg->detections.size();i++)
	{
		t(0)=msg->detections[i].pose.pose.pose.position.x;
		t(1)=msg->detections[i].pose.pose.pose.position.y;
		t(2)=msg->detections[i].pose.pose.pose.position.z;
		q.w()=msg->detections[i].pose.pose.pose.orientation.w;
		q.x()=msg->detections[i].pose.pose.pose.orientation.x;
		q.y()=msg->detections[i].pose.pose.pose.orientation.y;
		q.z()=msg->detections[i].pose.pose.pose.orientation.z;
		q.normalize();

		Eigen::Isometry3d T;
		T.translation()=t;
		T.matrix().topLeftCorner(3,3)=q.toRotationMatrix();

		std::string bundle_name;
		if(msg->detections[i].id.size()==160) 
		{
			bundle_name="bundle_all";
		}
		else
		{
			bundle_name="bundle_"+std::to_string(msg->detections[i].id[0]);
		}

		T_ci_left_.insert(std::pair<std::string,Eigen::Isometry3d>(bundle_name,T));
	}

	if(T_ci_left_.find("bundle_all")==T_ci_left_.end()) return;

	Eigen::Isometry3d T=T_ci_left_.find("bundle_all")->second.inverse();
	Eigen::Matrix3d R=T.matrix().topLeftCorner(3,3);
	Eigen::Quaterniond qq(R);

	// msg to publish: /pose_wheel_camera_left 
	pose_msg_left_.header.stamp=ros::Time::now();
	pose_msg_left_.header.frame_id="bundle_all";
	pose_msg_left_.pose.position.x=T.translation()(0);
	pose_msg_left_.pose.position.y=T.translation()(1);
	pose_msg_left_.pose.position.z=T.translation()(2);
	pose_msg_left_.pose.orientation.w=qq.w();
	pose_msg_left_.pose.orientation.x=qq.x();
	pose_msg_left_.pose.orientation.y=qq.y();
	pose_msg_left_.pose.orientation.z=qq.z();

	// bundlea_all is observed by camera_left;
	double rmse_rot=0, rmse_trn=0, mean_ratio=0;
	std::cout<<std::endl<<"*****************************************"<<std::endl;
	std::cout<<"camera_left:"<<std::endl;
	for(iterTransform it=T_ci_left_.begin();it!=T_ci_left_.end();++it)
	{
		if(it->first=="bundle_all") continue;
		// T_bi=T_cb^{-1}*T_ci;
		Eigen::Isometry3d T_bi=T_ci_left_.find("bundle_all")->second.inverse()*it->second;

		Eigen::Isometry3d T_delta=T_bi.inverse()*T_bi_gt_.find(it->first)->second;
		Eigen::Matrix3d R_delta=T_delta.rotation();
		Eigen::Vector3d t_delta=T_delta.translation();

		double error_rot=acos((R_delta.trace()-1.0)/2.0)*180.0/M_PI;
		double error_trn=t_delta.norm();

		double error_trn_ratio=error_trn/T_bi_gt_.find(it->first)->second.translation().norm();

		std::cout<<it->first<<"\t"<<error_trn<<"\t"<<error_rot<<"\t"<<error_trn_ratio<<std::endl;

		rmse_rot+=error_rot*error_rot;
		rmse_trn+=error_trn*error_trn;
		mean_ratio+=error_trn_ratio;
	}

	rmse_rot/=T_ci_left_.size(); rmse_rot=sqrt(rmse_rot);
	rmse_trn/=T_ci_left_.size(); rmse_trn=sqrt(rmse_trn);
	mean_ratio/=T_ci_left_.size();

	std::cout<<std::endl;
	std::cout<<"rmse:\t"<<rmse_trn<<"\t"<<rmse_rot<<"\t"<<mean_ratio<<std::endl;
	std::cout<<std::endl;
}

void TagLocalization::TagCallBack_rear(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
	if(msg->detections.size()==0) return;

	Eigen::Vector3d t;
	Eigen::Quaterniond q;

	// bundle_all
	// T_ci_rear_[0]
	for(int i=0;i<msg->detections.size();i++)
	{
		t(0)=msg->detections[i].pose.pose.pose.position.x;
		t(1)=msg->detections[i].pose.pose.pose.position.y;
		t(2)=msg->detections[i].pose.pose.pose.position.z;
		q.w()=msg->detections[i].pose.pose.pose.orientation.w;
		q.x()=msg->detections[i].pose.pose.pose.orientation.x;
		q.y()=msg->detections[i].pose.pose.pose.orientation.y;
		q.z()=msg->detections[i].pose.pose.pose.orientation.z;
		q.normalize();

		Eigen::Isometry3d T;
		T.translation()=t;
		T.matrix().topLeftCorner(3,3)=q.toRotationMatrix();

		std::string bundle_name;
		if(msg->detections[i].id.size()==160) 
		{
			bundle_name="bundle_all";
		}
		else
		{
			bundle_name="bundle_"+std::to_string(msg->detections[i].id[0]);
		}

		T_ci_rear_.insert(std::pair<std::string,Eigen::Isometry3d>(bundle_name,T));
	}

	if(T_ci_rear_.find("bundle_all")==T_ci_rear_.end()) return;

	Eigen::Isometry3d T=T_ci_rear_.find("bundle_all")->second.inverse();
	Eigen::Matrix3d R=T.matrix().topLeftCorner(3,3);
	Eigen::Quaterniond qq(R);

	// msg to publish: /pose_wheel_camera_rear 
	pose_msg_rear_.header.stamp=ros::Time::now();
	pose_msg_rear_.header.frame_id="bundle_all";
	pose_msg_rear_.pose.position.x=T.translation()(0);
	pose_msg_rear_.pose.position.y=T.translation()(1);
	pose_msg_rear_.pose.position.z=T.translation()(2);
	pose_msg_rear_.pose.orientation.w=qq.w();
	pose_msg_rear_.pose.orientation.x=qq.x();
	pose_msg_rear_.pose.orientation.y=qq.y();
	pose_msg_rear_.pose.orientation.z=qq.z();

	// bundlea_all is observed by camera_rear;
	double rmse_rot=0, rmse_trn=0, mean_ratio=0;
	std::cout<<std::endl<<"*****************************************"<<std::endl;
	std::cout<<"camera_rear:"<<std::endl;
	for(iterTransform it=T_ci_rear_.begin();it!=T_ci_rear_.end();++it)
	{
		if(it->first=="bundle_all") continue;
		// T_bi=T_cb^{-1}*T_ci;
		Eigen::Isometry3d T_bi=T_ci_rear_.find("bundle_all")->second.inverse()*it->second;

		Eigen::Isometry3d T_delta=T_bi.inverse()*T_bi_gt_.find(it->first)->second;
		Eigen::Matrix3d R_delta=T_delta.rotation();
		Eigen::Vector3d t_delta=T_delta.translation();

		double error_rot=acos((R_delta.trace()-1.0)/2.0)*180.0/M_PI;
		double error_trn=t_delta.norm();

		double error_trn_ratio=error_trn/T_bi_gt_.find(it->first)->second.translation().norm();

		std::cout<<it->first<<"\t"<<error_trn<<"\t"<<error_rot<<"\t"<<error_trn_ratio<<std::endl;

		rmse_rot+=error_rot*error_rot;
		rmse_trn+=error_trn*error_trn;
		mean_ratio+=error_trn_ratio;
	}

	rmse_rot/=T_ci_rear_.size(); rmse_rot=sqrt(rmse_rot);
	rmse_trn/=T_ci_rear_.size(); rmse_trn=sqrt(rmse_trn);
	mean_ratio/=T_ci_rear_.size();

	std::cout<<std::endl;
	std::cout<<"rmse:\t"<<rmse_trn<<"\t"<<rmse_rot<<"\t"<<mean_ratio<<std::endl;
	std::cout<<std::endl;
}
