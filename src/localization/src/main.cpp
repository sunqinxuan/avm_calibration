/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2021-09-08 16:17
#
# Filename: main.cpp
#
# Description: 
#
************************************************/

#include "localize.h"

using namespace std;
int main(int argc, char *argv[])
{
//	Eigen::Matrix3d R=Eigen::Matrix3d::Zero();
//	R(1,0)=1;
//	R(0,1)=1;
//	R(2,2)=1;
//	cout<<R<<endl;
//	
//	Eigen::Quaterniond q(R);
//	cout<<endl<<"q= "<<q.w()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<endl;
//	return 0;

	
	ros::init(argc, argv, "localize", ros::init_options::AnonymousName);

	if(!ros::ok())
	{
		return 0;
	}

	TagLocalization tag_localize;

	ros::Rate loop_rate(10); // 10Hz
	ros::spinOnce();

	while(ros::ok())
	{
		// publish 
		tag_localize.publishPose();

		ros::spinOnce();
		loop_rate.sleep();
	}

//	tag_localize.run();
//	ros::spin();

	return 0;

}

