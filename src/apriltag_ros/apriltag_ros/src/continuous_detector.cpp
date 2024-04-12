/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  camera_image_subscriber_right_ = it_->subscribeCamera("image_rect_right", 1, &ContinuousDetector::imageCallback_right, this);
  camera_image_subscriber_front_ = it_->subscribeCamera("image_rect_front", 1, &ContinuousDetector::imageCallback_front, this);
  camera_image_subscriber_rear_  = it_->subscribeCamera("image_rect_rear",  1, &ContinuousDetector::imageCallback_rear, this);
  camera_image_subscriber_left_  = it_->subscribeCamera("image_rect_left",  1, &ContinuousDetector::imageCallback_left, this);

  tag_detections_publisher_right_ = nh.advertise<AprilTagDetectionArray>("tag_detections_right", 1);
  tag_detections_publisher_front_ = nh.advertise<AprilTagDetectionArray>("tag_detections_front", 1);
  tag_detections_publisher_rear_  = nh.advertise<AprilTagDetectionArray>("tag_detections_rear", 1);
  tag_detections_publisher_left_  = nh.advertise<AprilTagDetectionArray>("tag_detections_left", 1);

  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_right_ = it_->advertise("tag_detections_image_right", 1);
    tag_detections_image_publisher_front_ = it_->advertise("tag_detections_image_front", 1);
    tag_detections_image_publisher_rear_  = it_->advertise("tag_detections_image_rear", 1);
    tag_detections_image_publisher_left_  = it_->advertise("tag_detections_image_left", 1);
  }
}

void ContinuousDetector::imageCallback_right (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_right_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_right_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  //std::cout<<"camera_right camera_info: "<<std::endl;
  //std::cout<<"K: ";
  //for(int i=0;i<camera_info->K.size();i++) std::cout<<camera_info->K[i]<<" ";
  //std::cout<<std::endl;
  //std::cout<<"D: ";
  //for(int i=0;i<camera_info->D.size();i++) std::cout<<camera_info->D[i]<<" ";
  //std::cout<<std::endl<<std::endl;

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the image
  cv_bridge::CvImagePtr cv_image_;
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_right_.publish(tag_detector_->detectTags(cv_image_,camera_info));

  // bundle_32
  //geometry_msgs::PoseStamped pose;
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=4.7;
  //pose.pose.position.y=-2.75;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::Stamped<tf::Transform> tag_transform;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_32"));

  // bundle_48
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=2.4;
  //pose.pose.position.y=-2.75;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_48"));

  // bundle_64
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=0.0;
  //pose.pose.position.y=-2.75;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_64"));

  // bundle_80
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=-2.3;
  //pose.pose.position.y=-2.75;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_80"));


  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_right_.publish(cv_image_->toImageMsg());
  }
}

void ContinuousDetector::imageCallback_front (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_front_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_front_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  //std::cout<<"camera_front camera_info: "<<std::endl;
  //std::cout<<"K: ";
  //for(int i=0;i<camera_info->K.size();i++) std::cout<<camera_info->K[i]<<" ";
  //std::cout<<std::endl;
  //std::cout<<"D: ";
  //for(int i=0;i<camera_info->D.size();i++) std::cout<<camera_info->D[i]<<" ";
  //std::cout<<std::endl<<std::endl;

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the image
  cv_bridge::CvImagePtr cv_image_;
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_front_.publish(tag_detector_->detectTags(cv_image_,camera_info));

  // bundle_0
  //geometry_msgs::PoseStamped pose;
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=4.7;
  //pose.pose.position.y=1.25;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::Stamped<tf::Transform> tag_transform;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_0"));

  // bundle_16
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=4.7;
  //pose.pose.position.y=-0.75;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_16"));

  // bundle_32
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=4.7;
  //pose.pose.position.y=-2.75;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_32"));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_front_.publish(cv_image_->toImageMsg());
  }
}

void ContinuousDetector::imageCallback_left (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_left_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_left_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  //std::cout<<"camera_left camera_info: "<<std::endl;
  //std::cout<<"K: ";
  //for(int i=0;i<camera_info->K.size();i++) std::cout<<camera_info->K[i]<<" ";
  //std::cout<<std::endl;
  //std::cout<<"D: ";
  //for(int i=0;i<camera_info->D.size();i++) std::cout<<camera_info->D[i]<<" ";
  //std::cout<<std::endl<<std::endl;

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the image
  cv_bridge::CvImagePtr cv_image_;
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_left_.publish(tag_detector_->detectTags(cv_image_,camera_info));

  // bundle_0
  //geometry_msgs::PoseStamped pose;
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=4.7;
  //pose.pose.position.y=1.25;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::Stamped<tf::Transform> tag_transform;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_0"));

  // bundle_144
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=2.4;
  //pose.pose.position.y=1.25;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_144"));

  // bundle_128
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=0.0;
  //pose.pose.position.y=1.25;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_128"));

  // bundle_112
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=-2.3;
  //pose.pose.position.y=1.25;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_112"));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_left_.publish(cv_image_->toImageMsg());
  }
}

void ContinuousDetector::imageCallback_rear (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_rear_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_rear_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  //std::cout<<"camera_rear camera_info: "<<std::endl;
  //std::cout<<"K: ";
  //for(int i=0;i<camera_info->K.size();i++) std::cout<<camera_info->K[i]<<" ";
  //std::cout<<std::endl;
  //std::cout<<"D: ";
  //for(int i=0;i<camera_info->D.size();i++) std::cout<<camera_info->D[i]<<" ";
  //std::cout<<std::endl<<std::endl;

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the image
  cv_bridge::CvImagePtr cv_image_;
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_rear_.publish(tag_detector_->detectTags(cv_image_,camera_info));

  // bundle_80
  //geometry_msgs::PoseStamped pose;
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=-2.3;
  //pose.pose.position.y=-2.75;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::Stamped<tf::Transform> tag_transform;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_80"));

  // bundle_96
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=-2.3;
  //pose.pose.position.y=-0.75;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_96"));

  // bundle_112
  //pose.header=cv_image_->header;
  //pose.header.frame_id="bundle_all";
  //pose.pose.position.x=-2.3;
  //pose.pose.position.y=1.25;
  //pose.pose.position.z=0;
  //pose.pose.orientation.w=1;
  //pose.pose.orientation.x=0;
  //pose.pose.orientation.y=0;
  //pose.pose.orientation.z=0;
  //tf::poseStampedMsgToTF(pose,tag_transform);
  //tf_pub_.sendTransform(tf::StampedTransform(tag_transform,tag_transform.stamp_,
		//	  								 "bundle_all","bundle_112"));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_rear_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros
