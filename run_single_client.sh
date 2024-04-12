#!/bin/sh

path="/home/sun/AVP-SLAM/OCamCalib/undistortFunctions/build"

source devel/setup.bash

roslaunch apriltag_ros single_image_client.launch image_save_path:=${path}/output.jpg image_load_path:=${path}/test_fisheye.jpg

# test_fisheye.jpg
# undistorted_perspective.jpg
 
