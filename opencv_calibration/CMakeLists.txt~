cmake_minimum_required(VERSION 2.6)  
set (CMAKE_CXX_STANDARD 11)
project(opencv_test)  
find_package(OpenCV REQUIRED)  
find_package(yaml-cpp REQUIRED)

add_executable(calibration fisheye_calibration.cpp)  
#add_executable(calibration camera_calibration.cpp)  

target_link_libraries(calibration ${OpenCV_LIBS} yaml-cpp)  
cmake_minimum_required(VERSION 2.8)
