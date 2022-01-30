#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list
// for Eigen
#include <Eigen/Core>
using Eigen::Vector2d;
using Eigen::Vector3d;




// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//for pcl
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

// std 
#include <vector> 
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include "math.h"
#include <stdio.h>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>


//ros
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include "/home/lie/ros_test_ws/src/msf/diff_msgs/WheelSpeedStatus.h"
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//QrCode
#include "/home/lie/ros_test_ws/src/msf/qrc_msgs/QuickResponseCode.h"

using namespace std; 
#endif
