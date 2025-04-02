#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "lidar_msg.h"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef PointCloudXYZI::Ptr CloudPtr;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
typedef PointCloudXYZRGB::Ptr RGBCloudPtr;

typedef pcl::PointCloud<robosense::Point> RSCloud;
