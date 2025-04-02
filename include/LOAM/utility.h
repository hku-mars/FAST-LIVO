#pragma once

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>  //
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/msg/cloud_msg.h"

#define SAVE_PCD false

// image_preojection
#define IMAGE_PROJECTION_DEBUG 1
#define EXTRACT_GROUND 0
#define EXTRACT_NOISE_BY_ROW_AND_COL 1 // 0 means BY_COL_ONLY, 1 means BY_ROW_AND_COL
#define CLOUD_SEGEMENTATION 0
// feature_association
#define ROW_ONLY_NOISE 0

// defination
typedef pcl::PointXYZINormal BaseP;
typedef pcl::PointCloud<PointType> BasePC;
typedef pcl::PointXYZINormal RsP;
typedef pcl::PointCloud<PointType> RsPC;
#define D2R (M_PI / 180.0)
#define R2D (180.0 / M_PI)

// 
struct HardWareParameter {
  // 
  std::string lidar_type = "m1p";
  // MEMS setup
  double MINIMUM_RANGE = 1;
  double MAX_RANGE = 100;
  int N_SCAN = 126;
  int Horizon_SCAN = 600;  // 120/0.2
  float ang_res_x = 0.2;
  float ang_res_y = 0.2;
  double ang_bottom = 12.5;
  int groundScanInd = 80;
  // IMUsetup
  int imuQueLength = 200;
};

// 分割
struct SegmentationParameter {
  // image projection
  double sensorMinimumRange = 1;
  int column_drop_num = 0;

  // segmentation
  double z_thresh = -1.0;
  double segmentTheta;       // decrese this value may improve accuracy
  int segmentValidPointNum = 5;  // 竖杆状特征最小点数
  int segmentValidLineNum = 10;  // 竖杆状特征最小行数
  double segmentAlphaX;          // = ang_res_x * D2R;
  double segmentAlphaY;          // = ang_res_y * D2R;
};

struct NoiseFilterParameter {
  double curva_thresh;
  int radius;
  double recall_map_plane_thresh;
  float fit_plane_dist_thresh;
  double recall_scan_plane_thresh;

  std::vector<double> pca_thresh;
};

enum class MODULE_STATUS {
  INIT = 0,
  LOW_ACCURACY = 1,
  NORMAL = 2,
  LOST = 3,
};

// segmentation msg
struct SegmentationInfo {
  // Header header

  std::vector<int> startRingIndex; // N_SCAN维
  std::vector<int> endRingIndex;

  double startOrientation;
  double endOrientation;
  double orientationDiff;
  RsPC start_end_cloud;

  // 非dense数组为 N_SCAN * Horizon_SCAN 维
  std::vector<bool>
      segmentedCloudGroundFlag;  // true - ground point, false - other points
  std::vector<uint> segmentedCloudColInd;  // point column index in range image
  std::vector<uint> segmentedCloudRowInd;  // point row index in range image
  std::vector<double> segmentedCloudRange;  // point range
  // std::vector<uint> segmentedCloudRing;    // point Ring
  std::vector<double> segmentedCloudTime;   // point timestamp

  // dense数组，N维，N是有效点数
  std::vector<double> segmentedCloudTimeDense;   // point timestamp
};

// 分割结果
struct SegmentationResult {
  double timestamp;
  BasePC fullCloudIndex;
  BasePC fullCloudRange;

  BasePC groundCloud;
  BasePC segmentedCloud;  // segmented_cloud话题
  BasePC segmentedCloudPure;
  cv::Mat rangeMat;

  BasePC outlierCloud;  // outlier_cloud话题
  RsPC lidarCloud;
  RsPC lidarCloudDense;

  SegmentationInfo segMsg;  // segmented_cloud_info话题

  double time_cost;
};

// 特征点云
struct FeatureClouds {
  double timestamp;
  BasePC pickedCloud; // 存储挑选结果
  BasePC segmentedCloud;
  BasePC semanticCloud;
  BasePC outlierCloud;
  BasePC validCloud;
};