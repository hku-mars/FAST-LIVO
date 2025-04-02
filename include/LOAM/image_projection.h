// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-timestamp.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.
#pragma once

#include "LOAM/utility.h"
#include "common_lib.h"

class ImageProjection {
  // user interface
 public:
  SegmentationResult cloudHandler(const RsPC::Ptr lidar_cloud) {
    *lidarCloud = *lidar_cloud;
    // 3. Range image projection
    projectPointCloud();
    // 4. Mark ground points
    if (EXTRACT_GROUND) {
      groundRemoval();
    }
    // 5. Point cloud segmentation
    cloudSegmentation();
    // 6. Publish all clouds
    setSegmentationResult();
    // 7. Reset parameters for next iteration
    resetParameters();

    return segClouds;
  }

 public:
  ImageProjection(const HardWareParameter &HW_param,
                  const SegmentationParameter &IM_param) {
    loadParams(HW_param, IM_param);
    allocateMemory();
    resetParameters();
  }
  ~ImageProjection() {}

 private:
  // config
  HardWareParameter HW_param;
  SegmentationParameter IM_param;

  // MEMS setup
  double MINIMUM_RANGE = 1;
  double MAX_RANGE = 100;
  int N_SCAN = 126;
  int Horizon_SCAN = 600;
  float ang_res_x = 0.2;
  float ang_res_y = 0.2;
  double ang_bottom = 12.5;
  int groundScanInd = 80;

  // image projection
  double sensorMinimumRange = 1;
  int column_drop_num = 0;

  // segmentation
  double z_thresh = -1.0;
  double segmentTheta = 20.0 / D2R;  // decrese this value may improve accuracy
  int segmentValidPointNum = 5;      // 竖杆状特征最小点数
  int segmentValidLineNum = 10;      // 竖杆状特征最小行数
  double segmentAlphaX = ang_res_x / D2R;
  double segmentAlphaY = ang_res_y / D2R;

  // algo
  SegmentationResult segClouds;
  RsPC::Ptr lidarCloud;
  RsPC::Ptr lidarCloudDense;

  BasePC::Ptr fullCloudIndex;  // projected raw cloud, but saved
                               // in the form of 1-D matrix
  BasePC::Ptr
      fullCloudRange;  // same as fullCloudIndex, but with intensity - range

  BasePC::Ptr groundCloud;
  BasePC::Ptr segmentedCloud;
  BasePC::Ptr segmentedCloudPure;
  BasePC::Ptr outlierCloud;
  BasePC::Ptr laserScanCloud;

  PointType nanPoint;  // fill in fullCloudIndex at each iteration

  cv::Mat rangeMat;   // range matrix for range image
  cv::Mat labelMat;   // label matrix for segmentaiton marking
  cv::Mat groundMat;  // ground matrix for ground cloud marking
  int labelCount;

  SegmentationInfo segMsg;  // info of segmented cloud

  std::vector<std::pair<int8_t, int8_t>>
      neighborIterator;  // neighbor iterator for segmentaiton process

  uint16_t *allPushedIndX;  // array for tracking points of a segmented object
  uint16_t *allPushedIndY;

  uint16_t *queueIndX;  // array for breadth-first search process of
                        // segmentation, for speed
  uint16_t *queueIndY;

 private:
  //  初始化
  void loadParams(const HardWareParameter &HW_param_,
                  const SegmentationParameter &IM_param_) {
    HW_param = HW_param_;
    IM_param = IM_param_;
    MINIMUM_RANGE = HW_param.MINIMUM_RANGE;
    MAX_RANGE = HW_param.MAX_RANGE;
    N_SCAN = HW_param.N_SCAN;
    Horizon_SCAN = HW_param.Horizon_SCAN;
    ang_res_x = HW_param.ang_res_x;
    ang_res_y = HW_param.ang_res_y;
    ang_bottom = HW_param.ang_bottom;
    groundScanInd = HW_param.groundScanInd;

    sensorMinimumRange = IM_param.sensorMinimumRange;
    column_drop_num = IM_param.column_drop_num;
    z_thresh = IM_param.z_thresh;
    segmentTheta = IM_param.segmentTheta;
    segmentValidPointNum = IM_param.segmentValidPointNum;
    segmentValidLineNum = IM_param.segmentValidLineNum;
    segmentAlphaX = IM_param.segmentAlphaX;
    segmentAlphaY = IM_param.segmentAlphaY;
  }

  void allocateMemory() {
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    fullCloudIndex.reset(new BasePC());
    fullCloudRange.reset(new BasePC());

    groundCloud.reset(new BasePC());
    segmentedCloud.reset(new BasePC());
    segmentedCloudPure.reset(new BasePC());
    outlierCloud.reset(new BasePC());

    laserScanCloud.reset(new BasePC());
    lidarCloud.reset(new RsPC());
    lidarCloudDense.reset(new RsPC());

    fullCloudIndex->points.resize(N_SCAN * Horizon_SCAN);
    fullCloudRange->points.resize(N_SCAN * Horizon_SCAN);

    segMsg.startRingIndex.assign(N_SCAN, 0);
    segMsg.endRingIndex.assign(N_SCAN, 0);

    segMsg.segmentedCloudGroundFlag.assign(N_SCAN * Horizon_SCAN, false);
    segMsg.segmentedCloudColInd.assign(N_SCAN * Horizon_SCAN, 0);
    segMsg.segmentedCloudRowInd.assign(N_SCAN * Horizon_SCAN, 0);
    segMsg.segmentedCloudRange.assign(N_SCAN * Horizon_SCAN, 0);
    segMsg.segmentedCloudTimeDense.assign(0, 0);

    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1, neighbor.second = 0;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0, neighbor.second = 1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0, neighbor.second = -1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 1, neighbor.second = 0;
    neighborIterator.push_back(neighbor);

    allPushedIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN * Horizon_SCAN];

    queueIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN * Horizon_SCAN];
  }

  void resetParameters() {
    groundCloud->clear();
    groundCloud->reserve(groundScanInd * Horizon_SCAN);
    segmentedCloud->clear();
    segmentedCloud->reserve(N_SCAN * Horizon_SCAN);
    segmentedCloudPure->clear();
    segmentedCloudPure->reserve(N_SCAN * Horizon_SCAN);
    outlierCloud->clear();
    laserScanCloud->clear();
    laserScanCloud->reserve(N_SCAN * Horizon_SCAN);

    lidarCloud->clear();
    lidarCloudDense->clear();
    lidarCloudDense->points.resize(N_SCAN * Horizon_SCAN);
    // segMsg.segmentedCloudTimeDense.clear();
    segMsg.start_end_cloud.clear();

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    std::fill(fullCloudIndex->points.begin(), fullCloudIndex->points.end(),
              nanPoint);
    std::fill(fullCloudRange->points.begin(), fullCloudRange->points.end(),
              nanPoint);
  }

  // 投影
  void projectPointCloud() {
    // range image projection
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize;
    PointType thisPoint;

    double max_yaw = -1000;
    double min_yaw = 1000;
    double max_pitch = -1000;
    double min_pitch = 1000;
    int max_col = -10000;
    int min_col = 10000;
    int max_row = -10000;
    int min_row = 10000;
    cloudSize = lidarCloud->points.size();
    int cnt = 0;
    for (size_t i = 0; i < cloudSize; ++i) {
      thisPoint.x = lidarCloud->points[i].x;
      thisPoint.y = lidarCloud->points[i].y;
      thisPoint.z = lidarCloud->points[i].z;

      float x2_y2 = thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y;
      // rowIdn
      verticalAngle = atan2(thisPoint.z, sqrt(x2_y2)) * R2D;
      min_pitch = verticalAngle < min_pitch ? verticalAngle : min_pitch;
      max_pitch = verticalAngle > max_pitch ? verticalAngle : max_pitch;
      rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
      if (rowIdn < 0 || rowIdn >= N_SCAN) {
        continue;
      }
      max_col = rowIdn > max_row ? rowIdn : max_row;
      min_col = rowIdn < min_row ? rowIdn : min_row;

      // columnIdn
      horizonAngle = atan2(thisPoint.x, thisPoint.y) * R2D;
      min_yaw = horizonAngle < min_yaw ? horizonAngle : min_yaw;
      max_yaw = horizonAngle > max_yaw ? horizonAngle : max_yaw;
      columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
      if (columnIdn < column_drop_num ||
          columnIdn >= Horizon_SCAN - column_drop_num) {
        continue;
      }
      max_row = columnIdn > max_row ? columnIdn : max_row;
      min_row = columnIdn < min_row ? columnIdn : min_row;

      // range
      range = sqrt(x2_y2 + thisPoint.z * thisPoint.z);
      if (range < sensorMinimumRange) {
        continue;
      }
      rangeMat.at<float>(rowIdn, columnIdn) = range;

      // 无序化点云
      thisPoint.intensity = (float)rowIdn + (float)columnIdn / (10000);
      thisPoint.normal_x = rowIdn; // DEBUG
      thisPoint.normal_y = columnIdn;
      thisPoint.normal_z = range;
      thisPoint.curvature = lidarCloud->points[i].curvature;
      index = columnIdn + rowIdn * Horizon_SCAN;
      fullCloudIndex->points[index] = thisPoint;
      fullCloudRange->points[index] = thisPoint;
      fullCloudRange->points[index].intensity = range;
      laserScanCloud->push_back(thisPoint);

      lidarCloudDense->points[index] = lidarCloud->points[i]; // 将点云非dense化
    }
    LDEBUG << "[ Image projection ] size: " << cloudSize << ", yaw range: " << min_yaw << " to " << max_yaw
           << ", pitch range: " << min_pitch << " to " << max_pitch
           << ", col range: " << min_col << " to " << max_col
           << ", row range: " << min_row << " to " << max_row << REND;
    // // 将深度图归一化到0-255的范围
    // cv::Mat normalizedDepth;
    // cv::normalize(rangeMat, normalizedDepth, 0, 255, cv::NORM_MINMAX, CV_8U);
    // cv::imwrite(DEBUG_FILE_DIR("depth_img.jpg"), normalizedDepth);
    // imshow("depth_img", rangeMat);    
  }

  // 分割
  void groundRemoval() {
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    // groundMat
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    // 第一次选取
    for (size_t j = 0; j < Horizon_SCAN; ++j) {
      for (size_t i = 0; i < groundScanInd; ++i) {
        if (fullCloudIndex->points[j + (i)*Horizon_SCAN].intensity == -1) {
          groundMat.at<int8_t>(i, j) = -1;
          continue;
        }
        if (fullCloudIndex->points[j + (i)*Horizon_SCAN].z < z_thresh) {
          groundMat.at<int8_t>(i, j) = 1;
        }
      }
    }
    // TODO: 从第一次的非地面点集筛选一些加入地面点集

    // extract ground cloud (groundMat == 1)
    // mark entry that doesn't need to label (ground and invalid point) for
    // segmentation note that ground remove is from 0~N_SCAN-1, need rangeMat
    // for mark label matrix for the 16th scan
    for (size_t i = 0; i < N_SCAN; ++i) {
      for (size_t j = 0; j < Horizon_SCAN; ++j) {
        if (groundMat.at<int8_t>(i, j) == 1 ||
            rangeMat.at<float>(i, j) == FLT_MAX) {
          labelMat.at<int>(i, j) = -1;
        }
      }
    }
  }

  void cloudSegmentation() {
#if CLOUD_SEGEMENTATION
    // segmentation process
    for (size_t i = 0; i < N_SCAN; ++i)
      for (size_t j = 0; j < Horizon_SCAN; ++j)
        if (labelMat.at<int>(i, j) == 0) labelComponents(i, j);

    int sizeOfSegCloud = 0;
    // extract segmented cloud for lidar odometry
    for (size_t i = 0; i < N_SCAN; ++i) {
      segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

      for (size_t j = 0; j < Horizon_SCAN; ++j) {
        if (labelMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1) {
          // outliers that will not be used for optimization (always continue)
          if (labelMat.at<int>(i, j) == 999999) {
            continue;
            // if (i > groundScanInd && j % 5 == 0) {
            //   outlierCloud->push_back(
            //       fullCloudIndex->points[j + i * Horizon_SCAN]);
            //   continue;
            // } else {
            //   continue;
            // }
          }
          // majority of ground points ares skipped TODO(rum): need more gnd?
          if (groundMat.at<int8_t>(i, j) == 1) {
            if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5) continue;
          }
          // mark ground points so they will not be considered as edge
          // features later
          segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] =
              groundMat.at<int8_t>(i, j);
          // mark the points' column index for marking occlusion later
          segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
          segMsg.segmentedCloudRowInd[sizeOfSegCloud] = i;
          // save range info
          segMsg.segmentedCloudRange[sizeOfSegCloud] =
              static_cast<double>(rangeMat.at<float>(i, j));
          // save seg cloud
          segmentedCloud->push_back(
              fullCloudIndex->points[j + i * Horizon_SCAN]);
          // size of seg cloud
          ++sizeOfSegCloud;
        }
      }

      segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
      // if (segMsg.endRingIndex[i] - segMsg.startRingIndex[i] < 10) {
      //   LWARNING << "scan " << i << " end=" << segMsg.endRingIndex[i]
      //            << "start=" << segMsg.startRingIndex[i] << REND;
      // }

    }
#else
    int sizeOfSegCloud = 0;
    for (size_t i = 0; i < N_SCAN; ++i)
      for (size_t j = 0; j < Horizon_SCAN; ++j) {
        if (rangeMat.at<float>(i, j) == FLT_MAX)
          continue;
        segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
        segMsg.segmentedCloudRowInd[sizeOfSegCloud] = i;
        segMsg.segmentedCloudRange[sizeOfSegCloud] =
            static_cast<double>(rangeMat.at<float>(i, j));
        segmentedCloud->push_back(fullCloudIndex->points[j + i * Horizon_SCAN]);
        ++sizeOfSegCloud;
      }
#endif
  }

  void labelComponents(int row, int col) {
    // use std::queue std::vector std::deque will slow the program down
    // greatly
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY;
    // bool lineCountFlag[N_SCAN] = {false};
    std::vector<bool> lineCountFlag(N_SCAN, false);

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;

    while (queueSize > 0) {
      // Pop point
      fromIndX = queueIndX[queueStartInd];
      fromIndY = queueIndY[queueStartInd];
      --queueSize;
      ++queueStartInd;
      // Mark popped point
      labelMat.at<int>(fromIndX, fromIndY) = labelCount;
      // Loop through all the neighboring grids of popped grid
      for (auto iter = neighborIterator.begin(); iter != neighborIterator.end();
           ++iter) {
        // new index
        thisIndX = fromIndX + (*iter).first;
        thisIndY = fromIndY + (*iter).second;
        // index should be within the boundary
        if (thisIndX < 0 || thisIndX >= N_SCAN) continue;
        // at range image margin (left or right side)
        if (thisIndY < 0) thisIndY = Horizon_SCAN - 1;
        if (thisIndY >= Horizon_SCAN) thisIndY = 0;
        // prevent infinite loop (caused by put already examined point back)
        if (labelMat.at<int>(thisIndX, thisIndY) != 0) continue;

        d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                      rangeMat.at<float>(thisIndX, thisIndY));
        d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                      rangeMat.at<float>(thisIndX, thisIndY));

        if ((*iter).first == 0)
          alpha = segmentAlphaX;
        else
          alpha = segmentAlphaY;

        angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

        if (angle > segmentTheta) {
          queueIndX[queueEndInd] = thisIndX;
          queueIndY[queueEndInd] = thisIndY;
          ++queueSize;
          ++queueEndInd;

          labelMat.at<int>(thisIndX, thisIndY) = labelCount;
          lineCountFlag[thisIndX] = true;

          allPushedIndX[allPushedIndSize] = thisIndX;
          allPushedIndY[allPushedIndSize] = thisIndY;
          ++allPushedIndSize;
        }
      }
    }

    // check if this segment is valid
    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
      feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum) {
      int lineCount = 0;
      for (size_t i = 0; i < N_SCAN; ++i)
        if (lineCountFlag[i] == true) ++lineCount;
      if (lineCount >= segmentValidLineNum) feasibleSegment = true;
    }
    // segment is valid, mark these points
    if (feasibleSegment == true) {
      ++labelCount;
    } else {  // segment is invalid, mark these points
      for (size_t i = 0; i < allPushedIndSize; ++i) {
        labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
      }
    }
  }

  // 输出
  void setSegmentationResult() {
    LDEBUG << "[ Image projection] valid raw ratio=" << laserScanCloud->size()
           << "/" << lidarCloud->size() << "="
           << laserScanCloud->size() / double(lidarCloud->size()) << "";

    // 调试
    int valid_pt_num = groundScanInd * Horizon_SCAN;
    if (1) {
      for (size_t i = 0; i <= groundScanInd; ++i) {
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
          if (groundMat.at<int8_t>(i, j) == 1)
            groundCloud->push_back(
                fullCloudIndex->points[j + i * Horizon_SCAN]);
          // if (labelMat.at<int>(i, j) == -1 ||
          //     labelMat.at<int>(i, j) == 999999) {
          //   valid_pt_num--;
          // }
        }
      }
    }

    // if (groundCloud->size() < 100) {
    //   LERROR << ", ground not enough, size: " << groundCloud->size();
    // } else {
    //   LDEBUG << ", gnd num: " << groundCloud->size()
    //          << " total: " << valid_pt_num
    //          << " ratio: " << double(groundCloud->size()) / (valid_pt_num);
    // }
    // seg
    if (IMAGE_PROJECTION_DEBUG) {
      for (size_t i = 0; i < N_SCAN; ++i) {
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
          if (labelMat.at<int>(i, j) > 0 && labelMat.at<int>(i, j) != 999999) {
            segmentedCloudPure->push_back(
                fullCloudIndex->points[j + i * Horizon_SCAN]);
            segmentedCloudPure->points.back().intensity =
                labelMat.at<int>(i, j);
          }
        }
      }
    }
    LDEBUG << ", segmentedCloud: " << segmentedCloud->size()
           << " pure(no gnd): " << segmentedCloudPure->size() << ", ratio="
           << double(segmentedCloud->size()) / laserScanCloud->size() << REND;

    // set result
    segClouds.fullCloudIndex = *fullCloudIndex;
    // segClouds.fullCloudRange = *fullCloudRange;
    segClouds.groundCloud = *groundCloud;  // for debug
    segClouds.segmentedCloud = *segmentedCloud;
    segClouds.rangeMat = rangeMat.clone();
    if (IMAGE_PROJECTION_DEBUG) {
      segClouds.segmentedCloudPure = *segmentedCloudPure;  // for debug
      segClouds.outlierCloud = *outlierCloud;              // for debug
      segClouds.lidarCloud = *lidarCloud;                  // for debug
      segClouds.lidarCloudDense = *lidarCloudDense;        // for debug
    }
    segClouds.segMsg = segMsg;
  }
};
