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
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.
#pragma once

#include "utility.h"

class FeatureAssociation {
  // user interface
 public:
  FeatureClouds CloudHandler(const SegmentationResult &msg, double ts) {
    FA_res.timestamp = ts;
    prepare(msg);
    calculateSmoothness();
    markOccludedPoints();
    FA_res.segmentedCloud = *segmentedCloud;
    return FA_res;
  }

 public:
   FeatureAssociation(const HardWareParameter &HW_param,
                      const NoiseFilterParameter &NF_param) {
     loadParams(HW_param, NF_param);
     initializationValue();
   }
   ~FeatureAssociation(){};

 private:
  // result
  FeatureClouds FA_res;
  BasePC::Ptr segmentedCloud;
  cv::Mat rangeMat;

  //  config
  int N_SCAN;
  int Horizon_SCAN;

  SegmentationInfo segInfo;

  // boundary filter
  const int col_drop_num = 1; // 离群点膨胀行列数，丢弃周围的行列
  const int row_drop_num = 1;
  int cloudSize, row, col;
  cv::Mat ROI_rm, d_ROI_rm, near_d_ROI_rm;
  int D_ROW, D_COL;
  double B_depth_jump_thresh, A_depth_jump_thresh;
  NoiseFilterParameter nf_p;
  std::vector<double> cloud_abs_range_diff;
  cv::Mat neighbor_picked_mat;
  Eigen::MatrixXi index_map;
  // recall plane
  int row_used_num{2}, col_used_num{2};
  const float max_valid_range = 100.;

 private:
   void loadParams(const HardWareParameter &HW_param,
                   const NoiseFilterParameter &NF_param) {
     N_SCAN = HW_param.N_SCAN;
     Horizon_SCAN = HW_param.Horizon_SCAN;
     nf_p = NF_param;
   }
  void initializationValue() {
    segmentedCloud.reset(new BasePC());
    neighbor_picked_mat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8SC1, cv::Scalar(0));

    D_ROW = row_used_num * 2 + 1;
    D_COL = col_used_num * 2 + 1;

    ROI_rm = cv::Mat::zeros(D_ROW, D_COL, CV_32F);
    d_ROI_rm = cv::Mat::zeros(D_ROW, D_COL, CV_32F);
    near_d_ROI_rm = cv::Mat(3, 3, CV_32F, max_valid_range); // 8邻域
  }

  void prepare(const SegmentationResult &msg)
  {
    *segmentedCloud = msg.segmentedCloud;
    cloudSize = segmentedCloud->points.size();
    segInfo = msg.segMsg;
    rangeMat = msg.rangeMat;

    FA_res.outlierCloud.points.clear();
    FA_res.outlierCloud.points.reserve(segmentedCloud->size());
    FA_res.validCloud.points.clear();
    FA_res.validCloud.points.reserve(segmentedCloud->size());
    FA_res.semanticCloud.points.clear();
    FA_res.semanticCloud.points.reserve(segmentedCloud->size());

    index_map.resize(N_SCAN, Horizon_SCAN);
    index_map.fill(-1);
    const int iter_upper = segmentedCloud->points.size();
    for (int i = 0; i < iter_upper; ++i) {
      index_map(segInfo.segmentedCloudRowInd[i],
                segInfo.segmentedCloudColInd[i]) = i;
    }
  }

  // 计算2D ROI区域深度差
  void calculateSmoothness() {
    neighbor_picked_mat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8SC1, cv::Scalar(0));
    cloud_abs_range_diff.resize(cloudSize, 0);

    const auto& rm = rangeMat;
    const int iter_lower = 0;
    const int iter_upper = cloudSize;
    for (int i = iter_lower; i < iter_upper ; ++i) {
      segmentedCloud->points[i].intensity = 0; // clear for semantic

      row = segInfo.segmentedCloudRowInd[i];
      col = segInfo.segmentedCloudColInd[i];
      if (row + row_used_num >= N_SCAN || row - row_used_num < 0 ||
          col + col_used_num >= Horizon_SCAN || col - col_used_num < 0) {
        neighbor_picked_mat.at<char>(row, col) = -15;
        cloud_abs_range_diff[i] = 99;
        segmentedCloud->points[i].curvature = cloud_abs_range_diff[i];
        continue;
      }
      neighbor_picked_mat.at<char>(row, col) = 1;
      const float center_r = rm.at<float>(row, col);

      // 行列平均深度差
      double mean_range_dif = 0;
      int valid_range_cnt = 0;
      for (int j = -row_used_num; j < row_used_num; ++j) {
        for (int k = -col_used_num; k < col_used_num; ++k) {
          if (j == 0 && k == 0 || rm.at<float>(row - j, col - k) > max_valid_range)
            continue;
          mean_range_dif += center_r - rm.at<float>(row - j, col - k);
          ++valid_range_cnt;
        }
      }
      mean_range_dif /= valid_range_cnt;
      cloud_abs_range_diff[i] = std::abs(mean_range_dif) / center_r;
      if (nf_p.recall_map_plane_thresh <= 0)
        segmentedCloud->points[i].curvature = cloud_abs_range_diff[i];
      // else curvature = 99
    }
  }

  void markOccludedPoints() {
    const auto &rm = rangeMat;

    int neighbor_col, neighbor_row;

    int boundary_num{0}, recall_num{0}, recall_num2{0}, recall_num3{0};
    int b_num{0}, b_neightbor_num{0}, a_num{0}, invalid_neighbor_num{0};

    const int iter_lower = col_drop_num;
    const int iter_upper = cloudSize - col_drop_num -1;
    // 1. 检测离群点
    for (int i = iter_lower; i < iter_upper ; ++i) {
      row = segInfo.segmentedCloudRowInd[i];
      col = segInfo.segmentedCloudColInd[i];

      // 边界点 膨胀点判断
      if (neighbor_picked_mat.at<char>(row, col) <= 0) {
        // 0 格子内没有点
        // -15 有点且越界
        // -10 B类膨胀点
        if (neighbor_picked_mat.at<char>(row, col) == -10 &&
            segmentedCloud->points[i].intensity >= 0) {
          segmentedCloud->points[i].intensity = -11;
          neighbor_picked_mat.at<char>(row, col) == -11;
          ++b_neightbor_num;
        }
        continue;
      }
      // 超出边界，标记当前点无效
      if (neighbor_picked_mat.at<uchar>(row, col)== -15) {
        segmentedCloud->points[i].intensity = -15;
        continue;
      }
      const float center_r = rm.at<float>(row, col);

      ROI_rm.setTo(0);
      d_ROI_rm.setTo(0);
      near_d_ROI_rm.setTo(max_valid_range);
      // ROI深度图, D_ROW*D_COL
      for (int m = -row_used_num; m <= row_used_num; ++m) {
        for (int n = -col_used_num; n <= col_used_num; ++n) {
          neighbor_row = row + m;
          neighbor_col = col + n;
          ROI_rm.at<float>(row_used_num + m, col_used_num + n) =
              std::min(rm.at<float>(neighbor_row, neighbor_col), max_valid_range);

          // ROI深度差图
          neighbor_row = row_used_num + m;
          neighbor_col = col_used_num + n;
          d_ROI_rm.at<float>(neighbor_row, neighbor_col) =
              ROI_rm.at<float>(neighbor_row, neighbor_col) - center_r;    
        }
      }

      // 8邻域差值绝对值 最小值记作中点深度差
      for (int m = -1; m <= 1; ++m) {
        for (int n = -1; n <= 1; ++n) {
          if (m == 0 && n == 0)
            continue;
          for (int step = 1; step <= row_used_num; ++step) {
            // neighbor = center + direction * step
            int row = row_used_num + m * step;
            int col = col_used_num + n * step;
            // near_d_ROI_rm ~[0, max_valid_range]
            near_d_ROI_rm.at<float>(1 + m, 1 + n) =
                std::min(near_d_ROI_rm.at<float>(1 + m, 1 + n),
                         std::abs(d_ROI_rm.at<float>(row, col)));
          }
        }
      }

      // 滤除B类噪点--单边孤立（前后景分层）
      B_depth_jump_thresh = center_r * nf_p.curva_thresh;
      A_depth_jump_thresh = B_depth_jump_thresh * 0.5;
      int A_edge_jump_cnt{0};
      std::vector<Eigen::Vector2i> B_edge_jump_dir; // 非空代表是B类
      for (int m = -1; m <= 1; ++m) {
        for (int n = -1; n <= 1; ++n) {
          if (m == 0 && n == 0)
            continue;
          const float &near_r = near_d_ROI_rm.at<float>(1 + m, 1 + n);
          if (near_r > B_depth_jump_thresh) {
            B_edge_jump_dir.push_back(Eigen::Vector2i(m, n));
          } else if (near_r > A_depth_jump_thresh) {
            ++A_edge_jump_cnt;
          }
        }
      }

      // B类拖点膨胀，拖点方向的外沿也认为拖点
      for (const auto &dir : B_edge_jump_dir) {
        for (int step = -col_drop_num; step <= col_drop_num; ++step) {
          neighbor_row = row + dir[0] * step;
          neighbor_col = col + dir[1] * step;

          if (neighbor_col < 0 || neighbor_col >= Horizon_SCAN ||
              neighbor_row < 0 || neighbor_row >= N_SCAN)
            continue;
          // B neighbor is selected
          neighbor_picked_mat.at<char>(neighbor_row, neighbor_col) = -10;
        }
      }
      if (B_edge_jump_dir.size()) {
        neighbor_picked_mat.at<char>(row, col) = -10;
        segmentedCloud->points[i].intensity = -10;
        continue; // B is selected
      }

      // 滤除A类噪点--多边孤立（凸起、凹陷点、或孤立点）
      if (A_edge_jump_cnt > 3) {
        neighbor_picked_mat.at<char>(row, col) = -9;
        segmentedCloud->points[i].intensity = -9;
      }
      // TODO: A类需要膨胀？需要调试
    }

    // 2.从所有离群点中召回平面点
    int pseudo_noise_num = 0;
    BasePC::Ptr cluster(new BasePC);
    std::vector<BaseP, Eigen::aligned_allocator<BaseP>> points_near;
    std::vector<BaseP, Eigen::aligned_allocator<BaseP>> valid_near;
    int max_neightbor_num = (2 * nf_p.radius + 1) * (2 * nf_p.radius + 1);
    if (nf_p.recall_map_plane_thresh > 0) {
      for (int i = iter_lower; i < iter_upper; ++i) {
        auto& pt = segmentedCloud->points[i];
        if (pt.intensity >= 0)
          continue;
        ++pseudo_noise_num;
        int row = segInfo.segmentedCloudRowInd[i];
        int col = segInfo.segmentedCloudColInd[i];
        // 离群点在已有地图附近
        //  TODO:mapping内计算curvature部分移到这里
        if (pt.curvature < 99) {
          // TODO: 且附近地图法向量平均夹角较小
          // if (pt.curvature > 5 * D2R)
          //   continue;
          if (cloud_abs_range_diff[i] < nf_p.recall_map_plane_thresh) {
            ++recall_num;
            pt.intensity = 2;
          }
        }
        // 离群点远离地图 则需要用scan邻域判断
        else {
          // fast neightbor search and normal calucation
          points_near.clear();
          points_near.reserve(max_neightbor_num);
          for (int m = -nf_p.radius; m <= nf_p.radius; ++m) {
            for (int n = -nf_p.radius; n <= nf_p.radius; ++n) {
              int roi_row = row + m;
              int roi_col = col + n;
              if (roi_row < 0 || roi_row >= N_SCAN || roi_col < 0 ||
                  roi_col >= Horizon_SCAN)
                continue;
              int index = index_map(roi_row, roi_col);
              if (index == -1)
                continue;
              points_near.push_back(segmentedCloud->points[index]);
            }
          }
          if (points_near.size() < max_neightbor_num * 0.5)
            continue;
          // // 拟合平面（计算法向量）
          // Eigen::Matrix<float, 4, 1> pabcd;
          // if (!esti_plane(pabcd, points_near, nf_p.fit_plane_dist_thresh)) {
          //   continue;
          // }
          // float pd2 =
          //     pabcd(0) * pt.x + pabcd(1) * pt.y + pabcd(2) * pt.z + pabcd(3);
          // float s = fabs(pd2) / segInfo.segmentedCloudRange[i];
          // if (s > nf_p.recall_scan_plane_thresh)
          //   continue;
          // TODO: 斜视时，2D框选的方形在3D是长方体，影响PCA特征值。希望要球形
          // PCA判断平面
          cluster->points = points_near;
          auto value_vector_pair = PCLPCA(cluster);
          auto &lbd = value_vector_pair.first; // 特征值 从大到小
          // float lbd_sum = lbd[0] + lbd[1] + lbd[2];
          float plane_ratio = lbd[2] / 1.; // TODO: 这里是绝对值，吃场景。但相对值实验效果不太好
          float mid_ratio = lbd[1] / 1.;
          float line_ratio = lbd[0] / 1.;

          if (lbd[0] < nf_p.pca_thresh[0]) {
            pt.intensity = 4;
            ++recall_num2;
          } else if (lbd[2] < nf_p.pca_thresh[2]) {
            pt.intensity = 6;
            ++recall_num3;
          }
#ifdef SAVE_PCD
          FA_res.semanticCloud.points.push_back(pt);
          FA_res.semanticCloud.points.back().curvature = valid_near.size();
          FA_res.semanticCloud.points.back().normal_x = plane_ratio;
          FA_res.semanticCloud.points.back().normal_y = line_ratio;
          FA_res.semanticCloud.points.back().normal_z = mid_ratio;
          FA_res.semanticCloud.points.back().intensity = pt.intensity;
#endif
        }
      }
    }
    // 3.点云分类
    for (int i = iter_lower; i < iter_upper; ++i) {
      if (segmentedCloud->points[i].intensity == 1) {
        FA_res.outlierCloud.points.push_back(segmentedCloud->points[i]);
      } else if (segmentedCloud->points[i].intensity == 2 ||
                 segmentedCloud->points[i].intensity == 4 ||
                 segmentedCloud->points[i].intensity == 6) {
        FA_res.validCloud.points.push_back(segmentedCloud->points[i]);
        FA_res.outlierCloud.points.push_back(segmentedCloud->points[i]);
      } else {
        if (segmentedCloud->points[i].intensity < 0) {
          FA_res.outlierCloud.points.push_back(segmentedCloud->points[i]);
          ++boundary_num;
        } else {
          FA_res.validCloud.points.push_back(segmentedCloud->points[i]);
        }
      }
    }
    // 数组边界认为拖点
    for (int i = 0; i < iter_lower; ++i) {
      segmentedCloud->points[i].intensity = -19;
      FA_res.outlierCloud.points.push_back(segmentedCloud->points[i]);
      ++boundary_num;
    }
    for (int i = iter_upper; i < cloudSize; ++i) {
      segmentedCloud->points[i].intensity = -19;
      FA_res.outlierCloud.points.push_back(segmentedCloud->points[i]);
      ++boundary_num;
    }

    for (int row = 0; row < N_SCAN; ++row) {
      for (int col = 0; col < Horizon_SCAN; ++col) {
        int picked_value = neighbor_picked_mat.at<char>(row, col);
        if (picked_value == -10 || picked_value == -11)
          ++b_num;
        else if (picked_value == -9)
          ++a_num;
        else if (picked_value == -20 || picked_value == -19)
          ++invalid_neighbor_num;
      }
    }

    LDEBUG << "[ Pick boundary] b_num(-10): " << b_num
          << " b_neightbor_num(-11): " << b_neightbor_num
          << " a_num(-9): " << a_num
          << " invalid_neighbor_num(-20): " << invalid_neighbor_num
          << " pseudo_noise: "
          << b_num + b_neightbor_num + a_num + invalid_neighbor_num << "/"
          << cloudSize << REND;

    if (nf_p.recall_map_plane_thresh > 0) {
      LDEBUG << "===Recall plane. pseudo_noise: " << pseudo_noise_num
            << " recall_num: " << recall_num << " recall_num2: " << recall_num2
            << " recall_num3: " << recall_num3 << REND;
    }
    LDEBUG << "boundary: " << boundary_num
          << " valid: " << FA_res.validCloud.points.size() << "/" << cloudSize
          << REND;
  }
};