#pragma once
#include <mutex>
#include <queue>
#include <thread>

#include "image_projection.h"
#include "feature_association.h"
#include "common/utility/yaml_reader.hpp"

struct LidarOdometryParameter {
  HardWareParameter HW_param;
  SegmentationParameter Seg_param;
  NoiseFilterParameter Filter_param;
};

class RsLidarOdometry {
 public:
  RsLidarOdometry() = default;
  RsLidarOdometry(const YAML::Node& config_node);
  ~RsLidarOdometry() {;}

  void AddLiDAR(const RsPC::Ptr lidar_cloud, const Pose& T_W_L_pose);
  RsPC GetVlidaCloud() { return Filter_res_.validCloud; };
  std::shared_ptr<SegmentationResult> GetSegResult() {
    return std::make_shared<SegmentationResult>(Seg_res_);
  };
  std::shared_ptr<FeatureClouds> GetFilterResult() {
    return std::make_shared<FeatureClouds>(Filter_res_);
  };

private:
  // 关键模块
  std::shared_ptr<ImageProjection> image_projection_;
  std::shared_ptr<FeatureAssociation> cloud_filter_;
  SegmentationResult Seg_res_;
  FeatureClouds Filter_res_;

  // 配置
  LidarOdometryParameter LO_param_;
  std::string lidar_type_ = "m1p";

  // 输入
  Pose T_W_L_pose_;

 private:
  void LoadParameters(const YAML::Node& config_node);
  void SavePcd();
};

inline void RsLidarOdometry::LoadParameters(const YAML::Node& config_node) {
  // module config
  auto &HW_p = LO_param_.HW_param;
  {
    auto module_node = config_node["hardware_setup"];
    yamlRead<double>(module_node, "MINIMUM_RANGE", HW_p.MINIMUM_RANGE, 1);
    yamlRead<double>(module_node, "MAX_RANGE", HW_p.MAX_RANGE, 0);
    yamlRead<int>(module_node, "N_SCAN", HW_p.N_SCAN, 126);
    yamlRead<int>(module_node, "Horizon_SCAN", HW_p.Horizon_SCAN, 600);
    yamlRead<float>(module_node, "ang_res_x", HW_p.ang_res_x, 0.2);
    yamlRead<float>(module_node, "ang_res_y", HW_p.ang_res_y, 0.2);
    yamlRead<double>(module_node, "ang_bottom", HW_p.ang_bottom, 12.5);
    yamlRead<int>(module_node, "groundScanInd", HW_p.groundScanInd, 80);
  }

  auto &IM_p = LO_param_.Seg_param;
  {
    auto &module_node = config_node["image_projection"];
    yamlRead<double>(module_node, "sensorMinimumRange", IM_p.sensorMinimumRange,
                     1);
    yamlRead<int>(module_node, "column_drop_num", IM_p.column_drop_num, 0);
    yamlRead<double>(module_node, "z_thresh", IM_p.z_thresh, -1.0);
    yamlRead<double>(module_node, "segmentTheta", IM_p.segmentTheta, 20.0);
    IM_p.segmentTheta = IM_p.segmentTheta * D2R;
    yamlRead<int>(module_node, "segmentValidPointNum",
                  IM_p.segmentValidPointNum, 5);
    yamlRead<int>(module_node, "segmentValidLineNum", IM_p.segmentValidLineNum,
                  10);
    yamlRead<double>(module_node, "segmentAlphaX", IM_p.segmentAlphaX,
                     HW_p.ang_res_x);
    IM_p.segmentAlphaX = IM_p.segmentAlphaX * D2R;
    yamlRead<double>(module_node, "segmentAlphaY", IM_p.segmentAlphaY,
                     HW_p.ang_res_y);
    IM_p.segmentAlphaY = IM_p.segmentAlphaY * D2R;
  }
  auto &F_p = LO_param_.Filter_param;
  {
    auto &module_node = config_node["noise_filter"];
    yamlRead<double>(module_node, "curva_thresh", F_p.curva_thresh,
                     F_p.curva_thresh);
    F_p.curva_thresh = F_p.curva_thresh * D2R;
    yamlRead<int>(module_node, "radius", F_p.radius, 2);
    yamlRead<double>(module_node, "recall_map_plane_thresh",
                     F_p.recall_map_plane_thresh, -1.);
    F_p.recall_map_plane_thresh = F_p.recall_map_plane_thresh * D2R;
    yamlRead<double>(module_node, "recall_scan_plane_thresh",
                     F_p.recall_scan_plane_thresh, -1.);
    F_p.recall_scan_plane_thresh = F_p.recall_scan_plane_thresh * D2R;       
    yamlRead<float>(module_node, "fit_plane_dist_thresh",
                     F_p.fit_plane_dist_thresh, 0.1);  
    std::vector<double> tmp(3,0);
    yamlRead<std::vector<double>>(module_node, "pca_thresh",
                     F_p.pca_thresh, tmp); 
  }
}

inline RsLidarOdometry::RsLidarOdometry(const YAML::Node& config_node) {
  // load algo params
  LoadParameters(config_node);

  image_projection_ =
      std::make_shared<ImageProjection>(LO_param_.HW_param, LO_param_.Seg_param);
  cloud_filter_ = std::make_shared<FeatureAssociation>(LO_param_.HW_param, LO_param_.Filter_param);
}

inline void RsLidarOdometry::AddLiDAR(const RsPC::Ptr lidar_cloud, const Pose& T_W_L_pose) {
  T_W_L_pose_ = T_W_L_pose;
  Seg_res_ = image_projection_->cloudHandler(lidar_cloud);
  Filter_res_ = cloud_filter_->CloudHandler(Seg_res_, T_W_L_pose_.timestamp);
#if SAVE_PCD
  SavePcd();
#endif
}

inline void TransformCloud(BasePC &cloud, const Eigen::Matrix4d &T_W_L) {
  if (cloud.size()) {
    pcl::transformPointCloud(cloud, cloud, T_W_L);
  }
}

inline void RsLidarOdometry::SavePcd() {
  Eigen::Matrix4d T_W_L = T_W_L_pose_.transform();
  // save segmented clouds
  // 分割结果
  if (Seg_res_.fullCloudIndex.size()) {
    TransformCloud(Seg_res_.fullCloudIndex, T_W_L);
    pcl::io::savePCDFileBinary("/apollo/data/log/vis-valid_raw.pcd",
                               Seg_res_.fullCloudIndex);
  }
  // if (Seg_res_.segmentedCloud.size()) {
  //   TransformCloud(Seg_res_.segmentedCloud, T_W_L);
  //   pcl::io::savePCDFileBinary("/apollo/data/log/vis-segmented_pure.pcd",
  //                              Seg_res_.segmentedCloud);
  // }
  if (Seg_res_.groundCloud.size()) {
    TransformCloud(Seg_res_.groundCloud, T_W_L);
    pcl::io::savePCDFileBinary("/apollo/data/log/vis-ground.pcd",
                               Seg_res_.groundCloud);
  }
  // 分类结果
  // 平面召回中间结果
  if (Filter_res_.semanticCloud.size()) {
    TransformCloud(Filter_res_.semanticCloud, T_W_L);
    pcl::io::savePCDFileBinary("/apollo/data/log/vis-semantic.pcd",
                               Filter_res_.semanticCloud);
  }
  // 全量点云=噪点+非噪点
  if (Filter_res_.segmentedCloud.size()) {
    TransformCloud(Filter_res_.segmentedCloud, T_W_L);
    pcl::io::savePCDFileBinary("/apollo/data/log/vis-segmented_pure.pcd",
                               Filter_res_.segmentedCloud);
  }
  // 噪点
  if (Filter_res_.outlierCloud.size()) {
    TransformCloud(Filter_res_.outlierCloud, T_W_L);
    pcl::io::savePCDFileBinary("/apollo/data/log/vis-outlier.pcd",
                               Filter_res_.outlierCloud);
  }
  // 非噪点
  if (Filter_res_.validCloud.size()) {
    BasePC validCloud_W;
    pcl::transformPointCloud(Filter_res_.validCloud, validCloud_W, T_W_L);
    pcl::io::savePCDFileBinary("/apollo/data/log/vis-valid_seg.pcd",
                               validCloud_W);
  }
}
