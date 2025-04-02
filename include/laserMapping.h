// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

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

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
// #include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <so3_math.h>
#include <Eigen/Core>
#include "common_lib.h"
#include "IMU_Processing.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>
#include "lidar_selection.h"
#include "vikit/pinhole_camera.h"
#include "LOAM/lidar_odometry.h"
#include <ikd-Tree/ikd_Tree.h>
#ifdef ADAPTIVE_INIT
#include "livo_initial/pre_integration.h"
#include "livo_initial/initializer.h"
#include "livo_initial/feature_tracker.h"
#endif
#include "feature.h"
#include "frame.h"
#include "point.h"

namespace robosense {
namespace slam {

enum SensorType { IMU = 0, LIDAR = 1, CAMERA = 2, OTHER = 3 };

struct SensorData {
  SensorType type;
  double timestamp;
  slam::Imu::ConstPtr imu;
  CloudPtr cloud;
  cv::Mat mat;
};

struct LIOLog {
  LIOLog(const std::string _log_dir) {
    log_dir = _log_dir;;
    log_file_.open(log_dir + "/lio_log.txt");
  }
  ~LIOLog() {
    log_file_.close();
  }
  std::string log_dir;
  double update_ts;

  // optimization
  bool flg_EKF_inited = false;
  double opt_time_cost = 0.0; /* optimization time cost, (s) */
  int opt_iter = 0;           /* optimization iteration */
  int valid_size = 0;
  double res_mean_last = 0;
  std::ofstream log_file_;

  // eskf
  VD(DIM_STATE) solution;

  // 
  void Print() {
    LDEBUG << "==============LIOLog==============" << REND;
    LDEBUG << " update_ts: " << fixed << update_ts << REND;
    LDEBUG << " flg_EKF_inited: " << flg_EKF_inited << REND;
    LDEBUG << " opt_time_cost: " << opt_time_cost << REND;
    LDEBUG << " opt_iter: " << opt_iter << REND;
    LDEBUG << " valid_size: " << valid_size << REND;
    LDEBUG << " res_mean_last: " << res_mean_last << REND;
    LDEBUG << "==============eskf==============" << REND;
    LDEBUG << " solution: " << solution.transpose() << REND;
  }
  void SaveLog() {
    // static ofstream log_file;
    if (!log_file_.is_open()) {
      log_file_.open(log_dir + "/lio_log.txt");
    }
    log_file_ << "==============LIOLog==============" << endl;
    log_file_ << " update_ts: " << fixed << update_ts << endl;
    log_file_ << " flg_EKF_inited: " << flg_EKF_inited << endl;
    log_file_ << " opt_time_cost: " << opt_time_cost << endl;
    log_file_ << " opt_iter: " << opt_iter << endl;
    log_file_ << " valid_size: " << valid_size << endl;
    log_file_ << " res_mean_last: " << res_mean_last << endl;
    log_file_ << "==============eskf==============" << endl;
    log_file_ << " solution: " << solution.transpose() << endl;
  }
};

struct LIOResult {
  LIOResult(const std::string _log_dir) {
    log_dir = _log_dir;
    log.reset(new LIOLog(log_dir));
    scan.reset(new PointCloudXYZI);
    map.reset(new PointCloudXYZI);
    kdtree_map.reset(new PointCloudXYZI);
    rgb_scan.reset(new PointCloudXYZRGB);
    rgb_map.reset(new PointCloudXYZRGB);
    log_file_.open(log_dir + "/lio_result.txt");
  }
  ~LIOResult() {
    log_file_.close();
  }
  double update_ts;
  Pose lidar_pose;     /* Lidar pose in W (World frame, first IMU frame)*/
  CloudPtr scan;       /* current lidar scan in W */
  CloudPtr map;        /* accumulated map (hitorical scans) in W */
  CloudPtr kdtree_map; /* LIO kdtree map in W */

  RGBCloudPtr rgb_scan; /* scan with rgb */
  RGBCloudPtr rgb_map;  /* map with rgb */

  StatesGroup state;

  // log
  std::shared_ptr<LIOLog> log; /* log */
  std::string log_dir;
  std::ofstream log_file_;
  void Print() {
    LDEBUG << "==============LIOResult==============" << REND;
    LDEBUG << " update_ts: " << fixed << update_ts << REND;
    // LDEBUG << " lidar_pose: " << lidar_pose << REND;
    LDEBUG << " scan size: " << scan->size() << REND;
    LDEBUG << " map size: " << map->size() << REND;
    LDEBUG << " kdtree_map size: " << kdtree_map->size() << REND;
    LDEBUG << " rgb_scan size: " << rgb_scan->size() << REND;
    LDEBUG << " rgb_map size: " << rgb_map->size() << REND;
    LDEBUG << "==============LIOState==============" << REND;
    LDEBUG << " p: " << state.pos_end.transpose() << REND;
    LDEBUG << " q: " << Quaterniond(state.rot_end).coeffs().transpose() << REND;
    LDEBUG << " v: " << state.vel_end.transpose() << REND;
    LDEBUG << " ba: " << state.bias_a.transpose() << REND;
    LDEBUG << " bg: " << state.bias_g.transpose() << REND;
    LDEBUG << " g: " << state.gravity.transpose() << REND;
    log->Print();
  }
  void SaveLog() {
    // static ofstream log_file;
    if (!log_file_.is_open()) {
      log_file_.open(log_dir + "/lio_result.txt");
    }
    log_file_ << "==============LIOResult==============" << endl;
    log_file_ << " update_ts: " << fixed << update_ts << endl;
    // log_file_ << " lidar_pose: " << lidar_pose << endl;
    log_file_ << " scan size: " << scan->size() << endl;
    log_file_ << " map size: " << map->size() << endl;
    log_file_ << " kdtree_map size: " << kdtree_map->size() << endl;
    log_file_ << " rgb_scan size: " << rgb_scan->size() << endl;
    log_file_ << " rgb_map size: " << rgb_map->size() << endl;
    log_file_ << "==============LIOState==============" << endl;
    log_file_ << " p: " << state.pos_end.transpose() << endl;
    log_file_ << " q: " << Quaterniond(state.rot_end).coeffs().transpose() << endl;
    log_file_ << " v: " << state.vel_end.transpose() << endl;
    log_file_ << " ba: " << state.bias_a.transpose() << endl;
    log_file_ << " bg: " << state.bias_g.transpose() << endl;
    log_file_ << " g: " << state.gravity.transpose() << endl;

    log->SaveLog();
  }
};

struct VIOLog {
  VIOLog(const std::string _log_dir) {
    log_dir = _log_dir;
    lidar_selector_result.reset(new lidar_selection::LidarSelectorResult(log_dir));
  }
  std::string log_dir;
  double update_ts;

  // prepare

  // optimization
  std::shared_ptr<lidar_selection::LidarSelectorResult> lidar_selector_result;

  // 
  void Print() {}
  void SaveLog() { lidar_selector_result->SaveLog(); }
};

struct VIOResult {
  VIOResult(const std::string _log_dir) {
    log_dir = _log_dir;
    log.reset(new VIOLog(log_dir));
  }
  double update_ts;
  double opt_time_cost = 0.0;
  Pose cam_pose;         /* Camera pose in W */
  cv::Mat img_cp;        /* feature image */
  cv::Mat img_noise;     /* noise image */
  cv::Mat img_raw;       /* raw image */
  cv::Mat img_all_cloud; /* all cloud proj to image */

  std::shared_ptr<VIOLog> log; /* log */
  std::string log_dir;

  // log
  void Print() {
    LDEBUG << "==============VIOResult==============" << REND;
    log->Print();
  }
  void SaveLog() { log->SaveLog(); }
};

struct CloudProcessResult {
  CloudProcessResult(const std::string _log_dir) {
    log_dir = _log_dir;
    log_file_.open(log_dir + "/cloud_process_log.txt");
  }
  ~CloudProcessResult() {
    log_file_.close();
  }
  double update_ts;
  // preprocess (rm flying cloud + downsample)
  int undistort_size = -1; /* 去畸变点云， 预处理算法输入 */
  // rm flying cloud
  int valid_size = -1;        /* 有效输入点 */
  int valid_seg_size = -1;    /* 有效分割点 */
  int boundary_semantic_size = -1;     /* 语义点（拖点+非拖点） */
  int boundary_size = -1;     /* 拖点 */
  int non_boundary_size = -1; /* 非拖点 */
  // downsample
  int feats_down_size = -1; /* 下采样 */

  // kdtree
  int build_size = -1;

  bool need_move;
  int kdtree_delete_counter = -1;
  double kdtree_delete_time = -1.;
  int current_size = -1;

  // log
  std::string log_dir;
  std::ofstream log_file_;
  void Print() {
    LDEBUG << "==============CloudProcessResult==============" << REND;
    LDEBUG << " update_ts: " << fixed << update_ts << REND;
    LDEBUG << " undistort_size: " << undistort_size << REND;
    LDEBUG << " valid_size: " << valid_size << REND;
    LDEBUG << " valid_seg_size: " << valid_seg_size << REND;
    LDEBUG << " boundary_semantic_size: " << boundary_semantic_size << REND;
    LDEBUG << " boundary_size: " << boundary_size << REND;
    LDEBUG << " non_boundary_size: " << non_boundary_size << REND;
    LDEBUG << " feats_down_size: " << feats_down_size << REND;

    LDEBUG << " build_size: " << build_size << REND;
    LDEBUG << " need_move: " << need_move << REND;
    LDEBUG << " kdtree_delete_counter: " << kdtree_delete_counter << REND;
    LDEBUG << " kdtree_delete_time: " << kdtree_delete_time << REND;
    LDEBUG << " current_size: " << current_size << REND;
  }
  void SaveLog(){
    // static ofstream log_file;
    if (!log_file_.is_open()) {
      log_file_.open(log_dir + "/cloud_process_log.txt");
    }
    log_file_ << "==============CloudProcessResult==============" << std::endl;
    log_file_ << " update_ts: " << fixed << update_ts << std::endl;
    log_file_ << " undistort_size: " << undistort_size << std::endl;
    log_file_ << " valid_size: " << valid_size << std::endl;
    log_file_ << " valid_seg_size: " << valid_seg_size << std::endl;
    log_file_ << " boundary_semantic_size: " << boundary_semantic_size << std::endl;
    log_file_ << " boundary_size: " << boundary_size << std::endl;
    log_file_ << " non_boundary_size: " << non_boundary_size << std::endl;
    log_file_ << " feats_down_size: " << feats_down_size << std::endl;
  
    log_file_ << " build_size: " << build_size << std::endl;
    log_file_ << " need_move: " << need_move << std::endl;
    log_file_ << " kdtree_delete_counter: " << kdtree_delete_counter << std::endl;
    log_file_ << " kdtree_delete_time: " << kdtree_delete_time << std::endl;
    log_file_ << " current_size: " << current_size << std::endl;
  }
};

struct TimeCost {
  TimeCost(const std::string _log_dir) {
    log_dir = _log_dir;
    log_file_.open(log_dir + "/time_cost_log.txt");
  }
  ~TimeCost() {
    log_file_.close();
  }
  std::string log_dir;
  std::ofstream log_file_;

  double imu_process_t = -1;
  double cloud_process_t = -1;
  double lio_t = -1;
  double vio_t = -1;
  double pub_vio_t = -1;
  double pub_lio_t = -1;
  void Print() {
    LDEBUG << "==============TimeCost(s)==============" << REND;
    LDEBUG << " imu_process_t: " << imu_process_t << REND;
    LDEBUG << " cloud_process_t: " << cloud_process_t << REND;
    LDEBUG << " lio_t: " << lio_t << REND;
    LDEBUG << " vio_t: " << vio_t << REND;
    LDEBUG << " pub_vio_t: " << pub_vio_t << REND;
    LDEBUG << " pub_lio_t: " << pub_lio_t << REND;
  }
  void SaveLog(double ts) {
    // static ofstream log_file;
    if (!log_file_.is_open()) {
      log_file_.open(log_dir + "/time_cost_log.txt");
    }
    // log_file_ << "==============TimeCost(s)==============" << std::endl;
    log_file_ << fixed << ts << " " << imu_process_t << " " << cloud_process_t
              << " " << lio_t << " " << vio_t << " " << pub_vio_t << " "
              << pub_lio_t << std::endl;
  }
};

struct LIVOResult {
  LIVOResult(const std::string _log_dir) {
    log_dir = _log_dir;
    source_str_map = {{int(SensorType::IMU), "IMU"},
                      {int(SensorType::LIDAR), "LIDAR"},
                      {int(SensorType::CAMERA), "CAMERA"},
                      {int(SensorType::OTHER), "OTHER"}};
    imu_process_result.reset(new ImuProcessResult(log_dir));
    cloud_process_result.reset(new CloudProcessResult(log_dir));
    lio_result.reset(new LIOResult(log_dir));
    vio_result.reset(new VIOResult(log_dir));
    time_cost.reset(new TimeCost(log_dir));
  }

  double update_ts = 0.;                 /* reuslt update time, (s) */
  SensorType source = SensorType::OTHER; /* 0: IMU, 1: Lidar, 2: Camera */
  std::map<int, string> source_str_map;

  // key module result
  shared_ptr<ImuProcessResult> imu_process_result;
  shared_ptr<CloudProcessResult> cloud_process_result;
  shared_ptr<LIOResult> lio_result;
  shared_ptr<VIOResult> vio_result;

  // module time-cost
  shared_ptr<TimeCost> time_cost;

  // debug
  std::string log_dir;
  void Print(bool print_imu_process_result, bool print_cloud_process_result,
             bool print_lio_result, bool print_vio_result,
             bool print_time_cost) {
    LDEBUG << "==============LIVOResult=============="
           << " source: " << source_str_map[source] << ", t(s): " << update_ts << REND;
    if (print_imu_process_result) {
      imu_process_result->Print();
    }
    if (print_cloud_process_result) {
      cloud_process_result->Print();
    }
    if (print_lio_result) {
      lio_result->Print();
    }
    if (print_vio_result) {
      // vio_result->Print();
    }
    if (print_time_cost) {
      time_cost->Print();
    }
    LDEBUG << REND;
  }
  void SaveLog(bool save_imu_process_result, bool save_cloud_process_result,
               bool save_lio_result, bool save_vio_result,
               bool save_time_cost) {
    // LDEBUG << "==============Save LIVOResult=============="
    //        << "source: " << source_str_map[source] << ", t(s): " << update_ts << REND;
    if (save_imu_process_result) {
      // imu_process_result->SaveLog();
    }
    if (save_cloud_process_result) {
      cloud_process_result->SaveLog();
    }
    if (save_lio_result) {
      lio_result->SaveLog();
    }
    if (save_vio_result) {
      vio_result->SaveLog();
    }
    if (save_time_cost) {
      time_cost->SaveLog(update_ts);
    }
  }
};

class FastLivoSlam {
 public:
  using Ptr = std::shared_ptr<FastLivoSlam>;

  static void resetStatics();
  FastLivoSlam()
  {
    resetStatics();
  }

  ~FastLivoSlam() {
    Stop();
    if (IsValid_log_file_.is_open()) {
      IsValid_log_file_.close();
    }
  }

  void Init(const std::string cfg_path);

  void AddData(const CloudPtr &msg_ptr, double timestamp) {
    standard_pcl_cbk(msg_ptr, timestamp);
  }

  void AddData(const slam::Imu::ConstPtr &msg_ptr) {
    imu_cbk(msg_ptr);
  }

  void AddData(const std::shared_ptr<cv::Mat> &msg_ptr, double timestamp) {
    img_cbk(msg_ptr, timestamp);
  }

  void RegisterGetLIVOResult(
      std::function<void(const LIVOResult &)> full_result_cb) {
    full_result_cb_ = full_result_cb;
  }

  void Start() {
    run_flag_ = true;
    if (msg_loop_thread_ptr_ == nullptr) {
      msg_loop_thread_ptr_ = std::make_unique<std::thread>(
          &FastLivoSlam::ProcessMsgBufferLoop, this);
    }
    if (core_loop_thread_ptr_ == nullptr) {
      core_loop_thread_ptr_ =
          std::make_unique<std::thread>(&FastLivoSlam::CoreLoop, this);
    }
  }

  void Stop() {
    LINFO << name() << ": stopping!" <<REND;
    run_flag_ = false;
    sig_buffer.notify_all();
    if (msg_loop_thread_ptr_ != nullptr) {
      if (msg_loop_thread_ptr_->joinable()) {
        msg_loop_thread_ptr_->join();
      }
      msg_loop_thread_ptr_.reset(nullptr);
    }
    if (core_loop_thread_ptr_ != nullptr) {
      if (core_loop_thread_ptr_->joinable()) {
        core_loop_thread_ptr_->join();
      }
      core_loop_thread_ptr_.reset(nullptr);
    }
    LINFO << name() << ": stopped!" <<REND;
  }

  // image
  void SetRGBImageCallback(const std::function<void(const cv::Mat &image)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    rgb_image_cb_ = callback;
  }

  void SetNoiseImageCallback(const std::function<void(const cv::Mat &image)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    noise_image_cb_ = callback;
  }

  void SetRawImageCallback(const std::function<void(const cv::Mat &image)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    raw_image_cb_ = callback;
  }

  void SetAllCloudImageCallback(const std::function<void(const cv::Mat &image)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    all_cloud_image_cb_ = callback;
  }

  // cloud
  void SetLaserCloudFullResRGBCallback(const std::function<void(const PointCloudXYZRGB::Ptr &msg_ptr)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    laser_cloud_full_res_rgb_cb_ = callback;
  }

  void SetLaserCloudFullResCallback(const std::function<void(const CloudPtr &msg_ptr)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    laser_cloud_full_res_cb_ = callback;
  }

  void SetSubVisualCloudCallback(const std::function<void(const CloudPtr &msg_ptr)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    sub_visual_cloud_cb_ = callback;
  }
  
  void SetLaserCloudEffectCallback(const std::function<void(const CloudPtr &msg_ptr)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    laser_cloud_effect_cb_ = callback;
  }

  void SetLaserCloudMapCallback(const std::function<void(const CloudPtr &msg_ptr)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    laser_cloud_map_cb_ = callback;
  }

  // path
  void SetOdomAftMappedCallback(const std::function<void(const Pose &msg_ptr)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    odom_aft_mapped_cb_ = callback;
  }

  void SetPathCallback(const std::function<void(const Pose &msg_ptr)> &callback) {
    std::lock_guard<std::mutex> lg(mx_cb_);
    path_cb_ = callback;
  }

 private:
  const std::string name() const {
    return "FastLivoSlam";
  }
  // in
  bool IsValid(int type, double ts, double new_iter_ts);
  void standard_pcl_cbk(const CloudPtr &msg, double timestamp);
  void imu_cbk(const slam::Imu::ConstPtr &msg_in);
  cv::Mat getImageFromMsg(const cv::Mat img_msg);
  void img_cbk(const std::shared_ptr<cv::Mat> &msg, double timestamp);
  // config
  void ReadParamters(const YAML::Node &algo_cfg_node);
  // out
  std::function<void(const LIVOResult&)> get_LIVO_res_;
  // chore funcs
  void ProcessMsgBufferLoop();
  void CoreLoop();
  bool CombineSensorMsgs(LidarMeasureGroup &meas);
  void PreprocessCloud(CloudPtr feats_undistort);

  // LIO
  void BuildKdTree(CloudPtr feats_down_body);
  void EstimateLIOState();
  BoxPointType LocalMap_Points;
  bool Localmap_Initialized = false;
  void CropKdTree(const StatesGroup &state);
  void RemovePointsFromKdTree() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // auto points_cache_size = points_history.size();
  }
  void KdTreeGrow();

  // VIO
  bool InitCam(const YAML::Node &cam_intrinsic_node, vk::AbstractCamera *&cam,
               double img_scaling_ratio = 1.0) {
    bool res = true;
    std::string cam_model = "Pinhole";
    yamlRead(cam_intrinsic_node, "cam_model", cam_model);
    if (cam_model == "Pinhole") {
      int cam_width, cam_height;
      double cam_d0, cam_d1, cam_d2, cam_d3, cam_d4, cam_d5, cam_d6, cam_d7;
      cam_d0 = cam_d1 = cam_d2 = cam_d3 = cam_d4 = cam_d5 = cam_d6 = cam_d7 = 0;
      yamlRead(cam_intrinsic_node, "cam_width", cam_width);
      yamlRead(cam_intrinsic_node, "cam_height", cam_height);
      yamlRead(cam_intrinsic_node, "cam_fx", cam_fx);
      yamlRead(cam_intrinsic_node, "cam_fy", cam_fy);
      yamlRead(cam_intrinsic_node, "cam_cx", cam_cx);
      yamlRead(cam_intrinsic_node, "cam_cy", cam_cy);
      yamlRead(cam_intrinsic_node, "cam_d0", cam_d0);
      yamlRead(cam_intrinsic_node, "cam_d1", cam_d1);
      yamlRead(cam_intrinsic_node, "cam_d2", cam_d2);
      yamlRead(cam_intrinsic_node, "cam_d3", cam_d3);
      yamlRead(cam_intrinsic_node, "cam_d4", cam_d4);
      yamlRead(cam_intrinsic_node, "cam_d5", cam_d5);
      yamlRead(cam_intrinsic_node, "cam_d6", cam_d6);
      yamlRead(cam_intrinsic_node, "cam_d7", cam_d7);
      LINFO << "InitCam: " << img_scaling_ratio << ", " << cam_width << ", " << cam_height << ", " << cam_fx << ", " << cam_fy << ", " << cam_cx << ", " << cam_cy << ", " << cam_d0 << ", " << cam_d1 << ", " << cam_d2 << ", " << cam_d3 << REND;
      cam = new vk::PinholeCamera(cam_width*img_scaling_ratio, cam_height*img_scaling_ratio, cam_fx*img_scaling_ratio, cam_fy*img_scaling_ratio, cam_cx*img_scaling_ratio, cam_cy*img_scaling_ratio, cam_d0, cam_d1, cam_d2, cam_d3, cam_d4, cam_d5, cam_d6, cam_d7);
#ifdef ADAPTIVE_INIT
      std::vector<double> dist_coeffs{cam_d0, cam_d1, cam_d2, cam_d3, cam_d4, cam_d5, cam_d6, cam_d7};
      livo_init_ptr->setCameraParam(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, dist_coeffs);
#endif
    } else {
      cam = NULL;
      res = false;
#ifdef ADAPTIVE_INIT
      livo_init_ptr.reset();
#endif
    }
    return res;
  }
  void EstimateVIOState();

  // publish result
  void PublishVIOResult();
  void PublishLIOResult();
  void SaveMap();

  // transform
  void pointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot_end *
                     (Lidar_rot_to_IMU * p_body + Lidar_offset_to_IMU) +
                 state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
  }

  void pointBodyToWorld(PointType const *const pi, PointType *const po,
                        StatesGroup &state) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot_end *
                     (Lidar_rot_to_IMU * p_body + Lidar_offset_to_IMU) +
                 state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
  }

  template <typename T>
  void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state.rot_end *
                     (Lidar_rot_to_IMU * p_body + Lidar_offset_to_IMU) +
                 state.pos_end);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
  }

  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot_end *
                     (Lidar_rot_to_IMU * p_body + Lidar_offset_to_IMU) +
                 state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - floor(intensity);

    int reflection_map = intensity * 10000;
  }

  // publish
  void publish_frame_world_rgb(lidar_selection::LidarSelectorPtr lidar_selector)
  {
      uint size = xyzi_scan->points.size();
      cv::Mat img_rgb = lidar_selector->img_rgb;
      if(img_en && img_rgb.cols && img_rgb.rows)
      {
          rgb_scan->clear();
          rgb_scan->reserve(size);
          for (int i=0; i<size; i++)
          {
              PointTypeRGB pointRGB;
              pointRGB.x =  xyzi_scan->points[i].x;
              pointRGB.y =  xyzi_scan->points[i].y;
              pointRGB.z =  xyzi_scan->points[i].z;
              V3D p_w(pointRGB.x, pointRGB.y, pointRGB.z);
              V3D pf(lidar_selector->new_frame_->w2f(p_w));
              if (pf[2] < 0) continue;
              V2D pc(lidar_selector->new_frame_->w2c(p_w));
              pc.x() = floor(pc.x());
              pc.y() = floor(pc.y());
              if (lidar_selector->new_frame_->cam_->isInFrame(pc.cast<int>(),0))
              {
                  V3F pixel = lidar_selector->getpixel(img_rgb, pc);
                  pointRGB.r = pixel[2];
                  pointRGB.g = pixel[1];
                  pointRGB.b = pixel[0];
                  rgb_scan->push_back(pointRGB);
              }
          }
      }

      if (1) {
        if (img_en && img_rgb.cols && img_rgb.rows) {
          if (laser_cloud_full_res_rgb_cb_) {
            laser_cloud_full_res_rgb_cb_(rgb_scan);
          }
        } else {
          if (laser_cloud_full_res_cb_) {
            laser_cloud_full_res_cb_(xyzi_scan);
          }
        }
      }
      /**************** save map ****************/
      /* 1. make sure you have enough memories
      /* 2. noted that pcd save will influence the real-time performences **/
      if (pcd_save_en)
      {
        if (voxel_size) {
          *rgb_map += *uniformSample<PointTypeRGB>(rgb_scan, voxel_size);
        } else {
          *rgb_map += *rgb_scan;
        }
      }
      // static int cnt{0};
      if(save_frame_mode == "single_ply")
          pcl::io::savePLYFile (root_dir+"PLY/tmp/"+std::to_string(publish_frame_world_rgb_cnt_)+".ply", *rgb_scan);
      else if(save_frame_mode == "whole_ply")
          pcl::io::savePLYFile (root_dir+"PLY/tmp/whole_"+std::to_string(publish_frame_world_rgb_cnt_)+".ply", *rgb_map);
      else if(save_frame_mode == "whole_pcd")
          pcl::io::savePCDFile (root_dir+"PCD/tmp/whole_"+std::to_string(publish_frame_world_rgb_cnt_)+".pcd", *rgb_map);
      else if(save_frame_mode == "single_pcd")
          pcl::io::savePCDFile (root_dir+"PCD/tmp/"+std::to_string(publish_frame_world_rgb_cnt_)+".pcd", *rgb_scan);
      
      publish_frame_world_rgb_cnt_++;
  }

  void publish_frame_world() {
    if (laser_cloud_full_res_cb_) {
      laser_cloud_full_res_cb_(xyzi_scan);
    }
    if (pcd_save_en)
      *xyzi_map += *xyzi_scan;
  }

  void publish_visual_world_map() {
    CloudPtr laserCloudFullRes(map_cur_frame_point);
    int size = laserCloudFullRes->points.size();
    if (size == 0)
      return;

    CloudPtr pcl_visual_wait_pub(new PointCloudXYZI());
    *pcl_visual_wait_pub = *laserCloudFullRes;
    // cb(pcl_visual_wait_pub);
  }

  void publish_visual_world_sub_map(
      const lidar_selection::LidarSelectorPtr lidar_selector) {
    // int size = lidar_selector->map_cur_frame_.size();
    int size_sub = lidar_selector->sub_map_cur_frame_.size();

    // map_cur_frame_point->clear();
    sub_map_cur_frame_point->clear();
    // for(int i=0; i<size; i++)
    // {
    //     PointType temp_map;
    //     temp_map.x = lidar_selector->map_cur_frame_[i]->pos_[0];
    //     temp_map.y = lidar_selector->map_cur_frame_[i]->pos_[1];
    //     temp_map.z = lidar_selector->map_cur_frame_[i]->pos_[2];
    //     temp_map.intensity = 0.;
    //     map_cur_frame_point->push_back(temp_map);
    // }
    for (int i = 0; i < size_sub; i++) {
      PointType temp_map;
      temp_map.x = lidar_selector->sub_map_cur_frame_[i]->pos_[0];
      temp_map.y = lidar_selector->sub_map_cur_frame_[i]->pos_[1];
      temp_map.z = lidar_selector->sub_map_cur_frame_[i]->pos_[2];
      temp_map.intensity = 0.;
      sub_map_cur_frame_point->push_back(temp_map);
    }

    CloudPtr laserCloudFullRes(sub_map_cur_frame_point);
    if (laserCloudFullRes->points.size() == 0)
      return;
    if(sub_visual_cloud_cb_)
    {
      sub_visual_cloud_cb_(laserCloudFullRes);
    }
  }

  void publish_effect_world() {
    CloudPtr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++) {
      RGBpointBodyToWorld(&laserCloudOri->points[i],
                          &laserCloudWorld->points[i]);
    }
    if (laser_cloud_effect_cb_) {
      laser_cloud_effect_cb_(laserCloudWorld);
    }
  }

  void publish_map() {
    if (laser_cloud_map_cb_) {
      laser_cloud_map_cb_(featsFromMap);
    }
  }

 private:
  bool run_flag_ = false;
  // core loop线程
  std::unique_ptr<std::thread> core_loop_thread_ptr_ = nullptr;
  mutex mtx_loop;
  condition_variable sig_buffer;

  // sensor msg buffer线程
  std::unique_ptr<std::thread> msg_loop_thread_ptr_ = nullptr;
  std::map<double, SensorData> ts_msg_buffer;
  mutex mtx_raw_msg_buffer;
  condition_variable cv_raw_msg;

  // 算法消息
  mutex mtx_buffer;

  // configs
  string root_dir, pcd_dir, log_dir;
  M3D Eye3d = M3D::Identity();
  V3D Zero3d{0, 0, 0};
  Vector3d Lidar_offset_to_IMU = Zero3d;
  M3D Lidar_rot_to_IMU = Eye3d;
  double gyr_cov_scale = 0, acc_cov_scale = 0;
  double filter_size_surf_min = 0, filter_size_map_min = 0;

  // rs configs
  double delay_t = 0.5;
  bool lidar_time_is_tail;
  int cam_keep_period{0};
  double light_min_thresh{0};
  bool use_map_proj;
  bool show_lidar_map;
  double VIO_wait_time;
  bool enable_normal_filter;
  double norm_thresh, weight;
  bool rs_debug, save_log, print_log;
  bool downsample_source_cloud;
  double z_thresh,y_thresh,x_thresh;
  double p2plane_thresh;
  double vis_degenerate_thresh;
  double lidar_degenerate_thresh;
  bool detect_lidar_degenetate;
  bool reset_state_when_degenerate;
  bool boundary_point_remove;
  bool trans_imu_to_lidar;
  Eigen::Matrix4d T_I_L, T_I_L_file;
  Eigen::Matrix3d R_L_I;
  Eigen::Matrix4d T_C_L;
  Pose T_I_L_pose, T_C_L_pose;
  bool show_rgb_map;
  double img_scaling_ratio;
  double blind_max, blind;

  // sensor msg buffer
  deque<CloudPtr> lidar_buffer;
  deque<double> time_buffer;
  deque<slam::Imu::ConstPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  deque<bool> img_valid_buffer;
  bool lidar_pushed;
  double last_timestamp_lidar = 0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double lidar_end_time = 0, first_lidar_time = 0.0;

  //estimator inputs and output;
  LidarMeasureGroup LidarMeasures;
  StatesGroup state;
  std::shared_ptr<LIVOResult> livo_result_;

  // ImuProcess
  shared_ptr<ImuProcess> p_imu;
  bool flg_imu_reset_ = false;
  StatesGroup state_propagat;

  // RsLidarOdometry
  std::shared_ptr<RsLidarOdometry> lidar_odometry_;
  Pose T_W_L_pose;

  // LIO
  float MOV_THRESHOLD = 1.5f;
  double DET_RANGE = 30.0f;
  int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0,\
      effct_feat_num = 0, time_log_counter = 0, publish_count = 0;
  double res_mean_last = 0.05;
  double cube_len = 0, total_distance = 0;
  vector<int> point_selected_surf; 
  vector<vector<int>> pointSearchInd_surf; 
  vector<PointVector> Nearest_Points; 
  vector<double> res_last;
  vector<double> extrinT;
  vector<double> extrinR;
  double total_residual;
  double LASER_POINT_COV; 
  double time_past;
  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;
  CloudPtr featsFromMap;
  CloudPtr feats_undistort;
  CloudPtr feats_down_body;
  CloudPtr feats_down_world;
  CloudPtr normvec;
  CloudPtr laserCloudOri;
  CloudPtr corr_normvect;
  pcl::PointIndices::Ptr valid_source_indices;
  pcl::PointIndices::Ptr invalid_source_indices;
  std::vector<int> invalid_reason;
  CloudPtr valid_source_cloud;
  CloudPtr invalid_source_cloud;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterMap;
  vector<BoxPointType> cub_needrm;
  vector<BoxPointType> cub_needad;
  KD_TREE ikdtree;
  VD(DIM_STATE) solution;
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
  V3D rot_add, t_add;
  V3D position_last;

  // VIO
  bool ncc_en;
  bool pub_dense_map = 1;
  int img_en = 1;
  int lidar_en = 1;
  int debug = 0;
  bool fast_lio_is_ready = false;
  int grid_size, patch_size;
  double outlier_threshold, ncc_thre;
  double delta_time = 0.0;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV, cam_fx, cam_fy, cam_cx, cam_cy;
  Eigen::Matrix3d Rcl;
  Eigen::Vector3d Pcl;
  lidar_selection::LidarSelectorPtr lidar_selector;

  // debug
  ofstream f_sensor_buf;
  ofstream fout_pre, fout_out, f_state_utm, f_lidar_state_utm;
  ofstream f_LIO_t;
  int frame_num = 0;
  double deltaT, deltaR, aver_total_t = 0, aver_update_lio_state_t = 0,
                         aver_time_match = 0, aver_time_solve = 0,
                         aver_time_const_H_time = 0;
  V3D euler_cur;

  double kdtree_grow_t = 0, kdtree_search_time = 0, kdtree_delete_time = 0.0;
  int kdtree_search_counter = 0, kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;;

  double update_LIO_state_t = 0, match_time = 0, solve_time = 0, solve_const_H_time = 0;
  double loop_beg_t, loop_end_t, match_start, solve_start, svd_time;
  double process2_t{0}, preprocess_t{0};
  double flatten_t{0}, filter_scan_t{0}, filter_map_t{0},
        detect_t{0}, VIO_total_t{0};
  double pub_vio_t{0}, pub_lio_t{0};

  // save map
  CloudPtr xyzi_scan;
  RGBCloudPtr rgb_scan;
  PointCloudXYZRGB::Ptr rgb_map;  //add save rbg map
  CloudPtr xyzi_map;  //add save xyzi map

  bool pcd_save_en = true;
  bool ply_save_en = true;
  std::string save_frame_mode;
  double voxel_size;
  bool pose_output_en = true;
#ifdef ADAPTIVE_INIT
  std::shared_ptr<livo_initial::Initializer> livo_init_ptr;
#endif

  // publish
  std::mutex mx_cb_;
  CloudPtr map_cur_frame_point;
  CloudPtr sub_map_cur_frame_point;
  std::function<void(const LIVOResult &full_result)> full_result_cb_;
  std::function<void(const cv::Mat &image)> rgb_image_cb_;
  std::function<void(const cv::Mat &image)> noise_image_cb_;
  std::function<void(const cv::Mat &image)> raw_image_cb_;
  std::function<void(const cv::Mat &image)> all_cloud_image_cb_;
  std::function<void(const PointCloudXYZRGB::Ptr &msg_ptr)> laser_cloud_full_res_rgb_cb_;
  std::function<void(const CloudPtr &msg_ptr)> laser_cloud_full_res_cb_;
  std::function<void(const CloudPtr &msg_ptr)> sub_visual_cloud_cb_;
  std::function<void(const CloudPtr &msg_ptr)> laser_cloud_effect_cb_;
  std::function<void(const CloudPtr &msg_ptr)> laser_cloud_map_cb_;
  std::function<void(const Pose &msg_ptr)> odom_aft_mapped_cb_;
  std::function<void(const Pose &msg_ptr)> path_cb_;

  // 替代static变量的成员变量
  bool core_loop_first_{true};     // 用于替代CoreLoop中的static变量
  double core_loop_first_lidar_time_{0.};      

  double process_msg_buffer_loop_last_msg_header_time_{0.};  
  int process_msg_buffer_loop_imu_cnt_{1}; // 用于替代ProcessMsgBuffer中的static变量
  int process_msg_buffer_loop_cam_cnt_{0};

  double CombineSensorMsgs_last_cam_t_{-1};
  double CombineSensorMsgs_last_lidar_t_{-1};

  std::ofstream IsValid_log_file_;

  int publish_frame_world_rgb_cnt_{0};
};

inline void FastLivoSlam::Init(const std::string cfg_path) {

  root_dir = cfg_path + "/../";
  pcd_dir = root_dir + "/PCD/";
  log_dir = root_dir + "/Log/";
  livo_result_.reset(new LIVOResult(log_dir));

  YAML::Node all_calib_node, livo_calib_node;
  try {
    all_calib_node = YAML::LoadFile(cfg_path + "/calibration.yaml");
    livo_calib_node = all_calib_node["ADDTION_INFO"];
  } catch (const std::exception &e) {
    LERROR << "load calibration.yaml error: " << e.what() << REND;
    exit(-1);
  }
  YAML::Node algo_cfg_node;
  try {
    algo_cfg_node = YAML::LoadFile(cfg_path + "/RS_META.yaml");
  } catch (const std::exception &e) {
    LERROR << "load RS_META.yaml error: " << e.what() << REND;
    exit(-1);
  }

  algo_cfg_node["mapping"]["extrinsic_T"] =  livo_calib_node["extrinsic_T"];
  algo_cfg_node["mapping"]["extrinsic_R"] =  livo_calib_node["extrinsic_R"];
  algo_cfg_node["camera"]["Rcl"] =  livo_calib_node["Rcl"];
  algo_cfg_node["camera"]["Pcl"] =  livo_calib_node["Pcl"];
  IsValid_log_file_.open(log_dir + "/drop_msg.txt");

  // LIO
  Lidar_offset_to_IMU = Zero3d;
  Lidar_rot_to_IMU = Eye3d;
  f_sensor_buf.open(log_dir+"sensor_buffer.txt", ios::out);
  extrinT = vector<double>(3, 0.0);
  extrinR = vector<double>(9, 0.0);
  featsFromMap.reset(new PointCloudXYZI());
  map_cur_frame_point.reset(new PointCloudXYZI());
  sub_map_cur_frame_point.reset(new PointCloudXYZI());

  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  normvec.reset(new PointCloudXYZI());
  laserCloudOri.reset(new PointCloudXYZI());
  corr_normvect.reset(new PointCloudXYZI());
  valid_source_indices.reset(new pcl::PointIndices);
  invalid_source_indices.reset(new pcl::PointIndices);
  valid_source_cloud.reset(new PointCloudXYZI);
  invalid_source_cloud.reset(new PointCloudXYZI);

  position_last = Zero3d;

  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  ReadParamters(algo_cfg_node);
#ifdef ADAPTIVE_INIT
  livo_init_ptr = std::make_shared<livo_initial::Initializer>(T_I_L, T_C_L, G_m_s2);
#endif

  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min,
                                filter_size_map_min);

  // ImuProcess
  p_imu.reset(new ImuProcess(log_dir));
  p_imu->result_ = livo_result_->imu_process_result;
  p_imu->img_en = img_en;

  Lidar_offset_to_IMU = T_I_L.block<3, 1>(0, 3); // VEC_FROM_ARRAY(extrinT);
  Lidar_rot_to_IMU = T_I_L.block<3, 3>(0, 0);    // MAT_FROM_ARRAY(extrinR);
  p_imu->set_extrinsic(Lidar_offset_to_IMU, Lidar_rot_to_IMU);
  p_imu->set_gyr_bias_cov(V3D(0.00001, 0.00001, 0.00001));
  p_imu->set_acc_bias_cov(V3D(0.00001, 0.00001, 0.00001));

  yamlRead<bool>(algo_cfg_node, "use_cfg_cov", p_imu->use_cfg_cov, false);
  yamlRead<double>(algo_cfg_node, "cov_acc", p_imu->cfg_cov_acc, 1.);
  yamlRead<double>(algo_cfg_node, "cov_gyr", p_imu->cfg_cov_gyr, 1.);
  yamlRead<double>(algo_cfg_node, "cov_bias_acc", p_imu->cfg_cov_bias_acc, 1.);
  yamlRead<double>(algo_cfg_node, "cov_bias_gyr", p_imu->cfg_cov_bias_gyr, 1.);
  yamlRead<double>(algo_cfg_node, "init_cov_rot", p_imu->init_cov_rot, 1.);
  yamlRead<double>(algo_cfg_node, "init_cov_pos", p_imu->init_cov_pos, 1.);
  yamlRead<double>(algo_cfg_node, "init_cov_vel", p_imu->init_cov_vel, 1.);
  yamlRead<double>(algo_cfg_node, "init_cov_bias_gyr", p_imu->init_cov_bias_gyr, 1.);
  yamlRead<double>(algo_cfg_node, "init_cov_bias_acc", p_imu->init_cov_bias_acc, 1.);

  // p_imu->set_gyr_bias_cov(V3D(1e-12, 1e-12, 1e-12));
  // p_imu->set_acc_bias_cov(V3D(1e-12, 1e-12, 1e-12));
#ifdef ADAPTIVE_INIT
  double cov_acc, cov_gyr;
  yamlRead<double>(algo_cfg_node["imu"], "cov_acc", cov_acc, 2.5e-1);
  yamlRead<double>(algo_cfg_node["imu"], "cov_gyr", cov_gyr, 9e-2);
  p_imu->set_acc_cov(V3D(cov_acc, cov_acc, cov_acc));
  p_imu->set_gyr_cov(V3D(cov_gyr, cov_gyr, cov_gyr));
#endif

  // VIO
  cameraextrinT = vector<double>(3, 0.0);
  cameraextrinR = vector<double>(9, 0.0);
  grid_size *= img_scaling_ratio;
  lidar_selector.reset(new lidar_selection::LidarSelector(
      grid_size, log_dir,
      livo_result_->vio_result->log->lidar_selector_result));

  if (!InitCam(livo_calib_node, lidar_selector->cam, img_scaling_ratio))
    // if(!vk::camera_loader::loadFromRosNs("laserMapping", lidar_selector->cam, img_scaling_ratio))
        throw std::runtime_error("Camera model not correctly specified.");

  // RS
  auto rs_node = algo_cfg_node["rs"];
  yamlRead<double>(rs_node, "sub_sparse_map_error_thresh", lidar_selector->sub_sparse_map_error_thresh, 8000);
  yamlRead<double>(rs_node, "depth_continuous_thresh", lidar_selector->depth_continuous_thresh, 1.5);
  yamlRead<double>(rs_node, "global_map_voxel_size", lidar_selector->global_map_voxel_size, 0.5);
  yamlRead<int>(rs_node, "max_obs_num", lidar_selector->max_obs_num, 20);
  yamlRead<int>(rs_node, "max_layer", lidar_selector->max_layer, 2);
  lidar_selector->layer_step_y = std::pow(2, lidar_selector->max_layer);
  lidar_selector->layer_step_x = lidar_selector->layer_step_y * lidar_selector->layer_step_y;
  std::cout << "lidar_selector->max_obs_num: " << lidar_selector->max_obs_num
            << ", lidar_selector->max_layer: " << lidar_selector->max_layer
            << ", lidar_selector->layer_step_y: " << lidar_selector->layer_step_y
            << ", lidar_selector->layer_step_x: " << lidar_selector->layer_step_x << std::endl;
  yamlRead<bool>(algo_cfg_node["local_map"], "map_sliding_en", lidar_selector->map_sliding_en, false);
  yamlRead<double>(algo_cfg_node["local_map"], "half_map_length", lidar_selector->half_map_length, 100.);
  yamlRead<double>(algo_cfg_node["local_map"], "sliding_thresh", lidar_selector->sliding_thresh, 8.);
  yamlRead<double>(rs_node, "vis_down_leaf", lidar_selector->vis_down_leaf, 0.2);
  yamlRead<double>(rs_node, "VIO_freq_ratio", lidar_selector->VIO_freq_ratio, 1.0);
  yamlRead<double>(rs_node, "weight2_ref_depth",lidar_selector->weight2_ref_depth,10.);
  auto vio_pub_node = rs_node["vio_pub"];
  yamlRead<bool>(vio_pub_node, "pub_noise_cloud", lidar_selector->pub_noise_cloud, false);
  yamlRead<bool>(vio_pub_node, "pub_all_cloud", lidar_selector->pub_all_cloud, false);
  yamlRead<double>(vio_pub_node, "depth_color_range", lidar_selector->depth_color_range, 10);
  auto img_filter_node = rs_node["img_filter"];
  yamlRead<double>(img_filter_node, "delta_time", delta_time, 0.0);
  yamlRead<int>(img_filter_node, "cam_keep_period", cam_keep_period, 3);
  yamlRead<double>(img_filter_node, "outlier_threshold", outlier_threshold, 100);
  yamlRead<double>(img_filter_node, "ncc_thre", ncc_thre, 100);
  yamlRead<bool>(img_filter_node, "ncc_en", ncc_en, false);
  yamlRead<int>(img_filter_node, "map_value_thresh",lidar_selector->map_value_thresh,10);
  yamlRead<int>(img_filter_node, "remove_down_pixel",lidar_selector->remove_down_pixel,100000);
  yamlRead<double>(img_filter_node, "uniform_feature",lidar_selector->uniform_feature,0);
  yamlRead<int>(img_filter_node, "exp_num_per_grid",lidar_selector->exp_num_per_grid,0);
  yamlRead<int>(img_filter_node, "patch_num_max",lidar_selector->patch_num_max,999999);
  if(lidar_selector->patch_num_max < 50) {
    LERROR<<"patch_num_max must be larger!"<<REND;
    lidar_selector->patch_num_max = 50;
  }

  lidar_selector->lidar_degenerate_thresh = lidar_degenerate_thresh;
  lidar_selector->img_scaling_ratio = img_scaling_ratio;
  lidar_selector->debug = debug;
  lidar_selector->patch_size = patch_size * img_scaling_ratio;
  lidar_selector->outlier_threshold = outlier_threshold;
  lidar_selector->ncc_thre = ncc_thre;
  lidar_selector->SetTCL(T_C_L);
  lidar_selector->SetTIL(Lidar_offset_to_IMU, Lidar_rot_to_IMU);
  lidar_selector->state = &state;
  lidar_selector->state_propagat = &state_propagat;
  lidar_selector->NUM_MAX_ITERATIONS = NUM_MAX_ITERATIONS;
  IMG_POINT_COV /= (img_scaling_ratio * img_scaling_ratio);
  lidar_selector->img_point_cov = IMG_POINT_COV;
  lidar_selector->fx = cam_fx * img_scaling_ratio;
  lidar_selector->fy = cam_fy * img_scaling_ratio;
  lidar_selector->cx = cam_cx * img_scaling_ratio;
  lidar_selector->cy = cam_cy * img_scaling_ratio;
  lidar_selector->ncc_en = ncc_en;
  lidar_selector->light_min_thresh = light_min_thresh;
  lidar_selector->vis_degenerate_thresh = vis_degenerate_thresh;

  lidar_selector->init();

  // 复用RS_LOAM处理拖点
  YAML::Node LOAM_config_node;
  try {
    LOAM_config_node = YAML::LoadFile(cfg_path + "/RS_LOAM.yaml");
  } catch (const std::exception &e) {
    LERROR << "load LOAM_config_node error: " << e.what() << REND;
    exit(-1);
  }
  lidar_odometry_.reset(new RsLidarOdometry(LOAM_config_node["lidar_odometry"]));

  // save map
  xyzi_scan.reset(new PointCloudXYZI()); // xyzi scan
  rgb_scan.reset(new PointCloudXYZRGB);
  rgb_map.reset(new PointCloudXYZRGB());  //add save rbg map
  xyzi_map.reset(new PointCloudXYZI());  //add save xyzi map

  // debug
  fout_pre.open(log_dir+"/mat_pre.txt",ios::out); // 前向传播后（优化前）
  fout_out.open(log_dir+"/mat_out.txt",ios::out); // LIO/VIO优化后
  f_state_utm.open(log_dir+"/utm_LV_opt_I_pose_W.txt",ios::out);
  f_lidar_state_utm.open(log_dir+"/utm_L_opt_pose_W.txt",ios::out);

  f_LIO_t.open(log_dir+"/t_LIO.txt", ios::out);
//------------------------------------------------------------------------------------------------------
  LINFO << name() << ": init succeed!" << REND;
}

inline void FastLivoSlam::ReadParamters(const YAML::Node &node) {
  yamlRead<double>(node, "lidar_max_range", DET_RANGE, 50);
  yamlRead<int>(node, "img_enable", img_en, 1);
  yamlRead<int>(node, "lidar_enable", lidar_en, 1);
  yamlRead<int>(node, "debug", debug, 0);
  yamlRead<int>(node, "max_iteration", NUM_MAX_ITERATIONS, 4);
  yamlRead<double>(node, "laser_point_cov", LASER_POINT_COV, 0.001);
  yamlRead<double>(node, "img_point_cov", IMG_POINT_COV, 10);
  yamlRead<double>(node, "filter_size_surf", filter_size_surf_min, 0.5);
  yamlRead<double>(node, "filter_size_map", filter_size_map_min, 0.5);
  yamlRead<double>(node, "cube_side_length", cube_len, 200);
  auto mapping_node = node["mapping"];
  yamlRead<double>(node, "gyr_cov_scale", gyr_cov_scale, 1.0);
  yamlRead<double>(node, "acc_cov_scale", acc_cov_scale, 1.0);
  // 外參
  yamlRead<vector<double>>(mapping_node, "extrinsic_T", extrinT,
                           vector<double>());
  yamlRead<vector<double>>(mapping_node, "extrinsic_R", extrinR,
                           vector<double>());
  auto camera_node = node["camera"];
  yamlRead<vector<double>>(camera_node, "Pcl", cameraextrinT, vector<double>());
  yamlRead<vector<double>>(camera_node, "Rcl", cameraextrinR, vector<double>());

  T_I_L = Eigen::Matrix4d::Identity();
  T_I_L.block<3, 3>(0, 0) << MAT_FROM_ARRAY(extrinR);
  T_I_L.block<3, 1>(0, 3) << VEC_FROM_ARRAY(extrinT);
  yamlRead<bool>(node["rs"]["visualize"], "trans_imu_to_lidar",
                 trans_imu_to_lidar, false);
  if (trans_imu_to_lidar) {
    T_I_L_file = T_I_L; // 记录外参，直接补偿到原始消息，将使用的重置
    R_L_I = T_I_L_file.block<3, 3>(0, 0).transpose();
    T_I_L.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    T_I_L.block<3, 1>(0, 3) = R_L_I * T_I_L.block<3, 1>(0, 3);
  }

  T_C_L = Eigen::Matrix4d::Identity();
  T_C_L.block<3, 3>(0, 0) << MAT_FROM_ARRAY(cameraextrinR);
  T_C_L.block<3, 1>(0, 3) << VEC_FROM_ARRAY(cameraextrinT);

  Eigen::Matrix4d T_L_C = Eigen::Matrix4d::Identity();
  T_L_C.block<3, 3>(0, 0) = T_C_L.block<3, 3>(0, 0).transpose();
  T_L_C.block<3, 1>(0, 3) =
      T_L_C.block<3, 3>(0, 0) * (-T_C_L.block<3, 1>(0, 3));

  T_I_L_pose = Pose(T_I_L, 0);
  T_C_L_pose = Pose(T_C_L, 0);

  yamlRead<int>(node, "grid_size", grid_size, 40);
  yamlRead<int>(node, "patch_size", patch_size, 4);
  auto pcd_save_node = node["pcd_save"];
  yamlRead<bool>(pcd_save_node, "pcd_save_en", pcd_save_en, false);
  yamlRead<bool>(pcd_save_node, "ply_save_en", ply_save_en, false);
  yamlRead<string>(pcd_save_node, "save_frame_mode", save_frame_mode, "");
  yamlRead<double>(pcd_save_node, "voxel_size", voxel_size, 100);
  auto rs_node = node["rs"];
  yamlRead<double>(rs_node, "delay_t", delay_t, 0.5);
  yamlRead<bool>(rs_node, "lidar_time_is_tail", lidar_time_is_tail, true);
  yamlRead<double>(rs_node, "light_min_thresh", light_min_thresh, 0);
  yamlRead<bool>(rs_node, "use_map_proj", use_map_proj, false);
  yamlRead<double>(rs_node, "VIO_wait_time", VIO_wait_time, 10.);
  yamlRead<bool>(rs_node, "enable_normal_filter", enable_normal_filter, false);
  yamlRead<double>(rs_node, "norm_thresh", norm_thresh, 60.);
  yamlRead<double>(rs_node, "weight", weight, 1.);
  norm_thresh = norm_thresh / 180. * M_PI;
  yamlRead<bool>(rs_node, "rs_debug", rs_debug, false);
  yamlRead<bool>(rs_node, "save_log", save_log, false);
  yamlRead<bool>(rs_node, "print_log", print_log, false);
  yamlRead<bool>(rs_node, "downsample_source_cloud", downsample_source_cloud,
                 false);
  yamlRead<double>(rs_node, "x_thresh", x_thresh, 100.);
  yamlRead<double>(rs_node, "y_thresh", y_thresh, 100.);
  yamlRead<double>(rs_node, "z_thresh", z_thresh, 100.);
  yamlRead<double>(rs_node, "p2plane_thresh", p2plane_thresh, 1.0);

  auto vis_node = rs_node["visualize"];
  yamlRead<bool>(vis_node, "trans_imu_to_lidar", trans_imu_to_lidar, false);
  yamlRead<bool>(vis_node, "pub_dense_map", pub_dense_map, 1);
  yamlRead<bool>(vis_node, "show_rgb_map", show_rgb_map, false);
  yamlRead<bool>(vis_node, "show_lidar_map", show_lidar_map, false);

  auto dege_node = rs_node["degeneration"];
  yamlRead<double>(dege_node, "vis_degenerate_thresh", vis_degenerate_thresh,
                   0.35);
  yamlRead<double>(dege_node, "lidar_degenerate_thresh",
                   lidar_degenerate_thresh, 100);
  yamlRead<bool>(dege_node, "detect_lidar_degenetate", detect_lidar_degenetate,
                 false);
  yamlRead<bool>(dege_node, "reset_state_when_degenerate",
                 reset_state_when_degenerate, false);
  yamlRead<bool>(rs_node, "boundary_point_remove", boundary_point_remove,
                 false);
  yamlRead<double>(rs_node["img_filter"], "img_scaling_ratio",
                   img_scaling_ratio, 1.0);
  auto preprocess_node = node["preprocess"];
  yamlRead<double>(preprocess_node, "blind_max", blind_max, 0.5);
  yamlRead<double>(preprocess_node, "blind", blind, 100.0);

#ifdef ADAPTIVE_INIT
  double vrw, arw, sigma_ba, sigma_bg;
  yamlRead<double>(rs_node["imu"], "vrw", vrw, 1e-1);
  yamlRead<double>(rs_node["imu"], "arw", arw, 1e-2);
  yamlRead<double>(rs_node["imu"], "sigma_ba", sigma_ba, 1e-3);
  yamlRead<double>(rs_node["imu"], "sigma_bg", sigma_bg, 1e-4);
  livo_initial::PreIntergration::setNoiseStd(
    Eigen::Vector3d(vrw,vrw,vrw), 
    Eigen::Vector3d(arw,arw,arw), 
    Eigen::Vector3d(sigma_ba,sigma_ba,sigma_ba), 
    Eigen::Vector3d(sigma_bg,sigma_bg,sigma_bg)
    );
#endif
  if (rs_debug) {
    LWARNING << "T_I_L" << REND;
    LDEBUG << T_I_L << REND;
    LWARNING << "T_C_L" << REND;
    LDEBUG << T_C_L << REND;
    LWARNING << "T_L_C" << REND;
    LDEBUG << T_L_C << REND;
    auto T_I_C = T_I_L * T_L_C;
    LDEBUG << "T_I_C\n" << T_I_C << REND;
    auto T_C_I = T_I_C.inverse();
    LDEBUG << "T_C_I\n" << T_C_I << REND;

    string pcd_in_path, pcd_out_path;
    yamlRead<string>(pcd_save_node, "pcd_in", pcd_in_path, "");
    yamlRead<string>(pcd_save_node, "pcd_out", pcd_out_path, "");
    if (pcd_in_path.size() > 20 && pcd_out_path.size() > 20) {
      LINFO << "reading pcd file from: " << pcd_in_path << REND;
      pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
      if (pcl::io::loadPCDFile<PointType>(pcd_in_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read file input_cloud.pcd \n");
      }
      // LINFO << "transform pcd from I to C" << REND;
      // pcl::transformPointCloud(*cloud, *cloud, Eigen::Matrix4d(T_C_I));
      LINFO << "downsampling, voxel size: " << voxel_size << REND;
      auto cloud_ds = uniformSample<PointType>(cloud, voxel_size);
      auto last_str = pcd_out_path.substr(pcd_out_path.size() - 4);
      if (last_str == ".ply") {
        LINFO << "save ply to " << pcd_out_path << REND;
        pcl::io::savePCDFileBinary(pcd_out_path, *cloud_ds);
      } else if (last_str == ".pcd") {
        LINFO << "save pcd to " << pcd_out_path << REND;
        pcl::io::savePCDFileBinary(pcd_out_path, *cloud_ds);
      }
    }
  }
}


} // namespace slam
} // namespace robosense
