#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "common_lib.h"
#include "livo_initial/common_data.hpp"
#include "livo_initial/pre_integration.h"
#include "livo_initial/feature_tracker.h"

namespace livo_initial{

class Initializer{
public:
  Initializer(const Eigen::Matrix4d _T_imu_from_lidar, const Eigen::Matrix4d _T_camera_from_lidar, const double _g_norm);
  ~Initializer();
  void setCameraParam(const int _width, const int _height, const double _fx, const double _fy, const double _cx, const double _cy, std::vector<double> &_distortion_coeffs);
  bool alignToWorld(std::map<double, Frame>& _frames, Eigen::Vector3d& _g_world_out, bool _has_sacle=false);
  void alignWorldToGravity(std::map<double, Frame>& _frames, const Eigen::Vector3d& _g_world, Eigen::Vector3d& _g_gravity_out);
  void addLidar(double _time, PointCloudXYZI::Ptr _lidar_ptr);
  void addImage(double _time, cv::Mat _img);
  void addImageInfo(ImageInfo &_img_info);
  void addImu(double _time, Eigen::Vector3d _fb, Eigen::Vector3d _wib_b);
  void process();
  double getLastFinishedTime() const;
  bool isProcessFinish() const;
  bool getInitResult(double &_time, Eigen::Matrix3d &_Rni, Eigen::Vector3d &_bg, Eigen::Vector3d &_vn, Eigen::Vector3d &_gn) const;
  void clearBuf();
private:
  std::vector<Frame> dataSync();
  bool recoverBVGS(std::map<double, Frame>& _frames, Eigen::Vector3d &_g_world_out, double& _s_out, bool _has_sacle=false);
  void solveBg(std::map<double, Frame>& _frames);
  bool solveVGS(std::map<double, Frame>& _frames, Eigen::Vector3d &_g_world_out, double& _s_out, bool _has_sacle=false);
  bool refineVGS(std::map<double, Frame>& _frames, const Eigen::Vector3d _g_world_init, Eigen::Vector3d &_g_world_out, double& _s_out, bool _has_sacle=false);
  Eigen::MatrixXd tangentBasis(Eigen::Vector3d &_vec);
  Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R) const;
  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr) const;
  Eigen::Matrix3d g2R(const Eigen::Vector3d &_vec) const;

  void visualTrack();
  bool addFeatureCheckParallax(int _frame_count, const std::map<FeatureIdType, Eigen::Vector4d> &_featureId_xyuv);
  double compensatedParallax2(const FeaturePerId &_it_per_id, int _frame_count);
  bool initialStructure();
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getCorresponding(int _frame_count_l, int _frame_count_r);
  bool solveRelativeRT(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &_corres, Eigen::Matrix3d &_Rotation, Eigen::Vector3d &_Translation);
  bool relativePose(Eigen::Matrix3d &_relative_R, Eigen::Vector3d &_relative_T, int &_l);
  bool visualInitialAlign();
  void slideWindow();
  void slideWindowNew();
  void slideWindowOld();
  bool checkStop(Frame &_frame1, Frame &_frame2);
  bool staticInitialize();

  Eigen::Matrix3d R_imu_from_lidar_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_imu_from_lidar_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_imu_from_camera_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_imu_from_camera_ = Eigen::Vector3d::Zero();
  double g_norm_;

  std::queue<std::pair<double, PointCloudXYZI::Ptr>> lidar_buf_;
  std::queue<std::pair<double, cv::Mat>> img_buf_;
  std::queue<ImageInfo> img_info_buf_;
  std::queue<ImuData> imu_buf_;

  std::mutex mutex_data_;
  std::condition_variable con_process_;
  std::atomic_bool process_active_{true};
  std::thread process_thread_;

  Eigen::Vector3d g_world_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_gravity_ = Eigen::Vector3d::Zero();

  double time_result_ = 0.;
  Eigen::Matrix3d Rni_result_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d bg_result_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vn_result_= Eigen::Vector3d::Zero();
  Eigen::Vector3d g_world_result_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_gravity_result_ = Eigen::Vector3d::Zero();

  std::mutex mutex_visual_;
  std::condition_variable con_visual_;
  std::atomic_bool visual_active_{true};
  std::thread visual_thread_;
  
  std::map<double, Frame> all_image_frame_;
  std::list<FeaturePerId> feature_list_;
  std::array<double, WINDOW_SIZE+1> keyframe_times_;
  std::array<Eigen::Vector3d, WINDOW_SIZE+1> keyframe_bas_;
  std::array<Eigen::Vector3d, WINDOW_SIZE+1> keyframe_bgs_;
  int frame_count_{0};
  MarginalizationFlag marginalization_flag_;
  FeatureTracker feature_tracker_;
  double fx_inv_;
  double fy_inv_;

  std::atomic<double> time_finished_last_{0.};

  // dataSync相关的成员变量
  double dataSync_time_last_{-1.};
  Eigen::Vector3d dataSync_fb_last_{0.,0.,0.};
  Eigen::Vector3d dataSync_wib_b_last_{0.,0.,0.};

  // 各个add函数相关的成员变量
  double addLidar_time_last_{0.};
  double addImage_time_last_{0.};
  double addImageInfo_time_last_{0.};
  double addImu_time_last_{0.};
};


} // namespace livo_initial
#endif // INITIALIZER_H