#ifndef COMMON_DATA_H
#define COMMON_DATA_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "common_lib.h"
#include "livo_initial/pre_integration.h"

namespace livo_initial
{
#define WINDOW_SIZE 10
#define FOCAL_LENGTH 460.0
#define MIN_PARALLAX 0.02173913 // 10./FOCAL_LENGTH

typedef uint32_t FeatureIdType;

enum CameraType
{
  PINHOLE = 0,
  MEI = 1,
};
enum MarginalizationFlag
{
  MARGIN_OLD = 0,
  MARGIN_SECOND_NEW = 1
};

struct ImuData
{
  ImuData(double _time, const Eigen::Vector3d _fb, const Eigen::Vector3d _wib_b)
  {
    timestamp = _time;
    fb = _fb;
    wib_b = _wib_b;
  }
  double timestamp{0.};
  Eigen::Vector3d fb{0., 0., 0.};
  Eigen::Vector3d wib_b{0., 0., 0.};
};

struct ImageInfo
{
  ImageInfo(double _timestamp, const cv::Mat &_img, std::map<FeatureIdType, Eigen::Vector4d> &_featureId_xyuv)
  {
    timestamp = _timestamp;
    img = _img;
    featureId_xyuv = _featureId_xyuv;
  }
  double timestamp{0.};
  cv::Mat img;
  std::map<FeatureIdType, Eigen::Vector4d> featureId_xyuv;
  std::vector<int> idx_idx2d;
  std::vector<std::vector<int>> idx2d_idx;
};

struct Frame
{
  double timestamp{0.};
  std::shared_ptr<PreIntergration> pre_integration;
  cv::Mat img;
  PointCloudXYZI::Ptr lidar;
  std::map<FeatureIdType, Eigen::Vector4d> featureId_xyuv;
  std::vector<double> imu_time_cache;
  std::vector<Eigen::Vector3d> fb_cache;
  std::vector<Eigen::Vector3d> wib_b_cache;

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Vector3d vb = Eigen::Vector3d::Zero();

  Eigen::Matrix3d R_world_from_camera = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_world_from_camera = Eigen::Vector3d::Zero();

  Eigen::Matrix3d R_world_from_imu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_world_from_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d vb_in_world = Eigen::Vector3d::Zero();

  Eigen::Matrix3d R_gravity_from_imu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_gravity_from_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d vb_in_gravity = Eigen::Vector3d::Zero();

  bool is_key_frame{false};
  bool is_stop{false};

  void preIntegration(const Eigen::Vector3d _ba, const Eigen::Vector3d _bg)
  {
    if(pre_integration)
    {
      pre_integration.reset();
    }
    pre_integration = std::make_shared<PreIntergration>(imu_time_cache.front(), fb_cache.front(), wib_b_cache.front(), _ba, _bg);
    for(int i=1; i<imu_time_cache.size(); i++)
    {
      pre_integration->addImuData(imu_time_cache[i], fb_cache[i], wib_b_cache[i]);
    }
  }
};

struct FeaturePerFrame
{
public:
  FeaturePerFrame(const Eigen::Matrix<double, 4, 1> &_point)
  {
      point.x() = _point(0);
      point.y() = _point(1);
      point.z() = 1.;
      uv.x() = _point(3);
      uv.y() = _point(4);
  }
  Eigen::Vector3d point;
  Eigen::Vector2d uv;
};

struct FeaturePerId
{
public:
  const FeatureIdType feature_id;
  int start_frame;
  std::vector<FeaturePerFrame> feature_per_frame;

  FeaturePerId(int _feature_id, int _start_frame)
      : feature_id(_feature_id), start_frame(_start_frame)
  {}

  int endFrame() const
  {
    return start_frame + feature_per_frame.size() - 1;
  }
};
}
#endif // COMMON_DATA_H