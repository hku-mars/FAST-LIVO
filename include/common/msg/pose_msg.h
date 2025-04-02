#pragma once
#include <iostream>
#include <Eigen/Eigen>
#include <fstream>
#include <iomanip>

// 官方定义
namespace fast_livo {
struct Pose6D {
  double offset_time;
  std::array<double, 3> acc;
  std::array<double, 3> gyr;
  std::array<double, 3> vel;
  std::array<double, 3> pos;
  std::array<double, 9> rot;
};
}  // namespace fast_livo

// RS定义
enum class STATUS /* status aft receive a lidar frame */ 
{
  IDLE = 0,
  LOW_ACCURACY = 1,
  NORMAL = 2,
  LOST = 3,
  NO_ENOUGH_MAP = 4,
  LOW_ACCURACY_RPZ = 5,
  LOW_ACCURACY_X = 6,
};

struct Pose
{
  std::string source = "lidar";
  std::string status = "Null";
  STATUS status_code = STATUS::IDLE;
  double timestamp = 0;  /* lidar timestamp, s */ 
  int zone = 1;
  double longitude = 0;
  double latitude = 0;
  double height = 0;
  double enu_heading = 0;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Pose() = default; 
  Pose(const Eigen::Matrix4d& T, double _timestamp) 
  {
    xyz = T.block<3,1>(0,3);
    q = Eigen::Quaterniond(T.block<3,3>(0,0));
    q.normalize();
    timestamp = _timestamp;
  }
  Pose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double _timestamp) 
  {
    xyz = t;
    q = Eigen::Quaterniond(R);
    q.normalize();
    timestamp = _timestamp;
  }
  Pose(const Eigen::Quaterniond& _q, const Eigen::Vector3d& _t, double _timestamp) 
  {
    xyz = _t;
    q = _q;
    timestamp = _timestamp;
  }
  void setStatus(STATUS _status)
  {
    status_code = _status;
    switch (status_code) {
      case STATUS::IDLE:
        status = "IDLE";
      case STATUS::LOW_ACCURACY:
        status = "LOW_ACCURACY";
      case STATUS::LOST:
        status = "LOST";
      case STATUS::NORMAL:
        status = "NORMAL";
      case STATUS::NO_ENOUGH_MAP:
        status = "NO_ENOUGH_MAP";
      case STATUS::LOW_ACCURACY_RPZ:
        status = "LOW_ACCURACY_RPZ";
      case STATUS::LOW_ACCURACY_X:
        status = "LOW_ACCURACY_X"; 
    }
  }

  void print(const std::string& prefix="") const
  {
    std::cout << "------------------" << prefix << "------------------" << std::endl;
    std::cout << "source    : " << source << std::endl;
    std::cout << "Status    : " << status << std::endl;
    std::cout << "timestamp : " << std::fixed << timestamp << std::endl;
    std::cout << "xyz       : " << xyz.transpose() << std::endl;
    std::cout << "q         : " << q.coeffs().transpose() << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;
  }

  Eigen::Matrix4d transform() const
  {
    Eigen::Affine3d T = Eigen::Translation3d(xyz.x(), xyz.y(), xyz.z()) * q;
    return T.matrix();
  }

  void updatePose(Pose& delta_pose) {
    q = delta_pose.q * q;
    xyz = delta_pose.q * xyz + delta_pose.xyz;
  }

  void updatePoseRight(Pose& delta_pose) {
    xyz = q * delta_pose.xyz + xyz;
    q = q * delta_pose.q;
  }

  Pose inverse() {
    Pose pose_inv;
    pose_inv.q = q.inverse();
    pose_inv.xyz = -(pose_inv.q * xyz);
    pose_inv.timestamp = timestamp;
    pose_inv.source = source;
    return pose_inv;
  }

  void savePose(std::string path) {
    std::ofstream ofs;
    ofs.open(path, std::ios::out | std::ios::app);

    ofs << std::fixed << std::setprecision(6) << timestamp << " " << xyz.x()
        << " " << xyz.y() << " " << xyz.z() << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << "\n";
    ofs.close();
  }
};

struct Velocity
{
  Eigen::Vector3d linear = Eigen::Vector3d::Zero(); /* vel x y z */ 
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();  /* vel roll, pitch, yaw */ 
};