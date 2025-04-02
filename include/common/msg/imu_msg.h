#pragma once
#include <iostream>
#include <Eigen/Eigen>
#include <array>
#include <memory>

namespace robosense {
namespace slam {

struct Header {
  std::string frame_id;
  uint64_t timestamp{0};
  uint32_t seq_id{0};
  double ToSec() const {
    return static_cast<double>(timestamp) / 1000000000UL;
  }
};

struct Quaternion {
  double x{0.};
  double y{0.};
  double z{0.};
  double w{0.};
};

struct Vector3 {
  double x{0.};
  double y{0.};
  double z{0.};
};

struct Imu {
  using Ptr = std::shared_ptr<Imu>;
  using ConstPtr = std::shared_ptr<Imu const>;

  Header header;
  Quaternion orientation;
  std::array<double, 9> orientation_covariance;
  Vector3 angular_velocity;
  std::array<double, 9> angular_velocity_covariance;
  Vector3 linear_acceleration;
  std::array<double, 9> linear_acceleration_covariance;
};

//
inline void rotateVector(const Vector3 &vec, const Eigen::Matrix3d &R,
                         Vector3 &result) {
  result.x = R(0, 0) * vec.x + R(0, 1) * vec.y + R(0, 2) * vec.z;
  result.y = R(1, 0) * vec.x + R(1, 1) * vec.y + R(1, 2) * vec.z;
  result.z = R(2, 0) * vec.x + R(2, 1) * vec.y + R(2, 2) * vec.z;
}
}  // namespace slam
}  // namespace robosense
