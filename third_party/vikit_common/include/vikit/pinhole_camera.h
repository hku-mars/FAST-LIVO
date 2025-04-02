/*
 * pinhole_camera.h
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#ifndef PINHOLE_CAMERA_H_
#define PINHOLE_CAMERA_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <vikit/abstract_camera.h>
#include <opencv2/opencv.hpp>

namespace vk {

using namespace std;
using namespace Eigen;

class PinholeCamera : public AbstractCamera {

private:
  const double fx_, fy_;
  const double cx_, cy_;
  bool distortion_;             //!< is it pure pinhole model or has it radial distortion?
  double d_[8];                 //!< distortion parameters, see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  cv::Mat cvK_, cvD_;
  cv::Mat undist_map1_, undist_map2_;
  bool use_optimization_;
  Matrix3d K_;
  Matrix3d K_inv_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeCamera(double width, double height,
                double fx, double fy, double cx, double cy,
                double d0=0.0, double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0, double d5=0.0, double d6=0.0, double d7=0.0);

  ~PinholeCamera();

  void
  initUnistortionMap();

  virtual Vector3d
  cam2world(const double& x, const double& y) const;

  virtual Vector3d
  cam2world(const Vector2d& px) const;

  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const;

  virtual Vector2d
  world2cam(const Vector2d& uv) const;

  const Vector2d focal_length() const
  {
    return Vector2d(fx_, fy_);
  }

  virtual double errorMultiplier2() const
  {
    return fabs(fx_);
  }

  virtual double errorMultiplier() const
  {
    return fabs(4.0*fx_*fy_);
  }

  inline const Matrix3d& K() const { return K_; };
  inline const Matrix3d& K_inv() const { return K_inv_; };
  inline double fx() const { return fx_; };
  inline double fy() const { return fy_; };
  inline double cx() const { return cx_; };
  inline double cy() const { return cy_; };
  inline double d0() const { return d_[0]; };
  inline double d1() const { return d_[1]; };
  inline double d2() const { return d_[2]; };
  inline double d3() const { return d_[3]; };
  inline double d4() const { return d_[4]; };

  void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

};

} // end namespace vk


#endif /* PINHOLE_CAMERA_H_ */
