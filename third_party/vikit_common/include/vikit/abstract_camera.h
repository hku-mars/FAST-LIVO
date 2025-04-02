/*
 * abstract_camera.h
 *
 *  Created on: Jul 23, 2012
 *      Author: cforster
 */

#ifndef ABSTRACT_CAMERA_H_
#define ABSTRACT_CAMERA_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace vk
{

using namespace std;
using namespace Eigen;

class AbstractCamera
{
protected:

  int width_;   // TODO cannot be const because of omni-camera model
  int height_;

public:
  virtual void undistortImage(const cv::Mat& raw, cv::Mat& rectified){};
  AbstractCamera() {}; // need this constructor for omni camera
  AbstractCamera(int width, int height) : width_(width), height_(height) {};

  virtual ~AbstractCamera() {};

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Vector3d
  cam2world(const double& x, const double& y) const = 0;

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Vector3d
  cam2world(const Vector2d& px) const = 0;

  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const = 0;

  /// projects unit plane coordinates to camera coordinates
  virtual Vector2d
  world2cam(const Vector2d& uv) const = 0;

  virtual double
  errorMultiplier2() const = 0;

  virtual double
  errorMultiplier() const = 0;

  inline int width() const { return width_; }

  inline int height() const { return height_; }

  inline bool isInFrame(const Vector2i & obs, int boundary=0) const
  {
    if(obs[0]>=boundary && obs[0]<width()-boundary
        && obs[1]>=boundary && obs[1]<height()-boundary)
      return true;
    return false;
  }

  inline bool isInFrame(const Vector2i &obs, int boundary, int level) const
  {
    if(obs[0] >= boundary && obs[0] < width()/(1<<level)-boundary
        && obs[1] >= boundary && obs[1] <height()/(1<<level)-boundary)
      return true;
    return false;
  }
};

} // end namespace CSfM

#endif /* ABSTRACT_CAMERA_H_ */
