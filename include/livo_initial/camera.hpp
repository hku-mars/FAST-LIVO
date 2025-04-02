#ifndef CAMERA_H
#define CAMERA_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace livo_initial
{
class CameraBase
{
public:
  CameraBase(const int _width, const int _height, const std::vector<double> &_intrinsics, const std::vector<double> &_distortion_coeffs)
  : width_(_width), height_(_height), intrinsics_(_intrinsics), distortion_coeffs_(_distortion_coeffs)
  {}
  ~CameraBase() = default;
  virtual void pixelToUnDistNormalized(double _u, double _v, double &_x, double &_y) const = 0;
  virtual void UnDistnormalizedToPixel(double _x, double _y, double &_u, double &_v) const = 0;
  virtual void undistortImage(cv::InputArray _src, cv::OutputArray _dst) const = 0;
  virtual bool pixelInBorder(double _u, double _v, int _border_size=1) const = 0;
  virtual void distortion(const double _x, const double _y, double &_xd, double &_yd) const = 0;
protected:
  int width_{0};
  int height_{0};
  std::vector<double> intrinsics_;
  std::vector<double> distortion_coeffs_;
};

class PinholeCamera : public CameraBase
{
public:
  PinholeCamera(const int _width, const int _height, const std::vector<double> &_intrinsics, const std::vector<double> &_distortion_coeffs)
  : CameraBase(_width, _height, _intrinsics, _distortion_coeffs)
  {
    fx_ = intrinsics_[0];
    fy_ = intrinsics_[1];
    cx_ = intrinsics_[2];
    cy_ = intrinsics_[3];

    k1_ = distortion_coeffs_[0];
    k2_ = distortion_coeffs_[1];
    p1_ = distortion_coeffs_[2];
    p2_ = distortion_coeffs_[3];
    k3_ = distortion_coeffs_[4];
    k4_ = distortion_coeffs_[5];
    k5_ = distortion_coeffs_[6];
    k6_ = distortion_coeffs_[7];

    has_distortion_ = std::abs(k1_) > 1e-8;

    K_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    D_ = (cv::Mat_<float>(1, 8) << k1_, k2_, p1_, p2_, k3_, k4_, k5_, k6_);

    cv::initUndistortRectifyMap(K_, D_,
                           std::vector<float>(), K_,
                           cv::Size(width_, height_),
                           CV_32FC1, mapX_, mapY_);
  }
  virtual void pixelToUnDistNormalized(double _u, double _v, double &_x, double &_y) const
  {
    if(has_distortion_)
    {
#if 0
      cv::Point2f uv(_u,_v), px;
      const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
      cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
      cv::undistortPoints(src_pt, dst_pt, K_, D_);
      _x = px.x;
      _y = px.y;
#else
      double mx_d = (_u - cx_) / fx_;
      double my_d = (_v - cy_) / fy_;
      // Recursive distortion model
      int n = 8;
      double du_x, du_y;
      distortion(mx_d, my_d, du_x, du_y);
      // Approximate value
      double mx_u = mx_d - du_x;
      double my_u = my_d - du_y;

      for (int i = 1; i < n; ++i)
      {
          distortion(mx_u, my_u, du_x, du_y);
          mx_u = mx_d - du_x;
          my_u = my_d - du_y;
      }
      _x = mx_u;
      _y = my_u;
#endif
    }
    else
    {
      _x = (_u - cx_) / fx_;
      _y = (_v - cy_) / fy_;
    }
  }
  virtual void UnDistnormalizedToPixel(double _x, double _y, double &_u, double &_v) const
  {
    if(has_distortion_)
    {
      const double u2 = _x * _x;
      const double uv = _x * _y;
      const double v2 = _y * _y;
      const double r2 = u2 + v2;
      const double r4 = r2 * r2;
      const double r6 = r4 * r2;
      const double a1 = 2 * uv;
      const double a2 = r2 + 2*u2;
      const double a3 = r2 + 2*v2 ;

      const double cdist = 1. + k1_ * r2 + k2_ * r4 + k3_ * r6;
      const double icdist2 = 1. / ( 1. + k4_ * r2 + k5_ * r4 + k6_ * r6);
      double xd = _x * cdist * icdist2 + p1_ * a1 + p2_ * a2;
      double yd = _y * cdist * icdist2 + p1_ * a3 + p2_ * a1;
      _u = fx_ * xd + cx_;
      _v = fy_ * yd + cy_;
    }
    else
    {
      _u = fx_ * _x + cx_;
      _v = fy_ * _y + cy_;
    }
  }
  virtual void undistortImage(cv::InputArray _src, cv::OutputArray _dst) const
  {
    cv::remap(_src, _dst, mapX_, mapY_, cv::INTER_LINEAR);
  }
  virtual bool pixelInBorder(double _u, double _v, int _border_size=1) const
  {
    int img_x = std::round(_u);
    int img_y = std::round(_v);
    return _border_size <= img_x && img_x < width_ - _border_size && _border_size <= img_y && img_y < height_ - _border_size;
  }
  virtual void distortion(const double _x, const double _y, double &_xd, double &_yd) const
  {
    const double u2 = _x * _x;
    const double uv = _x * _y;
    const double v2 = _y * _y;
    const double r2 = u2 + v2;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double a1 = 2 * uv;
    const double a2 = r2 + 2*u2;
    const double a3 = r2 + 2*v2 ;

    const double cdist = 1. + k1_ * r2 + k2_ * r4 + k3_ * r6;
    const double icdist2 = 1. / ( 1. + k4_ * r2 + k5_ * r4 + k6_ * r6);
    _xd = _x * cdist * icdist2 + p1_ * a1 + p2_ * a2 - _x;
    _yd = _y * cdist * icdist2 + p1_ * a3 + p2_ * a1 - _y;
  }
private:
  cv::Mat K_;
  cv::Mat D_;
  cv::Mat mapX_, mapY_;
  double fx_{0.};
  double fy_{0.};
  double cx_{0.};
  double cy_{0.};
  double k1_{0.};
  double k2_{0.};
  double p1_{0.};
  double p2_{0.};
  double k3_{0.};
  double k4_{0.};
  double k5_{0.};
  double k6_{0.}; 
  bool has_distortion_{false};
  
};

}
#endif // CAMERA_H