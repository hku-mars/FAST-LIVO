/*
 * pinhole_camera.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>

namespace vk {

PinholeCamera::
PinholeCamera(double width, double height,
              double fx, double fy,
              double cx, double cy,
              double d0, double d1, double d2, double d3, double d4, double d5, double d6, double d7) :
              AbstractCamera(width, height),
              fx_(fx), fy_(fy), cx_(cx), cy_(cy),
              distortion_(fabs(d0) > 0.0000001),
              undist_map1_(height_, width_, CV_16SC2),
              undist_map2_(height_, width_, CV_16SC2),
              use_optimization_(false)
{
  d_[0] = d0; d_[1] = d1; d_[2] = d2; d_[3] = d3; d_[4] = d4; d_[5] = d5; d_[6] = d6; d_[7] = d7;
  cvK_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cvD_ = (cv::Mat_<float>(1, 8) << d_[0], d_[1], d_[2], d_[3], d_[4], d_[5], d_[6], d_[7]);
  cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<double>::eye(3,3), cvK_,
                              cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_);
  K_ << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
  K_inv_ = K_.inverse();
}

PinholeCamera::
~PinholeCamera()
{}

Vector3d PinholeCamera::
cam2world(const double& u, const double& v) const
{
  Vector3d xyz;
  // if(!distortion_)
  if(true)
  {
    xyz[0] = (u - cx_)/fx_;
    xyz[1] = (v - cy_)/fy_;
    xyz[2] = 1.0;
  }
  else
  {
    cv::Point2f uv(u,v), px;
    const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
    cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
    cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
    xyz[0] = px.x;
    xyz[1] = px.y;
    xyz[2] = 1.0;
  }
  return xyz.normalized();
}

Vector3d PinholeCamera::
cam2world (const Vector2d& uv) const
{
  return cam2world(uv[0], uv[1]);
}

Vector2d PinholeCamera::
world2cam(const Vector3d& xyz) const
{
  return world2cam(project2d(xyz));
}

Vector2d PinholeCamera::
world2cam(const Vector2d& uv) const
{
  Vector2d px;
  // if(!distortion_)
  if(true)
  {
    px[0] = fx_*uv[0] + cx_;
    px[1] = fy_*uv[1] + cy_;
  }
  else
  {
    const double k1 = d_[0];
    const double k2 = d_[1];
    const double p1 = d_[2];
    const double p2 = d_[3];
    const double k3 = d_[4];
    const double k4 = d_[5];
    const double k5 = d_[6];
    const double k6 = d_[7];

    const double u = uv[0];
    const double v = uv[1];
    const double u2 = u * u;
    const double uv = u * v;
    const double v2 = v * v;
    const double r2 = u2 + v2;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double a1 = 2 * uv;
    const double a2 = r2 + 2*u2;
    const double a3 = r2 + 2*v2 ;

    const double cdist = 1. + k1 * r2 + k2 * r4 + k3 * r6;
    const double icdist2 = 1. / ( 1. + k4 * r2 + k5 * r4 + k6 * r6);
    double xd = u * cdist * icdist2 + 2. * p1 * a1 + p2 * a2;
    double yd = v * cdist * icdist2 + 2. * p1 * a3 + p2 * a1;
    px[0] = xd*fx_ + cx_;
    px[1] = yd*fy_ + cy_;
  }
  return px;
}

void PinholeCamera::
undistortImage(const cv::Mat& raw, cv::Mat& rectified)
{
  if(distortion_)
    cv::remap(raw, rectified, undist_map1_, undist_map2_, cv::INTER_LINEAR);
  else
    rectified = raw.clone();
}

} // end namespace vk
