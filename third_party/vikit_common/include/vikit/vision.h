/*
 * vision.h
 *
 *  Created on: May 14, 2013
 *      Author: cforster
 */

#ifndef VIKIT_VISION_H_
#define VIKIT_VISION_H_

#include <vikit/aligned_mem.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace vk
{

//! Return value between 0 and 255
//! WARNING This function does not check whether the x/y is within the border
inline float
interpolateMat_8u(const cv::Mat& mat, float u, float v)
{
  assert(mat.type()==CV_8U);
  int x = floor(u);
  int y = floor(v);
  float subpix_x = u-x;
  float subpix_y = v-y;

  float w00 = (1.0f-subpix_x)*(1.0f-subpix_y);
  float w01 = (1.0f-subpix_x)*subpix_y;
  float w10 = subpix_x*(1.0f-subpix_y);
  float w11 = 1.0f - w00 - w01 - w10;

  const int stride = mat.step.p[0];
  unsigned char* ptr = mat.data + y*stride + x;
  return w00*ptr[0] + w01*ptr[stride] + w10*ptr[1] + w11*ptr[stride+1];
}

void halfSample(const cv::Mat& in, cv::Mat& out);

float shiTomasiScore(const cv::Mat& img, int u, int v);

} // namespace vk

#endif // VIKIT_VISION_H_
