/*
 * math_utils.h
 *
 *  Created on: Jul 20, 2012
 *      Author: cforster
 */

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_


#include <Eigen/Core>
#include <Eigen/StdVector>
#include <sophus/se3.h>

namespace vk
{

using namespace Eigen;
using namespace std;
using namespace Sophus;

inline double norm_max(const Eigen::VectorXd & v)
{
  double max = -1;
  for (int i=0; i<v.size(); i++)
  {
    double abs = fabs(v[i]);
    if(abs>max){
      max = abs;
    }
  }
  return max;
}

inline Vector2d project2d(const Vector3d& v)
{
  return v.head<2>()/v[2];
}

template<class T>
T getMedian(vector<T>& data_vec)
{
  assert(!data_vec.empty());
  typename vector<T>::iterator it = data_vec.begin()+floor(data_vec.size()/2);
  nth_element(data_vec.begin(), it, data_vec.end());
  return *it;
}

} // end namespace vk

#endif /* MATH_UTILS_H_ */
