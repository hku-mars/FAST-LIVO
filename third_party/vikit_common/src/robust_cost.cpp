/**
* This file is part of dvo.
*
* Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/data/software/dvo>.
*
* dvo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* dvo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/

#include <numeric>
#include <algorithm>
#include <vikit/robust_cost.h>
#include <vikit/math_utils.h>

namespace vk {
namespace robust_cost {

const float TDistributionScaleEstimator::INITIAL_SIGMA = 5.0f;
const float TDistributionScaleEstimator::DEFAULT_DOF = 5.0f;

TDistributionScaleEstimator::
TDistributionScaleEstimator(const float dof) :
  dof_(dof),
  initial_sigma_(INITIAL_SIGMA)
{}

float TDistributionScaleEstimator::
compute(std::vector<float>& errors) const
{
  float initial_lamda = 1.0f / (initial_sigma_ * initial_sigma_);
  int num = 0;
  float lambda = initial_lamda;
  int iterations = 0;
  do
  {
    ++iterations;
    initial_lamda = lambda;
    num = 0;
    lambda = 0.0f;

    for(std::vector<float>::iterator it=errors.begin(); it!=errors.end(); ++it)
    {
      if(std::isfinite(*it))
      {
        ++num;
        const float error2 = (*it)*(*it);
        lambda += error2 * ( (dof_ + 1.0f) / (dof_ + initial_lamda * error2) );
      }
    }
    lambda = float(num) / lambda;
  } while(std::abs(lambda - initial_lamda) > 1e-3);

  return std::sqrt(1.0f / lambda);
}

const float MADScaleEstimator::NORMALIZER = 1.48f; // 1 / 0.6745

float MADScaleEstimator::
compute(std::vector<float>& errors) const
{
  // error must be in absolute values!
  return NORMALIZER * vk::getMedian(errors);
}

float NormalDistributionScaleEstimator::
compute(std::vector<float>& errors) const
{
  const float mean = std::accumulate(errors.begin(), errors.end(), 0)/errors.size();
  float var = 0.0;
  std::for_each(errors.begin(), errors.end(), [&](const float d) {
      var += (d - mean) * (d - mean);
  });
  return std::sqrt(var); // return standard deviation
}

const float TukeyWeightFunction::DEFAULT_B = 4.6851f;

TukeyWeightFunction::TukeyWeightFunction(const float b)
{
  configure(b);
}

float TukeyWeightFunction::value(const float& x) const
{
  const float x_square = x * x;
  if(x_square <= b_square)
  {
    const float tmp = 1.0f - x_square / b_square;
    return tmp * tmp;
  }
  else
  {
    return 0.0f;
  }
}

void TukeyWeightFunction::
configure(const float& param)
{
  b_square = param * param;
}

const float TDistributionWeightFunction::DEFAULT_DOF = 5.0f;

TDistributionWeightFunction::
TDistributionWeightFunction(const float dof)
{
  configure(dof);
}

float TDistributionWeightFunction::
value(const float & x) const
{
  return ((dof_ + 1.0f) / (dof_ + (x * x)));
}

void TDistributionWeightFunction::
configure(const float& param)
{
  dof_ = param;
  normalizer_ = dof_ / (dof_ + 1.0f);
}

const float HuberWeightFunction::DEFAULT_K = 1.345f;

HuberWeightFunction::
HuberWeightFunction(const float k)
{
  configure(k);
}

void HuberWeightFunction::
configure(const float& param)
{
  k = param;
}

float HuberWeightFunction::
value(const float& t) const
{
  const float t_abs = std::abs(t);
  if(t_abs < k)
    return 1.0f;
  else
    return k / t_abs;
}

} // namespace robust_cost
} // namespace vk


