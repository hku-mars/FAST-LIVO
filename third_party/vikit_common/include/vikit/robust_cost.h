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

#ifndef VIKIT_ROBUST_COST_H_
#define VIKIT_ROBUST_COST_H_

#include <vector>
#include <stdlib.h>
#include <memory>
namespace vk {
namespace robust_cost {

// interface for scale estimators
class ScaleEstimator
{
public:
  virtual ~ScaleEstimator() {};
  virtual float compute(std::vector<float>& errors) const = 0;
};
typedef std::shared_ptr<ScaleEstimator> ScaleEstimatorPtr;

class UnitScaleEstimator : public ScaleEstimator
{
public:
  UnitScaleEstimator() {}
  virtual ~UnitScaleEstimator() {}
  virtual float compute(std::vector<float>& errors) const { return 1.0f; };
};

// estimates scale by fitting a t-distribution to the data with the given degrees of freedom
class TDistributionScaleEstimator : public ScaleEstimator
{
public:
  TDistributionScaleEstimator(const float dof = DEFAULT_DOF);
  virtual ~TDistributionScaleEstimator() {};
  virtual float compute(std::vector<float>& errors) const;

  static const float DEFAULT_DOF;
  static const float INITIAL_SIGMA;
protected:
  float dof_;
  float initial_sigma_;
};

// estimates scale by computing the median absolute deviation
class MADScaleEstimator : public ScaleEstimator
{
public:
  MADScaleEstimator() {};
  virtual ~MADScaleEstimator() {};
  virtual float compute(std::vector<float>& errors) const;

private:
  static const float NORMALIZER;;
};

// estimates scale by computing the standard deviation
class NormalDistributionScaleEstimator : public ScaleEstimator
{
public:
  NormalDistributionScaleEstimator() {};
  virtual ~NormalDistributionScaleEstimator() {};
  virtual float compute(std::vector<float>& errors) const;
private:
};

/**
 * Interface for weight functions. A weight function is the first derivative of a symmetric robust function p(sqrt(t)).
 * The errors are assumed to be normalized to unit variance.
 *
 * See:
 *   "Lucas-Kanade 20 Years On: A Unifying Framework: Part 2" - Page 23, Equation (54)
 */
class WeightFunction
{
public:
  virtual ~WeightFunction() {};
  virtual float value(const float& x) const = 0;
  virtual void configure(const float& param) {};
};
typedef std::shared_ptr<WeightFunction> WeightFunctionPtr;

class UnitWeightFunction : public WeightFunction
{
public:
  UnitWeightFunction() {};
  virtual ~UnitWeightFunction() {};
  virtual float value(const float& x) const { return 1.0f; };
};

/**
 * Tukey's hard re-descending function.
 *
 * See:
 *   http://en.wikipedia.org/wiki/Redescending_M-estimator
 */
class TukeyWeightFunction : public WeightFunction
{
public:
  TukeyWeightFunction(const float b = DEFAULT_B);
  virtual ~TukeyWeightFunction() {};
  virtual float value(const float& x) const;
  virtual void configure(const float& param);

  static const float DEFAULT_B;
private:
  float b_square;
};

class TDistributionWeightFunction : public WeightFunction
{
public:
  TDistributionWeightFunction(const float dof = DEFAULT_DOF);
  virtual ~TDistributionWeightFunction() {};
  virtual float value(const float& x) const;
  virtual void configure(const float& param);

  static const float DEFAULT_DOF;
private:
  float dof_;
  float normalizer_;
};

class HuberWeightFunction : public WeightFunction
{
public:
  HuberWeightFunction(const float k = DEFAULT_K);
  virtual ~HuberWeightFunction() {};
  virtual float value(const float& x) const;
  virtual void configure(const float& param);

  static const float DEFAULT_K;
private:
  float k;
};

} // namespace robust_cost
} // namespace vk
#endif // VIKIT_ROBUST_COST_H_
