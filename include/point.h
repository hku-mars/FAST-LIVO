// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SVO_POINT_H_
#define SVO_POINT_H_

#include <boost/noncopyable.hpp>
#include <common_lib.h>
#include <feature.h>
#include <frame.h>

namespace lidar_selection {

typedef Matrix<double, 2, 3> Matrix23d;

/// A 3D point on the surface of the scene.
class Point : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // enum PointType {
  //   TYPE_DELETED,
  //   TYPE_CANDIDATE,
  //   TYPE_UNKNOWN,
  //   TYPE_GOOD
  // };

  static int                  point_counter_;           //!< Counts the number of created points. Used to set the unique id.
  int                         id_;                      //!< Unique ID of the point.
  Vector3d                    pos_;                     //!< 3d pos of the point in the world coordinate frame.
  float x, y, z, intensity;
  Vector3d                    normal_;                  //!< Surface normal at point.
  Matrix3d                    normal_information_;      //!< Inverse covariance matrix of normal estimation.
  bool                        normal_set_;              //!< Flag whether the surface normal was estimated or not.
  list<FeaturePtr>              obs_;                     //!< References to keyframes which observe the point.
  size_t                      n_obs_;                   //!< Number of observations: Keyframes AND successful reprojections in intermediate frames.
  int                         last_published_ts_;       //!< Timestamp of last publishing.
  int                         last_projected_kf_id_;    //!< Flag for the reprojection: don't reproject a pt twice.
  // PointType                   type_;                    //!< Quality of the point.
  int                         n_failed_reproj_;         //!< Number of failed reprojections. Used to assess the quality of the point.
  int                         n_succeeded_reproj_;      //!< Number of succeeded reprojections. Used to assess the quality of the point.
  int                         last_structure_optim_;    //!< Timestamp of last point optimization
  bool                        have_scaled;
  float                       value;
  Point(const Vector3d& pos);
  Point(const Vector3d& pos, FeaturePtr ftr);
  ~Point();

  void getFurthestViewObs(const Vector3d& framepos, FeaturePtr& ftr) const;
  void deleteFeatureRef(FeaturePtr ftr);

  /// Add a reference to a frame.
  void addFrameRef(FeaturePtr ftr);

  /// Remove reference to a frame.
  bool deleteFrameRef(Frame* frame);

  /// Initialize point normal. The inital estimate will point towards the frame.
  void initNormal();

  /// Check whether mappoint has reference to a frame.
  FeaturePtr findFrameRef(Frame* frame);

  bool getClosePose(const FramePtr& new_frame, FeaturePtr& ftr) const;

  /// Get Frame with similar viewpoint.
  bool getCloseViewObs(const Vector3d& pos, FeaturePtr& obs, const Vector2d& cur_px) const;

  bool getCloseViewObs_test(const Vector3d& pos, FeaturePtr& obs, const Vector2d& cur_px, double& min_cos_angle) const;

  /// Get number of observations.
  inline size_t nRefs() const { return obs_.size(); }

  /// Optimize point position through minimizing the reprojection error.
  void optimize(const size_t n_iter);

  /// Jacobian of point projection on unit plane (focal length = 1) in frame (f).
  inline static void jacobian_xyz2uv(
      const Vector3d& p_in_f,
      const Matrix3d& R_f_w,
      Matrix23d& point_jac)
  {
    const double z_inv = 1.0/p_in_f[2];
    const double z_inv_sq = z_inv*z_inv;
    point_jac(0, 0) = z_inv;
    point_jac(0, 1) = 0.0;
    point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
    point_jac(1, 0) = 0.0;
    point_jac(1, 1) = z_inv;
    point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
    point_jac = - point_jac * R_f_w;
  }
};

} // namespace lidar_selection

#endif // SVO_POINT_H_
