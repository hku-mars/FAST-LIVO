/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "common/msg/cloud_msg.h"

inline std::pair<CloudPtr, CloudPtr> DrawCloudNormal(const CloudPtr cloud, const pcl::PointCloud<pcl::Normal>::Ptr normal,
                                // const std::vector<double> target_residual, int num = 5, double inter = 0.05,
                                int num = 5, double inter = 0.1,
                                int intensity = 100)
{
  // target cloud
  CloudPtr target_cloud(new PointCloudXYZI);

  // source and normal
  CloudPtr source_normal_cloud(new PointCloudXYZI);
  source_normal_cloud->reserve(cloud->size() * 6);
  int nan_cnt{ 0 }, zero_cnt{ 0 }, bad_cnt{ 0 };
  for (int j = 0; j < cloud->size(); ++j) {
    source_normal_cloud->push_back(cloud->points[j]);

    if (std::isnan(normal->points[j].normal_x) || std::isnan(normal->points[j].normal_y)|| std::isnan(normal->points[j].normal_z))
    {
      ++nan_cnt;
      continue;
    }
    if (normal->points[j].normal_x == 0 || normal->points[j].normal_y == 0 || normal->points[j].normal_z == 0)
    {
      ++zero_cnt;
      continue;
    }
    double squre = std::sqrt(normal->points[j].normal_x * normal->points[j].normal_x +
                             normal->points[j].normal_y * normal->points[j].normal_y +
                             normal->points[j].normal_z * normal->points[j].normal_z);
    if (squre < 0.99 || squre > 1.01)
    {
      normal->points[j].normal_x = normal->points[j].normal_x / squre;
      normal->points[j].normal_y = normal->points[j].normal_y / squre;
      normal->points[j].normal_z = normal->points[j].normal_z / squre;
      ++bad_cnt;
      continue;
    }

    // normal
    for (int i = 0; i < num; i++)
    {
      PointType tmp;
      tmp.x = cloud->points[j].x + i * inter * normal->points[j].normal_x;
      tmp.y = cloud->points[j].y + i * inter * normal->points[j].normal_y;
      tmp.z = cloud->points[j].z + i * inter * normal->points[j].normal_z;
      tmp.intensity = intensity + i * 10;
      source_normal_cloud->push_back(tmp);
    }
    // // target
    // PointType tmp;
    // tmp.x = cloud->points[j].x + -target_residual[j] * normal->points[j].normal_x;
    // tmp.y = cloud->points[j].y + -target_residual[j] * normal->points[j].normal_y;
    // tmp.z = cloud->points[j].z + -target_residual[j] * normal->points[j].normal_z;
    // tmp.intensity = std::fabs(target_residual[j]);
    // target_cloud->push_back(tmp);
  }

  return std::make_pair(source_normal_cloud, target_cloud);
}

inline CloudPtr DrawCloudNormal(const CloudPtr cloud, int num = 5,
                                double inter = 0.05, int intensity = 100) {
  // source and normal
  CloudPtr source_normal_cloud(new PointCloudXYZI);
  source_normal_cloud->reserve(cloud->size() * 6);
  for (int j = 0; j < cloud->size(); ++j) {
    source_normal_cloud->push_back(cloud->points[j]);

    // normal
    for (int i = 0; i < num; i++) {
      auto &pt = cloud->points[j];
      PointType tmp;
      tmp.x = pt.x + i * inter * pt.normal_x;
      tmp.y = pt.y + i * inter * pt.normal_y;
      tmp.z = pt.z + i * inter * pt.normal_z;
      tmp.intensity = intensity + i * 10;
      source_normal_cloud->push_back(tmp);
    }
  }

  return source_normal_cloud;
}

template <typename T>
inline typename pcl::PointCloud<T>::Ptr uniformSample(
    const typename pcl::PointCloud<T>::Ptr& cloud, double leaf_size = 0.5) {
  pcl::UniformSampling<T> us;

  us.setInputCloud(cloud);
  us.setRadiusSearch(leaf_size);
  typename pcl::PointCloud<T>::Ptr cloud_filtered(new
                                                  typename pcl::PointCloud<T>);
  us.filter(*cloud_filtered);

  return cloud_filtered;
}

// inline void StateToTxt(std::ofstream &ofs, const StateGroup &state, double ts,
//                        int precision = 6) {
//   auto t = state.pos_end;
//   auto q = Eigen::Quaterniond(state.rot_end);
//   ofs << std::fixed << std::setprecision(precision) << ts << " " << t.x() << " "
//       << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z()
//       << " " << q.w() << std::endl;
// }

template <int N>
struct less_vec {
  inline bool operator()(const Eigen::Matrix<int, N, 1> &v1,
                         const Eigen::Matrix<int, N, 1> &v2) const;
};

template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1> &v1,
                                    const Eigen::Matrix<int, 2, 1> &v2) const {
  return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

inline pcl::PointCloud<pcl::Normal>::Ptr getNormals(
    const PointCloudXYZI::Ptr& cloud, int num, double radius) {
  // Create the normal estimation class, and pass the input dataset to it
#ifdef MP_EN
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne;
  ne.setNumberOfThreads(MP_PROC_NUM);
#else  
  pcl::NormalEstimation<PointType, pcl::Normal> ne;
#endif

  ne.setInputCloud(cloud);

  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);

  if (num > 0) {
    // method 1 , Knn
    ne.setKSearch(num); // K
  } else {
    // method 2 , Use all neighbors in a sphere of radius 30cm
    ne.setRadiusSearch(radius);
  }

  // Compute the features
  ne.compute(*cloud_normals);

  std::cout << "cloud size and cloud_normals size: " << cloud->points.size()
            << " " << cloud_normals->points.size() << std::endl;

  return cloud_normals;
}

inline double CalcDiffNormalAngle(const PointVector &points_near,
                                  std::vector<float> sq_dis_near) {
  double diff_normal_angle{0};
  const PointType &nearest_point = points_near[0];
  const Eigen::Vector3d nearest_normal(nearest_point.normal_x, nearest_point.normal_y, nearest_point.normal_z);
  if (abs(nearest_point.normal_x)+abs(nearest_point.normal_y)+abs(nearest_point.normal_z)<0.1)// std::isnan(nearest_point.normal_x))
    return 999;
  int valid_near_num{0};
  for (int j = 1; j < points_near.size(); ++j) {
    if ( abs(points_near[j].normal_x)+abs(points_near[j].normal_y)+abs(points_near[j].normal_z)<0.1 )
      continue;
    Eigen::Vector3d near_normal{points_near[j].normal_x, points_near[j].normal_y, points_near[j].normal_z};
    double angle = acos(near_normal.dot(nearest_normal));
    angle = angle > M_PI ? 2 * M_PI - angle : angle;
    diff_normal_angle += angle;
    ++valid_near_num;
  }
  if(valid_near_num == 0)
    return 999;

  return diff_normal_angle/valid_near_num;
}

inline std::pair<Eigen::Vector3f, Eigen::Matrix3f> PCLPCA(const PointCloudXYZI::Ptr& cloud) {
  pcl::PCA<PointType> pca;

  // 设置输入点云
  pca.setInputCloud(cloud);

  // 获取主成分
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

  return std::make_pair(eigenvalues, eigenvectors);
}
