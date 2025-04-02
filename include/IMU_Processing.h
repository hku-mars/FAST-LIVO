
#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H
#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (200)

const bool time_list(PointType &x, PointType &y); //{return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
struct ImuProcessResult {
  ImuProcessResult(const std::string _log_dir) {
    log_dir = _log_dir;
  }
  double update_ts;

  int init_process;   /* process, (%) */
  Pose prediction_pose; /* Imu pose in W */
  std::string log_dir;
  void Print() {
    LDEBUG << "==============ImuProcessResult==============" << REND;
    LDEBUG << "init_process: " << init_process << REND;
  }
};

class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess(std::string dir);
  ~ImuProcess();

  std::shared_ptr<ImuProcessResult> result_;

  void Reset();
  void Reset(double start_timestamp, const slam::Imu::ConstPtr &lastimu);
  void push_update_state(double offs_t, StatesGroup state);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
#ifdef ADAPTIVE_INIT
  void set_acc_cov(const V3D &_cov_acc);
  void set_gyr_cov(const V3D &_cov_gyr);
#endif
  void Process(const LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
  void PredicteStateAndUndistortCloud(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
  void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  void Prediction(const LidarMeasureGroup &lidar_meas, StatesGroup &state_inout);
  void UndistortPcl(const LidarMeasureGroup &lidar_meas, PointCloudXYZI &pcl_out);

  ofstream fout, f_I_state_L_utm, f_I_state_utm;

  V3D cov_acc, cov_gyr, cov_bias_gyr, cov_bias_acc;
  bool use_cfg_cov = false;
  double cfg_cov_acc, cfg_cov_gyr, cfg_cov_bias_gyr, cfg_cov_bias_acc;
  double init_cov_rot, init_cov_pos, init_cov_vel, init_cov_bias_gyr, init_cov_bias_acc;

  double first_lidar_time;
  std::string log_dir;
  float init_process = 0.;
  bool imu_need_init_ = true;
  bool img_en = true;
#ifdef ADAPTIVE_INIT
  bool init_data_ready_{false};
  double init_time_{0.};
  Eigen::Matrix3d init_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d init_v_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_g_ = Eigen::Vector3d::Zero();
  StatesGroup init_state_;
  V3D cov_acc_init_;
  V3D cov_gyr_init_;
#endif
 private:
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);

  PointCloudXYZI::Ptr cur_pcl_un_;
  slam::Imu::ConstPtr last_imu_;
  deque<slam::Imu::ConstPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lid_rot_to_IMU;
  V3D Lid_offset_to_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  V3D last_acc;
  V3D last_ang;
  double last_imu_t; // last imu timestamp
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  M3D Eye3d;
  V3D Zero3d;

  // 替换static变量
  double Prediction_last_tgt_time_ = -1.0;
  double addRelTf_prev_time_ = -1.0; // init to -1 to determine if the first rel_tf is added

// rs
 public:
  int MAX_QUEUE_SIZE = 1000;
  std::map<double, Pose> rel_tf_queue_;
  std::mutex rel_tf_queue_mutex_;
  void addRelTf(const Pose& rel_tf) {
    std::lock_guard<std::mutex> lk(rel_tf_queue_mutex_);
    rel_tf_queue_.insert(std::make_pair(rel_tf.timestamp, rel_tf));
    // std::cout << std::fixed << rel_tf.timestamp << std::endl;

    /* remove early elements when queue size */
    auto it = rel_tf_queue_.begin();
    while (it != rel_tf_queue_.end() && rel_tf_queue_.size() > MAX_QUEUE_SIZE) {
      it = rel_tf_queue_.erase(it);
    }

    // static double prev_time = rel_tf.timestamp;
    if(addRelTf_prev_time_ < 0) {
      addRelTf_prev_time_ = rel_tf.timestamp;
    }

    if (rel_tf.timestamp - addRelTf_prev_time_ > 0.0099) {
      std::cout << "WARNING: rel tf missing msg" << std::endl;
    }
    addRelTf_prev_time_ = rel_tf.timestamp;
    // if (int(prev_time) % 5 == 0)
    //   std::cout << "rel que size: " << rel_tf_queue_.size()
    //             << " front t:" << rel_tf_queue_.begin()->first
    //             << " back t:" << rel_tf_queue_.rbegin()->first << std::endl;
  }
  bool interpolate(std::map<double, Pose>& queue,
                                        double t, Pose& pose_at_t) {
    if (queue.size() < 2) {
      std::cout << "WARN: Interpolate queue size less than 2" << std::endl;
      return false;
    }

    const double TIME_DIFF_THRESH = 1e-3;
    auto rel_next = queue.lower_bound(t);
    if (rel_next == queue.end()) /* t is newer than latest element in queue */
    {
      LERROR << "interpolate err, t > queue.back().timestamp" << REND;
      if (std::fabs(t - queue.rbegin()->first) < TIME_DIFF_THRESH) {
        LERROR << std::fabs(t - queue.rbegin()->first)
               << " is tolerance time diff, use last pose" << REND;
        pose_at_t = queue.rbegin()->second;
      } else /* interpolate outside right bound */
      {
        LERROR<<"outside"<<REND;
        rel_next = std::prev(rel_next);
        auto rel_prev = std::prev(rel_next);

        Velocity vel;
        getVelFromPose(rel_prev->second, rel_next->second,
                       vel); // vel in global frame
        Velocity local_vel;
        local_vel.linear = rel_next->second.q.inverse() * vel.linear;
        local_vel.angular = rel_next->second.q.inverse() * vel.angular;
        Eigen::Matrix4d T_at_t = rel_next->second.transform() *
                                deltaTransform(local_vel, t - rel_next->first);
        pose_at_t = Pose(T_at_t, t);
      }
      LWARNING << "pose t:" << t << " next t:" << rel_next->first << REND;
    } else if (rel_next ==
              queue.begin()) /* t is earlier than the first element in queue */
    {
      LERROR<< "interpolate err, t < queue.front().timestamp"<<REND;
    } else if (std::fabs(t - rel_next->first) < TIME_DIFF_THRESH) {
      pose_at_t = rel_next->second;
      return true;
    } else /* interpolate */
    {
      // LINFO << "pose t:" << t << " next t:" << rel_next->first << REND;
      auto rel_prev = std::prev(rel_next);
      double ratio = (t - rel_prev->first) / (rel_next->first - rel_prev->first);
      pose_at_t.xyz = (1 - ratio) * rel_prev->second.xyz + ratio * rel_next->second.xyz;
      pose_at_t.q = rel_prev->second.q.slerp(ratio, rel_next->second.q);
    }

    pose_at_t.source = rel_next->second.source;
    pose_at_t.timestamp = t;
    return true;
  }
  bool pointsUndistort(const std::vector<double>& cloud_ts,
                       const PointCloudXYZI::Ptr &cloud_ptr,
                       const PointCloudXYZI::Ptr &outcloud,
                       std::vector<int> &instance, int &point_filter_num,
                       const Pose &pre_odom, const Pose &cur_odom,
                       double undistort_time, Eigen::Affine3d &dst_pose,
                       bool com_to_end = true) {
    if (pre_odom.timestamp > cur_odom.timestamp)
    {
      std::cout << " undistort return " << std::endl;
      return false;
    }
    // Eigen::Affine3d dst_pose;
    if (com_to_end)
    {
      dst_pose.translation() = cur_odom.xyz;
      dst_pose.linear() = cur_odom.q.toRotationMatrix();
    }
    else
    {
      dst_pose = poseInterp(undistort_time, pre_odom, cur_odom);
    }
    Eigen::Matrix4d dst_pose_inv = dst_pose.matrix().inverse();
    int pt_count = 0;

    double last_time = 0.0;
    Eigen::Affine3d pt_pose;
    Eigen::Matrix4d t_mat;

    for (size_t i = 0; i < cloud_ptr->size(); ++i) {
      if (instance[i] != -1) continue;
      pt_count++;
      PointType added_pt;
      if (pt_count % point_filter_num == 0) {
        // double timestamp = cloud_ptr->points[i].intensity; // NOTE: intensity represent timestamp
        double timestamp = cloud_ts[i];
        if (timestamp - last_time > 2e-5) {
          pt_pose = poseInterp(timestamp, pre_odom, cur_odom);
          t_mat = dst_pose_inv * pt_pose.matrix();
          last_time = timestamp;
        }
        if (timestamp > cur_odom.timestamp) {
          continue;
        }

        Eigen::Affine3d transformation;
        transformation.matrix() = t_mat;
        Eigen::Vector3d pt1(cloud_ptr->points[i].x, cloud_ptr->points[i].y,
                            cloud_ptr->points[i].z);
        pcl::transformPoint(pt1, pt1, transformation);
        added_pt.x = pt1.x();
        added_pt.y = pt1.y();
        added_pt.z = pt1.z();
        added_pt.intensity = cloud_ptr->points[i].intensity;
        outcloud->points.emplace_back(added_pt);
      }
    }
    std::cout << "undistort, size in: "<<cloud_ptr->size()<<", out: " << outcloud->size() << std::endl;
    return true;
  }

  Eigen::Affine3d poseInterp(double t, Pose const& aff1, Pose const& aff2)
  {
    /* assume here t1 <= t <= t2 */ 
    double t1 = aff1.timestamp;
    double t2 = aff2.timestamp;
    double alpha = 0.0;
    if (t2 != t1)
    {
      alpha = (t - t1) / (t2 - t1);
    }

    Eigen::Quaternion<double> rot1, rot2;

    rot1.w() = aff1.q.w();
    rot1.x() = aff1.q.x();
    rot1.y() = aff1.q.y();
    rot1.z() = aff1.q.z();

    rot2.w() = aff2.q.w();
    rot2.x() = aff2.q.x();
    rot2.y() = aff2.q.y();
    rot2.z() = aff2.q.z();

    Eigen::Vector3d trans1, trans2;

    trans1.x() = aff1.xyz.x();
    trans1.y() = aff1.xyz.y();
    trans1.z() = aff1.xyz.z();

    trans2.x() = aff2.xyz.x();
    trans2.y() = aff2.xyz.y();
    trans2.z() = aff2.xyz.z();

    Eigen::Affine3d result;
    result.translation() = (1.0 - alpha) * trans1 + alpha * trans2;
    result.linear() = rot1.slerp(alpha, rot2).toRotationMatrix();

    return result;
  }
  // intensity represent timestamp
  void undistortPointCloud(
      std::vector<double> cloud_ts, const PointCloudXYZI::Ptr lidar_cloud,
      PointCloudXYZI::Ptr& undistorted_cloud, std::vector<int>& instance,
      int point_filter_num=1) {
    /*get rel_tf pose at lidar_cloud->header.stamp*/
    Pose rel_tf;
    rel_tf_queue_mutex_.lock();
    if (rel_tf_queue_.size() < 2) {
      undistorted_cloud = lidar_cloud;
      LERROR << "undistortPointCloud fail rel_tf_queue_<2" << REND;
      rel_tf_queue_mutex_.unlock();
      return;
    }

    /* 1. get rel_tf at lidar_cloud->header.stamp */
    Pose rel_tf_begin;
    Pose rel_tf_end;
    cout << std::fixed << "undistortPointCloud from: " << cloud_ts[0]
           << " to: " << cloud_ts.back() << REND;
    bool ret1 = interpolate(rel_tf_queue_, cloud_ts[0],
                            rel_tf_begin);
    bool ret2 = interpolate(rel_tf_queue_, cloud_ts.back(),
                            rel_tf_end);
    rel_tf_queue_mutex_.unlock();

    Eigen::Affine3d dst = Eigen::Affine3d::Identity();
    if (ret1 && ret2) {
      pointsUndistort(cloud_ts, lidar_cloud, undistorted_cloud, instance, point_filter_num,
                      rel_tf_begin, rel_tf_end, 0, dst, true);
      undistorted_cloud->header = lidar_cloud->header;
    } else {
      undistorted_cloud = lidar_cloud;
      LERROR << "undistortPointCloud fail" << REND;
    }

    return;
  }
  bool getVelFromPose(const Pose &pose_prev, const Pose &pose_next,
                      Velocity &vel) {
    double delta_t = pose_next.timestamp - pose_prev.timestamp;
    vel.linear = (pose_next.xyz - pose_prev.xyz) / delta_t;

    Eigen::Matrix3d R_prev = pose_prev.q.toRotationMatrix();
    Eigen::Matrix3d R_next = pose_next.q.toRotationMatrix();
    Eigen::Matrix3d R_delta = R_prev.transpose() * R_next;
    Eigen::AngleAxisd rot = Eigen::AngleAxisd().fromRotationMatrix(R_delta);
    Eigen::Vector3d rot_vec = rot.angle() * rot.axis();
    vel.angular = rot_vec / delta_t;
    return true;
  }

   bool getVelFromPoseQueue(
      const std::map<double, Pose>& queue, double t, Velocity& vel) {
    if (queue.size() < 2) {
      std::cout << "WARN: cannot get vel from queue that is smaller than 2"
                << std::endl;
      return false;
    }

    auto rel_next = queue.lower_bound(t);
    decltype(rel_next) rel_prev;

    if (rel_next == queue.end()) /* t is newer than latest element in queue */
    {
      rel_next = std::prev(rel_next);
      rel_prev = std::prev(rel_next);
    } else if (rel_next ==
              queue.begin()) /* t is older than the earliest element in queue */
    {
      rel_prev = rel_next;
      rel_next = std::next(rel_prev);
    } else {
      rel_prev = std::prev(rel_next);
    }

    return getVelFromPose(rel_prev->second, rel_next->second, vel);
  }

  Eigen::Matrix4d deltaTransform(const Velocity& local_vel,
                                                      double delta_t) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Vector3d rot_vec = local_vel.angular * delta_t;
    Eigen::AngleAxisd delta_R(rot_vec.norm(), rot_vec.normalized());
    Eigen::Vector3d v2 = delta_R * local_vel.linear;
    T.block<3, 1>(0, 3) = (local_vel.linear + v2) * 0.5 * delta_t;
    T.block<3, 3>(0, 0) = delta_R.toRotationMatrix();
    return T;
  }

};
#endif
