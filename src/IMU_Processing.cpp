#include <omp.h>
#include"IMU_Processing.h"

const bool time_list(PointType &x, PointType &y)
{
  return (x.curvature < y.curvature);
}

ImuProcess::ImuProcess(std::string dir)
    : Eye3d(M3D::Identity()), Zero3d(0, 0, 0), b_first_frame_(true),
      imu_need_init_(true), start_timestamp_(-1), log_dir(dir) {
  init_iter_num = 1;
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.1, 0.1, 0.1);
  cov_bias_acc  = V3D(0.1, 0.1, 0.1);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  Lid_offset_to_IMU = Zero3d;
  Lid_rot_to_IMU    = Eye3d;
  last_imu_.reset(new slam::Imu());

  fout.open(log_dir+"/imu_prediction.txt", ios::out);
  f_I_state_L_utm.open(log_dir+"/utm_I_pose_L.txt",ios::out);
  f_I_state_utm.open(log_dir+"/utm_I_pose_W.txt",ios::out);

  result_.reset(new ImuProcessResult(dir));
}

ImuProcess::~ImuProcess() {
  fout.close();
  f_I_state_L_utm.close();
  f_I_state_utm.close();
}

void ImuProcess::Reset() 
{
  LERROR<<"Reset ImuProcess"<<REND;
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new slam::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::push_update_state(double offs_t, StatesGroup state)
{
  // V3D acc_tmp(last_acc), angvel_tmp(last_ang), vel_imu(state.vel_end), pos_imu(state.pos_end);
  // M3D R_imu(state.rot_end);
  // angvel_tmp -= state.bias_g;
  // acc_tmp   = acc_tmp * G_m_s2 / mean_acc.norm() - state.bias_a;
  // acc_tmp  = R_imu * acc_tmp + state.gravity;
  // IMUpose.push_back(set_pose6d(offs_t, acc_tmp, angvel_tmp, vel_imu, pos_imu, R_imu));
  V3D acc_tmp=acc_s_last, angvel_tmp=angvel_last, vel_imu(state.vel_end), pos_imu(state.pos_end);
  M3D R_imu(state.rot_end);
  IMUpose.push_back(set_pose6d(offs_t, acc_tmp, angvel_tmp, vel_imu, pos_imu, R_imu));
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU    = rot;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}
#ifdef ADAPTIVE_INIT
void ImuProcess::set_acc_cov(const V3D &_cov_acc)
{
  cov_acc_init_ = _cov_acc;
}

void ImuProcess::set_gyr_cov(const V3D &_cov_gyr)
{
  cov_gyr_init_ = _cov_gyr;
}
#endif
void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  result_->init_process = init_process = double(N) / MAX_INI_COUNT * 100;
  printf("IMU Initializing: %.1f %%\n", init_process);
  fout << "IMU Initializing:" << init_process << "\n";
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    // first_lidar_time = meas.lidar_beg_time;
    // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }

  state_inout.gravity = - mean_acc / mean_acc.norm() * G_m_s2;
  
  state_inout.rot_end = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  state_inout.bias_g  = mean_gyr;

  state_inout.cov.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.00001; //1e-12;
  // state_inout.cov.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-12;

  last_imu_ = meas.imu.back();
}

void ImuProcess::Prediction(const LidarMeasureGroup &lidar_meas, StatesGroup &state_inout)
{
  // static double last_tgt_time{-1.};
  MeasureGroup meas;
  meas = lidar_meas.measures.back();
  double tgt_time = 0.;
  if(lidar_meas.is_lidar_end)
  {
    tgt_time = lidar_meas.lidar_beg_time + lidar_meas.lidar->points.back().curvature / double(1000);
  }
  else
  {
    tgt_time = lidar_meas.lidar_beg_time + meas.img_offset_time;
  }
  fout << lidar_meas.is_lidar_end << " last_tgt_time: " << std::to_string(Prediction_last_tgt_time_) << " " << "tgt_time: " << std::to_string(tgt_time) << std::endl;
  // cout<<"meas.imu.size: "<<meas.imu.size()<<endl;
  auto v_imu = meas.imu;
  if(v_imu.size() == 0) return;
  V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  M3D R_imu(state_inout.rot_end);
  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  if(Prediction_last_tgt_time_>0.)
  {
    v_imu.push_front(last_imu_);
  }
  int cnt{0};
  v_imu.push_back(meas.imu_next);
  fout << lidar_meas.is_lidar_end << " last_imu_: " << std::to_string(last_imu_->header.ToSec()) << " " << "meas.imu_next: " << std::to_string(
      meas.imu_next->header.ToSec()) << std::endl;
  double dt = 0;
  for (auto it_imu = v_imu.begin(); it_imu != v_imu.end()-1 ; it_imu++)
  {
    auto &&head = *(it_imu);

    auto &&tail = *(it_imu + 1);
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    auto tmp = angvel_avr;
    angvel_avr -= state_inout.bias_g;
    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

    double head_ts = head->header.ToSec();
    double tail_ts = tail->header.ToSec();
    if (v_imu.begin() == it_imu && Prediction_last_tgt_time_ > 0.) {
      dt = tail_ts - Prediction_last_tgt_time_;
    } else if (v_imu.end() - 2 == it_imu) {
      dt = tgt_time - head_ts;
    } else {
      dt = tail_ts - head_ts;
    }

    /* covariance propagation */
    M3D acc_avr_skew;
    M3D Exp_f   = Exp(angvel_avr, dt);
    acc_avr_skew<<SKEW_SYM_MATRX(acc_avr);

    fout << std::to_string(head_ts) << " " << std::to_string(tail_ts) << " "
         << dt << " " 
         << angvel_avr.x() << " " << angvel_avr.y() << " " << angvel_avr.z() << " "
         << tmp.x() << " " << tmp.y() << " " << tmp.z() << " "
         << state_inout.bias_g.x() << " " << state_inout.bias_g.y() << " " << state_inout.bias_g.z() << " "
         << std::endl;

    F_x.setIdentity();
    cov_w.setZero();

    F_x.block<3,3>(0,0)  = Exp(angvel_avr, - dt);
    F_x.block<3,3>(0,9)  = - Eye3d * dt;
    // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
    F_x.block<3,3>(3,6)  = Eye3d * dt;
    F_x.block<3,3>(6,0)  = - R_imu * acc_avr_skew * dt;
    F_x.block<3,3>(6,12) = - R_imu * dt;
    F_x.block<3,3>(6,15) = Eye3d * dt;

    cov_w.block<3,3>(0,0).diagonal()   = cov_gyr * dt * dt;
    cov_w.block<3,3>(6,6)              = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
    cov_w.block<3,3>(9,9).diagonal()   = cov_bias_gyr * dt * dt; // bias gyro covariance
    cov_w.block<3,3>(12,12).diagonal() = cov_bias_acc * dt * dt; // bias acc covariance

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    /* propogation of IMU attitude */
    R_imu = R_imu * Exp_f;

    /* Specific acceleration (global frame) of IMU */
    acc_imu = R_imu * acc_avr + state_inout.gravity;

    /* propogation of IMU */
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    /* velocity of IMU */
    vel_imu = vel_imu + acc_imu * dt;

    last_imu_t = head_ts;

    Pose T_W_I_pose(R_imu, pos_imu, tail_ts);
    f_I_state_utm << std::fixed<< std::setprecision(6)<< T_W_I_pose.timestamp << " " << T_W_I_pose.xyz.x()
                << " " << T_W_I_pose.xyz.y() << " " << T_W_I_pose.xyz.z() << " " << T_W_I_pose.q.x() << " "
                << T_W_I_pose.q.y() << " " << T_W_I_pose.q.z() << " "
                << T_W_I_pose.q.w() << endl;

    Pose T_I_L(Lid_rot_to_IMU, Lid_offset_to_IMU, 0);
    T_W_I_pose.updatePoseRight(T_I_L);
    Pose& T_W_L = T_W_I_pose;
    addRelTf(T_W_L);
    f_I_state_L_utm << std::fixed<< std::setprecision(6)<< T_W_L.timestamp << " " << T_W_L.xyz.x()
                << " " << T_W_L.xyz.y() << " " << T_W_L.xyz.z() << " " << T_W_L.q.x() << " "
                << T_W_L.q.y() << " " << T_W_L.q.z() << " "
                << T_W_L.q.w() << endl;
  }
  state_inout.vel_end = vel_imu;
  state_inout.rot_end = R_imu;
  state_inout.pos_end = pos_imu;
  last_imu_ = meas.imu.back();
  Prediction_last_tgt_time_ = tgt_time;

  state_inout.bias_g_test = state_inout.bias_g;
  return;
}

void ImuProcess::UndistortPcl(const LidarMeasureGroup &lidar_meas, PointCloudXYZI &pcl_out)
{
  PointCloudXYZI::Ptr undis_cloud(new PointCloudXYZI);
  PointCloudXYZI::Ptr in_cloud = lidar_meas.lidar;
  std::vector<double> cloud_ts(in_cloud->size(), 0.0);
  for (int i = 0; i < in_cloud->size(); i++) {
    cloud_ts[i] = lidar_meas.lidar_beg_time +
                  in_cloud->points[i].curvature / double(1000.0);
  }
  std::vector<int> instance(in_cloud->size(), -1);
  // double t1 = omp_get_wtime();
  undistortPointCloud(cloud_ts, in_cloud, undis_cloud, instance);
  // double t2 = omp_get_wtime();
  // LINFO << "rs undistort time: " << t2 - t1 << REND;
  pcl_out = *undis_cloud;
}

void ImuProcess::PredicteStateAndUndistortCloud(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();
  if (lidar_meas.lidar == nullptr) {
    throw std::runtime_error("lidar_meas.lidar is nullptr!");
  }
//  ROS_ASSERT(lidar_meas.lidar != nullptr);
  MeasureGroup meas = lidar_meas.measures.back();
#ifdef ADAPTIVE_INIT
  if(img_en)
  {
    result_->init_process = init_process = imu_need_init_ ? 0 : 100;
  }
#endif
  
  if (imu_need_init_)
  {
#ifdef ADAPTIVE_INIT
    if(img_en)
    {
      if(meas.imu.empty()) {return;}
      if(!init_data_ready_) {return;}
      if(lidar_meas.is_lidar_end)
      {
        return;
      }
      else
      {
        double tgt_time = lidar_meas.lidar_beg_time + meas.img_offset_time;
        if(std::abs(tgt_time-init_time_)<1e-2)
        {
          imu_need_init_ = false;
          init_state_.rot_end = init_R_;
          init_state_.bias_g  = init_bg_;
          init_state_.vel_end = init_v_;
          init_state_.gravity = init_g_;
          init_state_.cov.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.00001;
          mean_acc = - Eigen::Vector3d(0., 0., 1.) * G_m_s2;
          cov_acc = cov_acc_init_;
          cov_gyr = cov_gyr_init_;
          stat = init_state_;
          last_imu_   = meas.imu.back();

          if (use_cfg_cov) {
            cov_acc << cfg_cov_acc, cfg_cov_acc, cfg_cov_acc;
            cov_gyr << cfg_cov_gyr, cfg_cov_gyr, cfg_cov_gyr;
            cov_bias_gyr << cfg_cov_bias_gyr, cfg_cov_bias_gyr,
                cfg_cov_bias_gyr;
            cov_bias_acc << cfg_cov_bias_acc, cfg_cov_bias_acc,
                cfg_cov_bias_acc;
            stat.cov.block<3, 3>(0, 0) = M3D::Identity() * init_cov_rot;
            stat.cov.block<3, 3>(3, 3) = M3D::Identity() * init_cov_pos;
            stat.cov.block<3, 3>(6, 6) = M3D::Identity() * init_cov_vel;
            stat.cov.block<3, 3>(9, 9) = M3D::Identity() * init_cov_bias_gyr;
            stat.cov.block<3, 3>(12, 12) = M3D::Identity() * init_cov_bias_acc;
          }
        }
        return;
      }
    }
#endif
    if(meas.imu.empty()) {return;};
    /// The very first lidar frame
    IMU_init(meas, stat, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    if (lidar_meas.is_lidar_end && init_iter_num > MAX_INI_COUNT) // NOTE: 避免cam频率不同导致的用于零偏计算的imu数量异常
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;
      printf("IMU Initials: Gravity: %.4f %.4f %.4f %.4f; acc covarience: %.8f "
             "%.8f %.8f; gry covarience: %.8f %.8f %.8f",
             stat.gravity[0], stat.gravity[1], stat.gravity[2], mean_acc.norm(),
             cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1],
             cov_gyr[2]);
      fout << "g: " << stat.gravity[0] << " " << stat.gravity[1] << " "
           << stat.gravity[2] << " cov_a: " << cov_acc[0] << " " << cov_acc[1]
           << " " << cov_acc[2] << " cov_g:" << cov_gyr[0] << " " << cov_gyr[1]
           << " " << cov_gyr[2] << "\n";
    }
    return;
  }
#if 0
  UndistortPcl(lidar_meas, stat, *cur_pcl_un_);
#else
  Prediction(lidar_meas, stat);
  cur_pcl_un_->clear();
  if(lidar_meas.is_lidar_end)
  {
    UndistortPcl(lidar_meas, *cur_pcl_un_);
  }
#endif
}
