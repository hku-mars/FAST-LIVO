#include <iostream>
#include "livo_initial/initializer.h"
#include "livo_initial/global_sfm.h"

namespace livo_initial{

Initializer::Initializer(const Eigen::Matrix4d _T_imu_from_lidar, const Eigen::Matrix4d _T_camera_from_lidar, const double _g_norm)
: g_norm_(_g_norm)
{
  R_imu_from_lidar_ = _T_imu_from_lidar.topLeftCorner(3,3);
  t_imu_from_lidar_ = _T_imu_from_lidar.topRightCorner(3,1);
  R_imu_from_camera_ = R_imu_from_lidar_*_T_camera_from_lidar.topLeftCorner(3,3).transpose();
  t_imu_from_camera_ = R_imu_from_lidar_*(-_T_camera_from_lidar.topLeftCorner(3,3).transpose()*_T_camera_from_lidar.topRightCorner(3,1)) + t_imu_from_lidar_;
  t_imu_from_camera_ << -0.030219, 0.02354, 0.;

  std::cout << "R_imu_from_lidar_:\n" << R_imu_from_lidar_ << std::endl;
  std::cout << "t_imu_from_lidar_:\n" << t_imu_from_lidar_ << std::endl;
  std::cout << "R_imu_from_camera_:\n" << R_imu_from_camera_ << std::endl;
  std::cout << "t_imu_from_camera_:\n" << t_imu_from_camera_ << std::endl;

  std::fill(keyframe_times_.begin(), keyframe_times_.end(), 0.);
  std::fill(keyframe_bas_.begin(), keyframe_bas_.end(), Eigen::Vector3d::Zero());
  std::fill(keyframe_bgs_.begin(), keyframe_bgs_.end(), Eigen::Vector3d::Zero());

  process_active_ = true;
  const auto& func = [this] { process(); };
  process_thread_ = std::thread(func);

  visual_active_ = true;
  const auto& func2 = [this] { visualTrack(); };
  visual_thread_ = std::thread(func2);
  
}

// 析构函数
Initializer::~Initializer()
{
  // 设置process_active_为false
  process_active_ = false;
  // 通知con_process_条件变量
  con_process_.notify_one();
  if (process_thread_.joinable()) {
    process_thread_.join();
  }

  // 设置visual_active_为false
  visual_active_ = false;
  // 通知con_visual_条件变量
  con_visual_.notify_one();
  if (visual_thread_.joinable()) {
    visual_thread_.join();
  }
}

void Initializer::setCameraParam(const int _width, const int _height, const double _fx, const double _fy, const double _cx, const double _cy, std::vector<double> &_distortion_coeffs)
{
  fx_inv_ = 1./_fx;
  fy_inv_ = 1./_fy;
  std::vector<double> intrinsics{_fx, _fy, _cx, _cy};
  feature_tracker_.setCameraParam(_width, _height, intrinsics, _distortion_coeffs, PINHOLE);
}

void Initializer::addLidar(double _time, PointCloudXYZI::Ptr _lidar_ptr)
{
  if(!process_active_)
  {
    return;
  }
  mutex_data_.lock();
  if(addLidar_time_last_ < _time)
  {
    PointCloudXYZI::Ptr lidar_clone(new PointCloudXYZI(*_lidar_ptr));
    lidar_buf_.emplace(_time, lidar_clone);
    addLidar_time_last_ = _time;
  }
  mutex_data_.unlock();
  con_process_.notify_one();
}

void Initializer::addImage(double _time, cv::Mat _img)
{
  if(!process_active_)
  {
    return;
  }
  mutex_visual_.lock();
  if(addImage_time_last_ < _time)
  {
    img_buf_.emplace(_time, _img.clone());
    addImage_time_last_ = _time;
  }
  mutex_visual_.unlock();
  con_visual_.notify_one();
}

void Initializer::addImageInfo(ImageInfo &_img_info)
{
  if(!process_active_)
  {
    return;
  }
  mutex_data_.lock();
  if(addImageInfo_time_last_ < _img_info.timestamp)
  {
    img_info_buf_.emplace(_img_info);
    addImageInfo_time_last_ = _img_info.timestamp;
  }
  mutex_data_.unlock();
  con_process_.notify_one();
}

void Initializer::addImu(double _time, Eigen::Vector3d _fb, Eigen::Vector3d _wib_b)
{
  if(!process_active_)
  {
    return;
  }
  mutex_data_.lock();
  if(addImu_time_last_ < _time)
  {
    imu_buf_.emplace(std::move(ImuData(_time, _fb, _wib_b)));
    addImu_time_last_ = _time;
  }
  mutex_data_.unlock();
  con_process_.notify_one();
}

void Initializer::process()
{
  double align_time_last = 0.;
  while(process_active_)
  {
    std::unique_lock<std::mutex> lk_data(mutex_data_);
    con_process_.wait(lk_data);
    lk_data.unlock();
    if(!process_active_) break;
    std::vector<Frame> frames_new = dataSync();
    if(frames_new.empty())
    {
      continue;
    }
    for(auto& frame : frames_new)
    {
      frame.preIntegration(keyframe_bas_[frame_count_], keyframe_bgs_[frame_count_]);
      if(!all_image_frame_.empty())
      {
        bool is_stop = checkStop(all_image_frame_.rbegin()->second, frame);
        if(is_stop)
        {
          all_image_frame_.rbegin()->second.is_stop = true;
          frame.is_stop = true;
        }
        else
        {
          frame.is_stop = false;
        }
      }
      all_image_frame_.emplace(frame.timestamp, frame);
      // continue;
      if (addFeatureCheckParallax(frame_count_, frame.featureId_xyuv))
        marginalization_flag_ = MARGIN_OLD;
      else
        marginalization_flag_ = MARGIN_SECOND_NEW;
      keyframe_times_[frame_count_] = frame.timestamp;
      if(WINDOW_SIZE == frame_count_)
      {
        bool result = staticInitialize();
        if(result)
        {
          process_active_ = false;
          time_finished_last_ = frame.timestamp;
          std::cout << "Static initialization finished!" << std::endl;
          break;
        }
        if((frame.timestamp - align_time_last) > 0.1)
        {
          result = initialStructure();
          align_time_last = frame.timestamp;
        }
        // std::cout << "result: " << result << ", g_world_: " << g_world_.transpose() << std::endl;
        // std::cout << "beg: " << std::to_string(all_image_frame_.begin()->first) << all_image_frame_.begin()->second.pre_integration->init_bg_.transpose() << ",  " << all_image_frame_.begin()->second.vb_in_gravity.transpose() << std::endl;
        // std::cout << "end: " << std::to_string(all_image_frame_.rbegin()->first) << all_image_frame_.rbegin()->second.pre_integration->init_bg_.transpose() << ",  " << all_image_frame_.rbegin()->second.vb_in_gravity.transpose() << std::endl;
        // std::cout << all_image_frame_.begin()->second.R_gravity_from_imu << std::endl;
        // std::cout << all_image_frame_.rbegin()->second.R_gravity_from_imu << std::endl;
        if(result)
        {
          process_active_ = false;
          time_finished_last_ = frame.timestamp;
          std::cout << "Dynamic initialization finished!" << std::endl;
          break;
        }
        else
        {
          slideWindow();
          time_finished_last_ = frame.timestamp;
        }
      }
      else
      {
        frame_count_++;
        bool result = staticInitialize();
        if(result)
        {
          process_active_ = false;
          time_finished_last_ = frame.timestamp;
          break;
        }
        time_finished_last_ = frame.timestamp;
      }
    }
  }
  std::cout << "------------- end of process() --------------" << std::endl;
}

std::vector<Frame> Initializer::dataSync()
{
  // static double time_last{-1.};
  // static Eigen::Vector3d fb_last{0.,0.,0.};
  // static Eigen::Vector3d wib_b_last{0.,0.,0.};  // declared as private me
  std::vector<Frame> frames_out;
  std::lock_guard<std::mutex> lk_data(mutex_data_);
  while(true)
  {
    if(img_info_buf_.empty() || imu_buf_.empty())
    {
      return std::move(frames_out);
    }
    if(imu_buf_.front().timestamp>img_info_buf_.front().timestamp) // drop first img data due to lack of imu data
    {
      img_info_buf_.pop();
      continue;
    }
    if(imu_buf_.back().timestamp<img_info_buf_.front().timestamp) // wait for more imu data
    {
      return std::move(frames_out);
    }
    frames_out.emplace_back(Frame());
    Frame &frame_cur = frames_out.back();
    frame_cur.timestamp = img_info_buf_.front().timestamp;
    frame_cur.img = img_info_buf_.front().img;
    frame_cur.featureId_xyuv = img_info_buf_.front().featureId_xyuv;
    img_info_buf_.pop();
    if(dataSync_time_last_<=0.)
    {
      dataSync_time_last_ = imu_buf_.front().timestamp;
      dataSync_fb_last_ = imu_buf_.front().fb;
      dataSync_wib_b_last_ = imu_buf_.front().wib_b;
      imu_buf_.pop();
    }
    frame_cur.imu_time_cache.emplace_back(dataSync_time_last_);
    frame_cur.fb_cache.emplace_back(dataSync_fb_last_);
    frame_cur.wib_b_cache.emplace_back(dataSync_wib_b_last_);
    while(imu_buf_.front().timestamp<=frame_cur.timestamp)
    {
      dataSync_time_last_ = imu_buf_.front().timestamp;
      dataSync_fb_last_ = imu_buf_.front().fb;
      dataSync_wib_b_last_ = imu_buf_.front().wib_b;
      imu_buf_.pop();
      frame_cur.imu_time_cache.emplace_back(dataSync_time_last_);
      frame_cur.fb_cache.emplace_back(dataSync_fb_last_);
      frame_cur.wib_b_cache.emplace_back(dataSync_wib_b_last_);
    }
    if(dataSync_time_last_<frame_cur.timestamp)
    {
      double time_next = imu_buf_.front().timestamp;
      double k = (frame_cur.timestamp-dataSync_time_last_)/(time_next-dataSync_time_last_);
      Eigen::Vector3d fb_cur = k*imu_buf_.front().fb+(1.-k)*dataSync_fb_last_;
      Eigen::Vector3d wib_b_cur = k*imu_buf_.front().wib_b+(1.-k)*dataSync_wib_b_last_;
      dataSync_time_last_ = frame_cur.timestamp;
      dataSync_fb_last_ = fb_cur;
      dataSync_wib_b_last_ = wib_b_cur;
      frame_cur.imu_time_cache.emplace_back(dataSync_time_last_);
      frame_cur.fb_cache.emplace_back(dataSync_fb_last_);
      frame_cur.wib_b_cache.emplace_back(dataSync_wib_b_last_);
    }
    if(frame_cur.imu_time_cache.size()<2)
    {
      std::cout << __FILE__ << ":" << __LINE__ << " imu data num < 2 !!!" << std::endl;
    }
  }
}

void Initializer::alignWorldToGravity(std::map<double, Frame>& _frames, const Eigen::Vector3d& _g_world, Eigen::Vector3d& _g_gravity_out)
{
  const int frame_num = _frames.size();
  Matrix3d R_gravity_from_world = g2R(_g_world);
  double yaw = R2ypr(R_gravity_from_world * _frames.rbegin()->second.R_world_from_imu).x();
  R_gravity_from_world = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R_gravity_from_world;
  _g_gravity_out = R_gravity_from_world * _g_world;
  Eigen::Vector3d t_gravity_from_world = Eigen::Vector3d::Zero();
  for(auto &frame:_frames)
  {
    frame.second.t_gravity_from_imu = R_gravity_from_world * frame.second.t_world_from_imu + t_gravity_from_world;
    frame.second.R_gravity_from_imu = R_gravity_from_world * frame.second.R_world_from_imu;
    frame.second.vb_in_gravity = R_gravity_from_world * frame.second.vb_in_world;
  }
}

bool Initializer::alignToWorld(std::map<double, Frame>& _frames, Eigen::Vector3d& _g_world_out, bool _has_sacle)
{
  Eigen::Vector3d g_world = Eigen::Vector3d::Zero();
  double s = 1.;
  for(auto &frame:_frames)
  {
    frame.second.R = frame.second.R_world_from_camera * R_imu_from_camera_.transpose();
    frame.second.t = frame.second.t_world_from_camera;
  }
  if(recoverBVGS(_frames, g_world, s, _has_sacle))
  {
    _g_world_out = g_world;
    int i{0};
    Eigen::Vector3d t_tmp = (s * _frames.begin()->second.t - _frames.begin()->second.R * t_imu_from_camera_);
    for(std::map<double, Frame>::iterator frame_i = _frames.begin(); frame_i != _frames.end(); frame_i++, i++)
    {
      frame_i->second.R_world_from_imu = frame_i->second.R;
      frame_i->second.t_world_from_imu = s * frame_i->second.t - frame_i->second.R * t_imu_from_camera_ - t_tmp;
      frame_i->second.vb_in_world = frame_i->second.R * frame_i->second.vb;
    }
    return true;
  }
  else
  {
    return false;
  }
}

bool Initializer::recoverBVGS(std::map<double, Frame>& _frames, Eigen::Vector3d &_g_world_out, double& _s_out, bool _has_sacle)
{
  solveBg(_frames);
  if(solveVGS(_frames, _g_world_out, _s_out, _has_sacle))
  {
    if(refineVGS(_frames, _g_world_out, _g_world_out, _s_out, _has_sacle))
    {
      return true;
    }
  }
  return false;
}

void Initializer::solveBg(std::map<double, Frame>& _frames)
{
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  std::map<double, Frame>::iterator frame_i;
  std::map<double, Frame>::iterator frame_j;
  Eigen::Matrix3d Aij;
  Eigen::Vector3d bij;
  for(frame_i = _frames.begin(); next(frame_i) != _frames.end(); frame_i++)
  {
    frame_j = next(frame_i);
    Aij.setZero();
    bij.setZero();

    Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
    Aij = frame_j->second.pre_integration->jacobian_.template block<3, 3>(EsAngleIdx, EsGyroBiasIdx);
    bij = 2 * (frame_j->second.pre_integration->delta_q_.inverse() * q_ij).vec();

    A += Aij.transpose()*Aij;
    b += Aij.transpose()*bij;
  }
  Eigen::Vector3d delta_bg = A.ldlt().solve(b);
  // ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());

  for(auto &frame: _frames)
  {
    Eigen::Vector3d ba, bg;
    frame.second.pre_integration->getBias(ba, bg);
    frame.second.pre_integration->setBias(ba, bg+delta_bg);
    frame.second.pre_integration->setBias(ba, Eigen::Vector3d(0.020219471,-0.005973533,-0.007718516));
    frame.second.pre_integration->reintegrate();
  }
}

bool Initializer::solveVGS(std::map<double, Frame>& _frames, Eigen::Vector3d &_g_world_out, double& _s_out, bool _has_sacle)
{
  const int frame_num = _frames.size();
  const int dim_x = 3 * frame_num + 3 + 1;
  Eigen::MatrixXd A(dim_x, dim_x);
  A.setZero();
  Eigen::VectorXd b(dim_x);
  b.setZero();
  std::map<double, Frame>::iterator frame_i;
  std::map<double, Frame>::iterator frame_j;
  Eigen::MatrixXd Aij(6, 10);
  Eigen::VectorXd bij(6);
  Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Identity();
  int i{0};
  for(frame_i = _frames.begin(); next(frame_i) != _frames.end(); frame_i++, i++)
  {
    frame_j = next(frame_i);
    Aij.setZero();
    bij.setZero();
    
    const double dt = frame_j->second.pre_integration->sum_dt_;
    Aij.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
    Aij.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Eigen::Matrix3d::Identity();
    Aij.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.t - frame_i->second.t) / 100.0;     
    bij.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p_ + frame_i->second.R.transpose() * frame_j->second.R * t_imu_from_camera_ - t_imu_from_camera_;

    Aij.block<3, 3>(3, 0) = -Matrix3d::Identity();
    Aij.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
    Aij.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity();
    bij.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v_;

    Eigen::MatrixXd ATA = Aij.transpose() * cov_inv * Aij;
    Eigen::VectorXd ATb = Aij.transpose() * cov_inv * bij;

    A.block<6, 6>(i * 3, i * 3) += ATA.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += ATb.head<6>();

    A.bottomRightCorner<4, 4>() += ATA.bottomRightCorner<4, 4>();
    b.tail<4>() += ATb.tail<4>();

    A.block<6, 4>(i * 3, dim_x - 4) += ATA.topRightCorner<6, 4>();
    A.block<4, 6>(dim_x - 4, i * 3) += ATA.bottomLeftCorner<4, 6>();
  }

  A = A * 1000.;
  b = b * 1000.;
  Eigen::VectorXd x = A.ldlt().solve(b);
  _g_world_out = x.segment<3>(dim_x - 4);
  _s_out = x(dim_x - 1) / 100.;

  if(fabs(_g_world_out.norm() - g_norm_) > 1. || _s_out < 0.)
  {
    return false;
  }
  else
  {
    int i{0};
    for(frame_i = _frames.begin(); frame_i != _frames.end(); frame_i++, i++)
    {
      frame_i->second.vb = x.segment<3>(i*3);
    }
    return true;
  }
}

bool Initializer::refineVGS(std::map<double, Frame>& _frames, const Eigen::Vector3d _g_world_init, Eigen::Vector3d &_g_world_out, double& _s_out, bool _has_sacle)
{
  const int frame_num = _frames.size();
  const int dim_x = 3 * frame_num + 2 + 1;
  Eigen::MatrixXd A(dim_x, dim_x);
  Eigen::VectorXd b(dim_x);
  std::map<double, Frame>::iterator frame_i;
  std::map<double, Frame>::iterator frame_j;
  Eigen::MatrixXd Aij(6, 9);
  Eigen::VectorXd bij(6);
  Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Vector3d g_world = _g_world_init.normalized() * g_norm_;
  Eigen::VectorXd x;
  for(int k=0; k<4; k++)
  {
    Eigen::MatrixXd lxly = tangentBasis(g_world);
    int i = 0;
    A.setZero();
    b.setZero();
    for(frame_i = _frames.begin(); next(frame_i) != _frames.end(); frame_i++, i++)
    {
      frame_j = next(frame_i);
      Aij.setZero();
      bij.setZero();

      const double dt = frame_j->second.pre_integration->sum_dt_;
      Aij.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
      Aij.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
      Aij.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.t - frame_i->second.t) / 100.0;     
      bij.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p_ + frame_i->second.R.transpose() * frame_j->second.R * t_imu_from_camera_ - t_imu_from_camera_ - frame_i->second.R.transpose() * dt * dt / 2 * g_world;

      Aij.block<3, 3>(3, 0) = -Matrix3d::Identity();
      Aij.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
      Aij.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
      bij.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v_ - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g_world;

      Eigen::MatrixXd ATA = Aij.transpose() * cov_inv * Aij;
      Eigen::VectorXd ATb = Aij.transpose() * cov_inv * bij;

      A.block<6, 6>(i * 3, i * 3) += ATA.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += ATb.head<6>();

      A.bottomRightCorner<3, 3>() += ATA.bottomRightCorner<3, 3>();
      b.tail<3>() += ATb.tail<3>();

      A.block<6, 3>(i * 3, dim_x - 3) += ATA.topRightCorner<6, 3>();
      A.block<3, 6>(dim_x - 3, i * 3) += ATA.bottomLeftCorner<3, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    Eigen::VectorXd dg = x.segment<2>(dim_x - 3);
    g_world = (g_world + lxly * dg).normalized() * g_norm_;
  }
  _g_world_out = g_world;
  _s_out = x(dim_x - 1)/100.;
  // fout_rg.close();
  if(_s_out < 0. )
    return false;   
  else
  {
    int i{0};
    for(frame_i = _frames.begin(); frame_i != _frames.end(); frame_i++, i++)
    {
      frame_i->second.vb = x.segment<3>(i*3);
    }
    return true;
  }
}

Eigen::MatrixXd Initializer::tangentBasis(Eigen::Vector3d &_vec)
{
  Eigen::Vector3d x_dir = _vec.normalized();
  Eigen::Vector3d tmp(0, 0, 1);
  if(x_dir == tmp)
      tmp << 1, 0, 0;
  Eigen::Vector3d y_dir = (tmp - x_dir * (x_dir.transpose() * tmp)).normalized();
  Eigen::Vector3d z_dir = x_dir.cross(y_dir);
  Eigen::MatrixXd yz(3, 2);
  yz.block<3, 1>(0, 0) = y_dir;
  yz.block<3, 1>(0, 1) = z_dir;
  return yz;
}

Eigen::Matrix3d Initializer::g2R(const Eigen::Vector3d &_vec) const
{
  Eigen::Matrix3d R0;
  Eigen::Vector3d ng1 = _vec.normalized();
  Eigen::Vector3d ng2{0, 0, 1.0};
  R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
  double yaw = R2ypr(R0).x();
  R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  return R0;
}

Eigen::Vector3d Initializer::R2ypr(const Eigen::Matrix3d &_R) const
{
  Eigen::Vector3d n = _R.col(0);
  Eigen::Vector3d o = _R.col(1);
  Eigen::Vector3d a = _R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> Initializer::ypr2R(const Eigen::MatrixBase<Derived> &_ypr) const
{
  typedef typename Derived::Scalar Scalar_t;

  Scalar_t y = _ypr(0) / 180.0 * M_PI;
  Scalar_t p = _ypr(1) / 180.0 * M_PI;
  Scalar_t r = _ypr(2) / 180.0 * M_PI;

  Eigen::Matrix<Scalar_t, 3, 3> Rz;
  Rz << cos(y), -sin(y), 0,
      sin(y), cos(y), 0,
      0, 0, 1;

  Eigen::Matrix<Scalar_t, 3, 3> Ry;
  Ry << cos(p), 0., sin(p),
      0., 1., 0.,
      -sin(p), 0., cos(p);

  Eigen::Matrix<Scalar_t, 3, 3> Rx;
  Rx << 1., 0., 0.,
      0., cos(r), -sin(r),
      0., sin(r), cos(r);

  return Rz * Ry * Rx;
}

void Initializer::visualTrack()
{
  while(process_active_)
  {
    std::unique_lock<std::mutex> lk_visual(mutex_visual_);
    con_visual_.wait(lk_visual);
    lk_visual.unlock();
    if(!visual_active_) break;
    while(!img_buf_.empty())
    {
      lk_visual.lock();
      double time = img_buf_.front().first;
      cv::Mat img = img_buf_.front().second;
      img_buf_.pop();
      lk_visual.unlock();

      if(feature_tracker_.addImage(time, img))
      {
        ImageInfo img_info = feature_tracker_.getTrackResult();
        addImageInfo(img_info);
      }
    }
    
  }
  std::cout << "------------- end of visualTrack() --------------" << std::endl;
}

bool Initializer::addFeatureCheckParallax(int _frame_count, const std::map<FeatureIdType, Eigen::Vector4d> &_featureId_xyuv)
{
  double parallax_sum = 0;
  int parallax_num = 0;
  int last_track_num = 0;
  for (auto &id_pts : _featureId_xyuv)
  {
    FeaturePerFrame f_per_fra(id_pts.second);

    FeatureIdType feature_id = id_pts.first;
    auto it = find_if(feature_list_.begin(), feature_list_.end(), [feature_id](const FeaturePerId &it)
                      {
        return it.feature_id == feature_id;
                      });

    if (it == feature_list_.end())
    {
      feature_list_.push_back(FeaturePerId(feature_id, _frame_count));
      feature_list_.back().feature_per_frame.push_back(f_per_fra);
    }
    else if (it->feature_id == feature_id)
    {
      it->feature_per_frame.push_back(f_per_fra);
      last_track_num++;
    }
  }

  if(_frame_count < 2 || last_track_num < 20)
  {
    return true;
  }

  for(auto &it_per_id : feature_list_)
  {
    if(it_per_id.start_frame <= _frame_count - 2 &&
      it_per_id.endFrame() >= _frame_count - 1)
    {
      parallax_sum += compensatedParallax2(it_per_id, _frame_count);
      parallax_num++;
    }
  }

  if (parallax_num == 0)
  {
    return true;
  }
  else
  {
    return parallax_sum / parallax_num >= MIN_PARALLAX;
  }
}

double Initializer::compensatedParallax2(const FeaturePerId &_it_per_id, int _frame_count)
{
  //check the second last frame is keyframe or not
  //parallax betwwen seconde last frame and third last frame
  const FeaturePerFrame &frame_i = _it_per_id.feature_per_frame[_frame_count - 2 - _it_per_id.start_frame];
  const FeaturePerFrame &frame_j = _it_per_id.feature_per_frame[_frame_count - 1 - _it_per_id.start_frame];

  Vector3d p_j = frame_j.point;

  double u_j = p_j(0);
  double v_j = p_j(1);

  Vector3d p_i = frame_i.point;
  double dep_i = p_i(2);
  double u_i = p_i(0) / dep_i;
  double v_i = p_i(1) / dep_i;
  double du = u_i - u_j, dv = v_i - v_j;

  return std::sqrt(du * du + dv * dv);
}

bool Initializer::initialStructure()
{
  //check imu observibility
  {
    std::map<double, Frame>::iterator frame_it;
    Vector3d sum_g;
    for (frame_it = all_image_frame_.begin(), frame_it++; frame_it != all_image_frame_.end(); frame_it++)
    {
      double dt = frame_it->second.pre_integration->sum_dt_;
      Vector3d tmp_g = frame_it->second.pre_integration->delta_v_ / dt;
      sum_g += tmp_g;
    }
    Vector3d aver_g;
    aver_g = sum_g * 1.0 / ((int)all_image_frame_.size() - 1);
    double var = 0;
    for (frame_it = all_image_frame_.begin(), frame_it++; frame_it != all_image_frame_.end(); frame_it++)
    {
      double dt = frame_it->second.pre_integration->sum_dt_;
      Vector3d tmp_g = frame_it->second.pre_integration->delta_v_ / dt;
      var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
      //std::cout << "frame g " << tmp_g.transpose() << std::endl;
    }
    var = sqrt(var / ((int)all_image_frame_.size() - 1));
    if(var < 0.25)
    {
      std::cout << "IMU excitation not enouth!" << std::endl;
      //return false;
    }
  }
  // global sfm
  std::vector<Eigen::Quaterniond> Q(frame_count_+1);
  std::vector<Eigen::Vector3d> T(frame_count_+1);
  std::map<int, Eigen::Vector3d> sfm_tracked_points; // 关键帧sfm得到的地图点坐标，key-地图点id, value-地图点坐标
  std::vector<SFMFeature> sfm_f; // 每个元素绑定一个地图点观测，内部存储了该地图点在所有观测帧中的去畸变归一化坐标以及对应的滑窗内帧索引
  for (auto &it_per_id : feature_list_) // 遍历所有地图点
  {
    int imu_j = it_per_id.start_frame - 1;
    SFMFeature tmp_feature;
    tmp_feature.state = false;
    tmp_feature.id = it_per_id.feature_id;
    for (auto &it_per_frame : it_per_id.feature_per_frame) // 遍历该地图点在所有关键帧中的观测信息
    {
      imu_j++;
      Eigen::Vector3d pts_j = it_per_frame.point;
      tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()})); // frame_id(滑窗内frame索引), 去畸变归一化坐标
    }
    sfm_f.push_back(tmp_feature);
  } 
  Eigen::Matrix3d relative_R;
  Eigen::Vector3d relative_T;
  int l;
  if (!relativePose(relative_R, relative_T, l))
  {
    std::cout << "Not enough features or parallax; Move device around" << std::endl;
    return false;
  }
  GlobalSFM sfm;
  if(!sfm.construct(frame_count_ + 1, Q.data(), T.data(), l,
            relative_R, relative_T,
            sfm_f, sfm_tracked_points))
  {
    std::cout << "global SFM failed!" << std::endl;
    marginalization_flag_ = MARGIN_OLD;
    return false;
  }

  //solve pnp for all frame
  std::map<double, Frame>::iterator frame_it;
  std::map<int, Vector3d>::iterator it;
  frame_it = all_image_frame_.begin( );
  std::map<double, std::pair<Eigen::Matrix3d, Eigen::Vector3d>> time_Rt;
    for (int i = 0; frame_it != all_image_frame_.end( ); frame_it++)
  {
    // provide initial guess
    cv::Mat r, rvec, t, D, tmp_r;
    if((frame_it->first) == keyframe_times_[i])
    {
        frame_it->second.is_key_frame = true;
        frame_it->second.R_world_from_camera = Q[i].toRotationMatrix();
        frame_it->second.t_world_from_camera = T[i];
        i++;
        continue;
    }
    if((frame_it->first) > keyframe_times_[i])
    {
      i++;
    }
    Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
    Vector3d P_inital = - R_inital * T[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    frame_it->second.is_key_frame = false;
    vector<cv::Point3f> pts_3_vector;
    vector<cv::Point2f> pts_2_vector;
    for (auto &id_pts : frame_it->second.featureId_xyuv)
    {
      FeatureIdType feature_id = id_pts.first;
      it = sfm_tracked_points.find(feature_id);
      if(it != sfm_tracked_points.end())
      {
        Vector3d world_pts = it->second;
        cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
        pts_3_vector.push_back(pts_3);
        Vector2d img_pts = id_pts.second.head<2>();
        cv::Point2f pts_2(img_pts(0), img_pts(1));
        pts_2_vector.push_back(pts_2);
      }
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
    if(pts_3_vector.size() < 6)
    {
      std::cout << "pts_3_vector size " << pts_3_vector.size() << endl;
      std::cout << "Not enough points for solve pnp !" << std::endl;
      return false;
    }
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
    {
      std::cout << "solve pnp fail!" << std::endl;
      return false;
    }
    cv::Rodrigues(rvec, r);
    Eigen::MatrixXd R_pnp,tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    T_pnp = R_pnp * (-T_pnp);
    frame_it->second.R_world_from_camera = R_pnp;
    frame_it->second.t_world_from_camera = T_pnp;
  }
  if (visualInitialAlign())
  {
    time_result_ = all_image_frame_.rbegin()->first;
    Rni_result_ = all_image_frame_.rbegin()->second.R_gravity_from_imu;
    bg_result_ = all_image_frame_.rbegin()->second.pre_integration->init_bg_;
    vn_result_ = all_image_frame_.rbegin()->second.vb_in_gravity;
    g_world_result_ = g_world_;
    g_gravity_result_ = g_gravity_;
    return true;
  }
  else
  {
    std::cout << "misalign visual structure with IMU" << std::endl;
    return false;
  }
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> Initializer::getCorresponding(int _frame_count_l, int _frame_count_r)
{
  vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
  for (auto &it : feature_list_)
  {
    if (it.start_frame <= _frame_count_l && it.endFrame() >= _frame_count_r)
    {
      Eigen::Vector3d a = Eigen::Vector3d::Zero(), b = Eigen::Vector3d::Zero();
      int idx_l = _frame_count_l - it.start_frame;
      int idx_r = _frame_count_r - it.start_frame;

      a = it.feature_per_frame[idx_l].point;

      b = it.feature_per_frame[idx_r].point;
      
      corres.push_back(make_pair(a, b));
    }
  }
  return corres;
}

bool Initializer::solveRelativeRT(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &_corres, Eigen::Matrix3d &_Rotation, Eigen::Vector3d &_Translation)
{
  if (_corres.size() >= 15)
  {
    std::vector<cv::Point2f> ll, rr;
    for (int i = 0; i < int(_corres.size()); i++)
    {
      ll.push_back(cv::Point2f(_corres[i].first(0), _corres[i].first(1)));
      rr.push_back(cv::Point2f(_corres[i].second(0), _corres[i].second(1)));
    }
    cv::Mat mask;
    cv::Mat E = cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 0.3 / 460, 0.99, mask);
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat rot, trans;
    int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
    //cout << "inlier_cnt " << inlier_cnt << endl;

    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    for (int i = 0; i < 3; i++)
    {   
      T(i) = trans.at<double>(i, 0);
      for (int j = 0; j < 3; j++)
      {
        R(i, j) = rot.at<double>(i, j);
      }
    }

    _Rotation = R.transpose();
    _Translation = -R.transpose() * T;
    if(inlier_cnt > 12)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

bool Initializer::relativePose(Eigen::Matrix3d &_relative_R, Eigen::Vector3d &_relative_T, int &_l)
{
  // find previous frame which contians enough correspondance and parallex with newest frame
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
    corres = getCorresponding(i, WINDOW_SIZE);
    if (corres.size() > 20)
    {
      double sum_parallax = 0;
      double average_parallax;
      for (int j = 0; j < int(corres.size()); j++)
      {
        Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
        Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax = sum_parallax + parallax;
      }
      average_parallax = 1.0 * sum_parallax / int(corres.size());
      if(average_parallax * FOCAL_LENGTH > 30 && solveRelativeRT(corres, _relative_R, _relative_T))
      {
        _l = i;
        std::cout << "average_parallax " << average_parallax*FOCAL_LENGTH << " choose l " << _l << " and newest frame to triangulate the whole structure" << std::endl;
        return true;
      }
    }
  }
  return false;
}

bool Initializer::visualInitialAlign()
{
  bool success{false};
  if(alignToWorld(all_image_frame_, g_world_, false))
  {
    alignWorldToGravity(all_image_frame_, g_world_, g_gravity_);
    success = true;
  }
  else
  {
    success = false;
  }
  for(int i = 0; i <= WINDOW_SIZE; i++)
  {
    Eigen::Vector3d ba, bg;
    all_image_frame_[keyframe_times_[i]].pre_integration->getBias(ba, bg);
    keyframe_bas_[i] = ba;
    keyframe_bgs_[i] = bg;
  }
  return success;
}

void Initializer::slideWindow()
{
  if (marginalization_flag_ == MARGIN_OLD)
  {
    if (frame_count_ == WINDOW_SIZE)
    {
      double t_0 = keyframe_times_[0];
      for (int i = 0; i < WINDOW_SIZE; i++)
      {
        keyframe_times_[i] = keyframe_times_[i + 1];
        keyframe_bas_[i].swap(keyframe_bas_[i + 1]);
        keyframe_bgs_[i].swap(keyframe_bgs_[i + 1]);
      }
      keyframe_times_[WINDOW_SIZE] = keyframe_times_[WINDOW_SIZE - 1];
      keyframe_bas_[WINDOW_SIZE] = keyframe_bas_[WINDOW_SIZE - 1];
      keyframe_bgs_[WINDOW_SIZE] = keyframe_bgs_[WINDOW_SIZE - 1];

      if (true)
      {
        map<double, Frame>::iterator it_0;
        it_0 = all_image_frame_.find(t_0);
        if(it_0->second.pre_integration)
        {
          it_0->second.pre_integration.reset();
        }
        for(map<double, Frame>::iterator it = all_image_frame_.begin(); it != it_0; ++it)
        {
          if(it->second.pre_integration)
          {
            it->second.pre_integration.reset();
          }
        }
        all_image_frame_.erase(all_image_frame_.begin(), it_0);
        all_image_frame_.erase(t_0);
      }
      slideWindowOld();
    }
  }
  else
  {
    if (frame_count_ == WINDOW_SIZE)
    {
      keyframe_times_[frame_count_ - 1] = keyframe_times_[frame_count_];
      keyframe_bas_[frame_count_ - 1] = keyframe_bas_[frame_count_];
      keyframe_bgs_[frame_count_ - 1] = keyframe_bgs_[frame_count_];
      slideWindowNew();
    }
  }
}

void Initializer::slideWindowNew()
{
  for (auto it = feature_list_.begin(), it_next = feature_list_.begin(); it != feature_list_.end(); it = it_next)
  {
    it_next++;

    if(it->start_frame == frame_count_)
    {
      it->start_frame--;
    }
    else
    {
      int j = WINDOW_SIZE - 1 - it->start_frame;
      if(it->endFrame() < frame_count_ - 1)
      {
        continue;
      }
      it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
      if(it->feature_per_frame.size() == 0)
      {
        feature_list_.erase(it);
      }
    }
  }
}
void Initializer::slideWindowOld()
{
  for (auto it = feature_list_.begin(), it_next = feature_list_.begin(); it != feature_list_.end(); it = it_next)
  {
    it_next++;
    if (it->start_frame != 0)
    {
      it->start_frame--;
    }
    else
    {
      it->feature_per_frame.erase(it->feature_per_frame.begin());
      if (it->feature_per_frame.size() == 0)
      {
        feature_list_.erase(it);
      }
    }
  }
}

double Initializer::getLastFinishedTime() const
{
  return time_finished_last_;
}

bool Initializer::isProcessFinish() const
{
  return !process_active_;
}

// 获取初始化结果
bool Initializer::getInitResult(double &_time, Eigen::Matrix3d &_Rni, Eigen::Vector3d &_bg, Eigen::Vector3d &_vn, Eigen::Vector3d &_gn) const
{
  if(process_active_)
  {
    _time = time_finished_last_;
    return false;
  }
  else
  {
    _time = time_result_;
    _Rni = Rni_result_;
    _bg = bg_result_;
    _vn = vn_result_;
    _gn = -g_gravity_result_;
    std::cout << "============ getInitResult =============\n";
    std::cout << "time: " << std::to_string(_time) << std::endl;
    std::cout << "Rni:\n" << _Rni << std::endl;
    std::cout << "bg: " << _bg.transpose() << std::endl;
    std::cout << "vn: " << _vn.transpose() << std::endl;
    std::cout << "gn: " << _gn.transpose() << std::endl;
    std::cout << "gw: " << g_world_result_.transpose() << std::endl;
    return true;
  }
}

bool Initializer::checkStop(Frame &_frame1, Frame &_frame2)
{
  auto it1 = _frame1.featureId_xyuv.begin();
  auto it1_end = _frame1.featureId_xyuv.end();
  auto it2 = _frame2.featureId_xyuv.begin();
  auto it2_end = _frame2.featureId_xyuv.end();
  int num{0};
  int den{static_cast<int>(_frame1.featureId_xyuv.size())};
  double error_normal_sum{0.};
  double error_normal_max{0.};

  while(it1 != it1_end && it2 != it2_end)
  {
    if(it1->first == it2->first)
    {
      Eigen::Vector2d delta_uv = it1->second.tail(2) - it2->second.tail(2);
      delta_uv.x() *= fx_inv_ * FOCAL_LENGTH;
      delta_uv.y() *= fy_inv_ * FOCAL_LENGTH;
      double error_normal = delta_uv.norm();
      error_normal_sum += error_normal;
      error_normal_max = error_normal > error_normal_max ? error_normal : error_normal_max;
      num++;
      it1++;
      it2++;
    }
    else if(it1->first < it2->first)
    {
      it1++;
    }
    else
    {
      it2++;
    }
  }

  double ratio = double(num)/den;
  double error_normal_avg = error_normal_sum/num;
  std::cout << "time1: " << std::to_string(_frame1.timestamp) << ", time2: " << std::to_string(_frame2.timestamp) 
            << ", den: " << den << ", num: " << num << ", ratio: " << ratio
            << ", error_pixel_avg: " << error_normal_avg << ", error_pixel_max: " << error_normal_max 
            << std::endl;
  if(ratio > 0.98 && error_normal_avg < 2. && error_normal_max < 4.)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Initializer::staticInitialize()
{
  bool success{false};
  double time_l{all_image_frame_.rbegin()->first};
  double time_r{all_image_frame_.rbegin()->first};
  
  for(auto it = all_image_frame_.rbegin(); it!= all_image_frame_.rend(); it++)
  {
    if(it->second.is_stop)
    {
      time_l = it->first;
    }
    else
    {
      break;
    }
  }
  if(time_r - time_l >= 1.)
  {
    success = true;
    int N = 0;
    Eigen::Vector3d fb_mean(0., 0., 0.);
    Eigen::Vector3d wib_b_mean(0.,0.,0.);
    Eigen::Vector3d fb_cov(0.,0.,0.);
    Eigen::Vector3d wib_b_cov(0.,0.,0.);
    auto it_l = std::next(all_image_frame_.find(time_l));
    auto it_r = std::next(all_image_frame_.find(time_r));
    for(auto it = it_l; it!= it_r; it++)
    {
      for(int i=0; i<it->second.pre_integration->time_buf.size(); i++)
      {
        Eigen::Vector3d &fb_cur = it->second.pre_integration->fb_buf[i];
        Eigen::Vector3d &wib_b_cur = it->second.pre_integration->wib_b_buf[i];
        fb_mean += (fb_cur - fb_mean) / (N+1);
        wib_b_mean += (wib_b_cur - wib_b_mean) / (N+1);
        fb_cov = fb_cov * N / (N+1) + (fb_cur - fb_mean).cwiseProduct(fb_cur - fb_mean) * N / ((N+1) * (N+1));
        wib_b_cov = wib_b_cov * N / (N+1) + (wib_b_cur - wib_b_mean).cwiseProduct(wib_b_cur - wib_b_mean) * N / ((N+1) * (N+1));
        N++;
      }
    }
    time_result_ = all_image_frame_.rbegin()->first;
    bg_result_ = wib_b_mean;
    vn_result_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_imu = fb_mean;
    g_world_result_ = R_imu_from_camera_.transpose() * fb_mean;
    g_gravity_result_ << 0., 0., fb_mean.norm();
    Rni_result_ = g2R(fb_mean);
  }
  else
  {
    success = false;
  }
  return success;
}

void Initializer::clearBuf()
{
  mutex_data_.lock();
  while(!imu_buf_.empty())
  {
    imu_buf_.pop();
  }
  while(!img_info_buf_.empty())
  {
    img_info_buf_.pop();
  }
  while(!lidar_buf_.empty())
  {
    lidar_buf_.pop();
  }
  mutex_data_.unlock();

  mutex_visual_.lock();
  while(!img_buf_.empty())
  {
    img_buf_.pop();
  }
  mutex_visual_.unlock();
  std::cout << "Initialization buffers have been cleared." << std::endl;

}



} // namespace livo_initial
