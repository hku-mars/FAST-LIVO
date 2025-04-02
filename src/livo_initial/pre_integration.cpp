#include <iostream>
#include "livo_initial/pre_integration.h"

namespace livo_initial {

Eigen::Vector3d PreIntergration::acc_std_ = Eigen::Vector3d::Zero();
Eigen::Vector3d PreIntergration::gyro_std_ = Eigen::Vector3d::Zero();
Eigen::Vector3d PreIntergration::ba_std_ = Eigen::Vector3d::Zero();
Eigen::Vector3d PreIntergration::bg_std_ = Eigen::Vector3d::Zero();
bool PreIntergration::is_std_set_ = false;

void PreIntergration::resetStatics()
{
  acc_std_ = Eigen::Vector3d::Zero();
  gyro_std_ = Eigen::Vector3d::Zero();
  ba_std_ = Eigen::Vector3d::Zero();
  bg_std_ = Eigen::Vector3d::Zero();
  is_std_set_ = false;
}

void PreIntergration::setNoiseStd(const Eigen::Vector3d _acc_std, const Eigen::Vector3d _gyro_std, const Eigen::Vector3d _ba_std, const Eigen::Vector3d _bg_std)
{
  acc_std_ = _acc_std;
  gyro_std_ = _gyro_std;
  ba_std_ = _ba_std;
  bg_std_ = _bg_std;
  is_std_set_ = true;
}

PreIntergration::PreIntergration(const double _init_time, const Eigen::Vector3d _init_fb, const Eigen::Vector3d _init_wib_b, const Eigen::Vector3d _init_ba, const Eigen::Vector3d _init_bg)
: init_ba_(_init_ba), init_bg_(_init_bg), fb_0_(_init_fb), wib_b_0_(_init_wib_b), ba_0_(_init_ba), bg_0_(_init_bg)
{
#ifdef ENABLE_COV
  if (!is_std_set_)
  {
    std::cout << "[PreIntergration] Noise std is not set!" << std::endl;
    exit(0);
  }
  noise_.block<3, 3>(0, 0) =  acc_std_.cwiseAbs2().asDiagonal();
  noise_.block<3, 3>(3, 3) =  gyro_std_.cwiseAbs2().asDiagonal();
  noise_.block<3, 3>(6, 6) =  acc_std_.cwiseAbs2().asDiagonal();
  noise_.block<3, 3>(9, 9) =  gyro_std_.cwiseAbs2().asDiagonal();
  noise_.block<3, 3>(12, 12) =  ba_std_.cwiseAbs2().asDiagonal();
  noise_.block<3, 3>(15, 15) =  bg_std_.cwiseAbs2().asDiagonal();
#endif
  time_buf.emplace_back(_init_time);
  dt_buf.emplace_back(0.);
  fb_buf.emplace_back(_init_fb);
  wib_b_buf.emplace_back(_init_wib_b);
}

void PreIntergration::addImuData(const double _time, const Eigen::Vector3d _fb, const Eigen::Vector3d _wib_b)
{
  dt_buf.emplace_back(_time-time_buf.back());
  time_buf.emplace_back(_time);
  fb_buf.emplace_back(_fb);
  wib_b_buf.emplace_back(_wib_b);
  integrateOnce(dt_buf.back(), fb_buf.back(), wib_b_buf.back());
}

void PreIntergration::getBias(Eigen::Vector3d &_ba, Eigen::Vector3d &_bg) const
{
  _ba = init_ba_;
  _bg = init_bg_;
}

void PreIntergration::setBias(const Eigen::Vector3d _ba, const Eigen::Vector3d _bg)
{
  init_ba_ = _ba;
  init_bg_ = _bg;
}

void PreIntergration::reset()
{
  sum_dt_ = 0.;
  delta_p_.setZero();
  delta_q_.setIdentity();
  delta_v_.setZero();
  jacobian_.setIdentity();
  covariance_.setZero();

  fb_0_ = fb_buf.front();
  wib_b_0_ = wib_b_buf.front();
  ba_0_ = init_ba_;
  bg_0_ = init_bg_;
}

int PreIntergration::getDataNum() const
{
  return time_buf.size();
}

void PreIntergration::integrateOnce(const double _dt, const Eigen::Vector3d _fb, const Eigen::Vector3d _wib_b)
{
  Eigen::Vector3d result_delta_p;
  Eigen::Quaterniond result_delta_q;
  Eigen::Vector3d result_delta_v;
  Eigen::Vector3d result_linearized_ba;
  Eigen::Vector3d result_linearized_bg;

  midPointIntegration(_dt, fb_0_, wib_b_0_, _fb, _wib_b, delta_p_, delta_q_, delta_v_,
                      ba_0_, bg_0_,
                      result_delta_p, result_delta_q, result_delta_v,
                      result_linearized_ba, result_linearized_bg, 1);

  delta_p_ = result_delta_p;
  delta_q_ = result_delta_q;
  delta_v_ = result_delta_v;
  ba_0_ = result_linearized_ba;
  bg_0_ = result_linearized_bg;
  delta_q_.normalize();
  sum_dt_ += _dt;
  fb_0_ = _fb;
  wib_b_0_ = _wib_b;  
}

void PreIntergration::reintegrate()
{
  reset();
  for(size_t i=1, i_end=dt_buf.size(); i<i_end; i++)
  {
    integrateOnce(dt_buf[i], fb_buf[i], wib_b_buf[i]);
  }
}

void PreIntergration::midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
{
    //ROS_INFO("midpoint integration");
    Eigen::Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    Eigen::Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    Eigen::Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;         

    if(update_jacobian)
    {
      Eigen::Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
      Eigen::Vector3d a_0_x = _acc_0 - linearized_ba;
      Eigen::Vector3d a_1_x = _acc_1 - linearized_ba;
      Eigen::Matrix3d R_w_x, R_a_0_x, R_a_1_x;

      R_w_x<<0, -w_x(2), w_x(1),
          w_x(2), 0, -w_x(0),
          -w_x(1), w_x(0), 0;
      R_a_0_x<<0, -a_0_x(2), a_0_x(1),
          a_0_x(2), 0, -a_0_x(0),
          -a_0_x(1), a_0_x(0), 0;
      R_a_1_x<<0, -a_1_x(2), a_1_x(1),
          a_1_x(2), 0, -a_1_x(0),
          -a_1_x(1), a_1_x(0), 0;

      Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);
      F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt + 
                            -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Eigen::Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
      F.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3,3) * _dt;
      F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
      F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
      F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - R_w_x * _dt;
      F.block<3, 3>(3, 12) = -1.0 * Eigen::MatrixXd::Identity(3,3) * _dt;
      F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt + 
                            -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Eigen::Matrix3d::Identity() - R_w_x * _dt) * _dt;
      F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
      F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
      F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
      F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
      F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
      //cout<<"A"<<endl<<A<<endl;
      jacobian_ = F * jacobian_;
#ifdef ENABLE_COV
      Eigen::MatrixXd V = Eigen::MatrixXd::Zero(15,18);
      V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
      V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
      V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
      V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
      V.block<3, 3>(3, 3) =  0.5 * Eigen::MatrixXd::Identity(3,3) * _dt;
      V.block<3, 3>(3, 9) =  0.5 * Eigen::MatrixXd::Identity(3,3) * _dt;
      V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
      V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
      V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
      V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
      V.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3,3) * _dt;
      V.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3,3) * _dt;

      covariance_ = F * covariance_ * F.transpose() + V * noise_ * V.transpose();
#endif
    }

}

} // namespace livo_initial