#ifndef PRE_INTEGRATION_H
#define PRE_INTEGRATION_H

#include <Eigen/Dense>
#include <vector>

// #define ENABLE_COV

namespace livo_initial
{
enum StateIdx {
  // -------State------------//
  StatePositionIdx = 0,
  StateAngleIdx = 3,  // quternion: [x,y,z,w], using Hamliton convention,
  StateLinearVelIdx = 7,
  StateAccBiasIdx = 10,
  StateGyroBiasIdx = 13,
  
  // Error-State
  EsPositionIdx = StatePositionIdx,
  EsAngleIdx = StateAngleIdx,  // quternion: [x,y,z,w], using Hamliton convention,
  EsLinearVelIdx = StateLinearVelIdx-1,
  EsAccBiasIdx = StateAccBiasIdx - 1,
  EsGyroBiasIdx = StateGyroBiasIdx - 1,
};
enum StateDim{
  StatePositionDim = 3,
  StateAngleDim = 4,  // quternion: [x,y,z,w], using Hamliton convention,
  StateLinearVelDim = 3,
  StateAccBiasDim = 3,
  StateGyroBiasDim = 3,
  StateNum = 5,
  StateDim = 16,

  EsPositionDim = StatePositionDim,
  EsAngleDim = StateAngleDim-1,
  EsLinearVelDim = StateLinearVelDim,
  EsAccBiasDim = StateAccBiasDim,
  EsGyroBiasDim = StateGyroBiasDim,
  EsNum = StateNum,
  EsDim = StateDim - 1,
};

class PreIntergration
{
public:
  PreIntergration(const double _init_time, const Eigen::Vector3d _init_fb, const Eigen::Vector3d _init_wib_b, const Eigen::Vector3d _init_ba, const Eigen::Vector3d _init_bg);
  ~PreIntergration() = default;
  void getBias(Eigen::Vector3d &_ba, Eigen::Vector3d &_bg) const;
  void setBias(const Eigen::Vector3d _ba, const Eigen::Vector3d _bg);
  void addImuData(const double _time, const Eigen::Vector3d _fb, const Eigen::Vector3d _wib_b);
  void reintegrate();
  int getDataNum() const;
  
  Eigen::Vector3d init_ba_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero();

  Eigen::Matrix<double, EsDim, EsDim> jacobian_ = Eigen::Matrix<double, EsDim, EsDim>::Identity();
  Eigen::Matrix<double, EsDim, EsDim> covariance_ = Eigen::Matrix<double, EsDim, EsDim>::Zero();
  Eigen::Matrix<double, 18, 18> noise_ = Eigen::Matrix<double, 18, 18>::Zero();

  Eigen::Vector3d delta_p_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond delta_q_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d delta_v_ = Eigen::Vector3d::Zero();
  double sum_dt_{0.};
  
  std::vector<double> time_buf;
  std::vector<double> dt_buf;
  std::vector<Eigen::Vector3d> fb_buf;
  std::vector<Eigen::Vector3d> wib_b_buf;
public:
  void integrateOnce(const double _dt, const Eigen::Vector3d _fb, const Eigen::Vector3d _wib_b);
  
  void reset();
  void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian);

  Eigen::Vector3d fb_0_= Eigen::Vector3d::Zero();
  Eigen::Vector3d wib_b_0_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba_0_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_0_ = Eigen::Vector3d::Zero();
public:
  static void setNoiseStd(const Eigen::Vector3d _acc_std, const Eigen::Vector3d _gyro_std, const Eigen::Vector3d _ba_std, const Eigen::Vector3d _bg_std);
  static void resetStatics();
private:
  static Eigen::Vector3d acc_std_;
  static Eigen::Vector3d gyro_std_;
  static Eigen::Vector3d ba_std_;
  static Eigen::Vector3d bg_std_;
  static bool is_std_set_;
};
} // namespace livo_initial

#endif // PRE_INTEGRATION_H