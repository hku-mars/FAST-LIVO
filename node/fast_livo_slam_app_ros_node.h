/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
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

#include "ros_dep.h"
#include "laserMapping.h"
#include "preprocess.h"

namespace robosense {
namespace slam {

class FastLivoSlamApp {
 public:
  FastLivoSlamApp(const std::string cfg_path);

  ~FastLivoSlamApp() {
    Stop();
  }

  void Start() {
    slam_ptr_->Start();
  }

  void Stop() {
    // LINFO<< "FastLivoSlamApp stopping"<<REND;
    // slam_ptr_->Stop(); //FIXME: 会析构两次，不必调用
    // LINFO<< "FastLivoSlamApp stopped"<<REND;
  }

  // 重启应用程序的方法
  void Restart(const std::string cfg_path) {
    LINFO << "Restarting FastLivoSlamApp..." << REND;
    resetStatics();
    
    // 创建新的SLAM实例
    slam_ptr_.reset(new slam::FastLivoSlam());
    slam_ptr_->Init(cfg_path);
    slam_ptr_->RegisterGetLIVOResult(full_result_func_);
    slam_ptr_->SetRGBImageCallback(rgb_img_func_);
    slam_ptr_->SetNoiseImageCallback(noise_img_func_);
    slam_ptr_->SetRawImageCallback(rs_raw_img_func_);
    slam_ptr_->SetAllCloudImageCallback(rs_all_cloud_img_func_);
    slam_ptr_->SetLaserCloudFullResCallback(cloud_register_func_);
    slam_ptr_->SetSubVisualCloudCallback(visual_sub_map_func_);
    slam_ptr_->SetLaserCloudEffectCallback(cloud_effected_func_);
    slam_ptr_->SetLaserCloudMapCallback(laser_map_func_);
    slam_ptr_->SetOdomAftMappedCallback(odom_aft_mapped_func_);
    slam_ptr_->SetPathCallback(path_func_);

    // 重新启动
    slam_ptr_->Start();
    
    LINFO << "FastLivoSlamApp restarted successfully" << REND;
  }

 private:
  std::string name() {
    return "FastLivoSlamApp";
  }
  // module
  YAML::Node cfg_node_;
  std::string cfg_path_;  // 存储配置路径
  shared_ptr<Preprocess> p_pre_;
  slam::FastLivoSlam::Ptr slam_ptr_;

  // input
  void LidarCallback(const PointCloud2MsgsConstPtr msg);
  void ImuCallback(const ImuMsgsConstPtr imu_msg_ptr);
  void ImageCallback(const ImageMsgsConstPtr image_ptr);
//  void RestartSignalCallback(const EmptyMsgsConstPtr &msg);  // 新增的重启信号回调
#ifdef USE_ROS2
//  void LidarZeroCpyCallback(const robosense_msgs::msg::RsPointCloud::ConstSharedPtr msg_ptr) {
//    // static double last_sys_t = 0;
//    // static double last_header_t = 0;
//    double sys_t = GetTimeNow().seconds();
//    double header_ts = HeaderToSec(msg->header);
//
//    // mtx_cb.lock();
//    // f_sensor_buf << std::fixed << "sys: " << sys_t
//    //              << " sys_dif: " << sys_t - last_sys_t << " lid " << header_ts
//    //              << " dif: " << header_ts - last_header_t << std::endl;
//    // mtx_cb.unlock();
//
//    LidarCallback_last_sys_t_ = sys_t;
//    LidarCallback_last_header_t_ = header_ts;
//
//    CloudPtr ptr(new PointCloudXYZI());
//    double cloud_abs_ts;
//
//    double b_t = omp_get_wtime();
//    CloudRosToCommon(msg, ptr, cloud_abs_ts);
//    double e_t = omp_get_wtime();
//    printf("[ INPUT ] preprocess cloud done, header_ts: %.6f cloud_ts: %.6f "
//           "size: %d cost(ms): %f.\n",
//           header_ts, cloud_abs_ts, int(ptr->points.size()), (e_t - b_t) * 1000);
//
//    slam_ptr_->AddData(ptr, cloud_abs_ts);
//  }
//  void ImageZeroCpyCallback(const robosense_msgs::msg::RsImage::ConstSharedPtr image_ptr) {
//    // static double last_sys_t = 0;
//    // static double last_header_t = 0;
//    double sys_t = GetTimeNow().seconds();
//    double header_ts = HeaderToSec(image_ptr->header);
//
//    // mtx_cb.lock();
//    // f_sensor_buf << std::fixed << "sys: " << sys_t
//    //              << " sys_dif: " << sys_t - last_sys_t << "   img " << header_ts
//    //              << " dif: " << header_ts - last_header_t << std::endl;
//    // mtx_cb.unlock();
//
//    ImuCallback_last_sys_t_ = sys_t;
//    ImuCallback_last_header_t_ = header_ts;
//
//    std::shared_ptr<cv::Mat> cv_img_ptr(new cv::Mat);
//    *cv_img_ptr = cv_bridge::toCvCopy(image_ptr, "bgr8")->image;
////    cv_img_ptr->rows = image_ptr->height;
////    cv_img_ptr->cols = image_ptr->width;
////    cv_img_ptr->d
//    slam_ptr_->AddData(cv_img_ptr, header_ts);
//  }
#endif

  // ROS to common
  void CloudRosToCommon(const PointCloud2MsgsConstPtr &msg, CloudPtr &ptr, double& cloud_abs_ts);
  slam::Imu::Ptr ImuRosToCommon(const ImuMsgsConstPtr &imu_msg_ptr);
  
  // ROS subscriber and publisher
#ifdef USE_ROS1
  ros::NodeHandle nh_;
  ros::Subscriber lidar_sub_, imu_sub_, image_sub_;
  ros::Subscriber restart_signal_sub_;  // 添加重启信号订阅器
  image_transport::Publisher img_pub_, noise_img_pub_, raw_img_pub_, rs_all_cloud_img_pub_;
  ros::Publisher pubLaserCloudFullResRGB_, pubLaserCloudFullRes_,
                 pubVisualCloud_, pubSubVisualCloud_, pubLaserCloudEffect_,
                 pubLaserCloudMap_;
  ros::Publisher pubOdomAftMapped_;
  ros::Publisher pubPath_;
  tf::TransformBroadcaster br_;
#elif defined(USE_ROS2)
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
//  rclcpp::Subscription<robosense_msgs::msg::RsPointCloud>::SharedPtr zero_cpy_lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
//  rclcpp::Subscription<robosense_msgs::msg::RsImage>::SharedPtr zero_cpy_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<EmptyMsgs>::SharedPtr restart_signal_sub_;
  image_transport::Publisher img_pub_, noise_img_pub_, raw_img_pub_, rs_all_cloud_img_pub_;
  rclcpp::Publisher<PointCloud2Msgs>::SharedPtr pubLaserCloudFullResRGB_, pubLaserCloudFullRes_,
      pubVisualCloud_, pubSubVisualCloud_, pubLaserCloudEffect_,
      pubLaserCloudMap_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_ptr_;
#endif
  PathMsgs path_;

  std::function<void(const LIVOResult&)> full_result_func_;
  std::function<void(const cv::Mat&)> rgb_img_func_;
  std::function<void(const cv::Mat&)> noise_img_func_;
  std::function<void(const cv::Mat&)> rs_raw_img_func_;
  std::function<void(const cv::Mat&)> rs_all_cloud_img_func_;
  std::function<void(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &msg_ptr)> cloud_register_func_;
  std::function<void(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &msg_ptr)> visual_sub_map_func_;
  std::function<void(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &msg_ptr)> cloud_effected_func_;
  std::function<void(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &msg_ptr)> laser_map_func_;
  std::function<void(const Pose &msg_ptr)> odom_aft_mapped_func_;
  std::function<void(const Pose &msg_ptr)> path_func_;

  // statics in previous version
  double LidarCallback_last_sys_t_ = 0;
  double LidarCallback_last_header_t_ = 0;
  double ImuCallback_last_sys_t_ = 0;
  double ImuCallback_last_header_t_ = 0;
  bool imu_init_done_ = false;
  void resetStatics()
  {
    LidarCallback_last_sys_t_ = 0;
    LidarCallback_last_header_t_ = 0;
    ImuCallback_last_sys_t_ = 0;
    ImuCallback_last_header_t_ = 0;
    imu_init_done_ = false;
    path_ = PathMsgs();
  }

  bool zero_copy_ = false;

};

} // namespace slam
} // namespace robosense
