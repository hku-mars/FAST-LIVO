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

#include <pcl_conversions/pcl_conversions.h> // 包含 pcl_conversions 头文件
#include <cv_bridge/cv_bridge.h>

#ifdef USE_ROS1 // 在ros1环境下的依赖
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using ImuMsgsConstPtr = sensor_msgs::Imu::ConstPtr;
using ImuMsgs = sensor_msgs::Imu;
using PointCloud2MsgsConstPtr = sensor_msgs::PointCloud2::ConstPtr;
using ImageMsgsConstPtr = sensor_msgs::ImageConstPtr;
using PointCloud2Msgs = sensor_msgs::PointCloud2;
using EmptyMsgs = std_msgs::Empty;
using EmptyMsgsConstPtr = std_msgs::EmptyConstPtr;

using PathMsgs = nav_msgs::Path;
using OdometryMsgs = nav_msgs::Odometry;
using QuaternionMsgs = geometry_msgs::Quaternion;
using PoseStampedMsgs = geometry_msgs::PoseStamped;

using PointCloud2Publisher = ros::Publisher;
using OdometryPublisher = ros::Publisher;
using PathPublisher = ros::Publisher;
using TfBroadcaster = tf::TransformBroadcaster;

using RosRate = ros::Rate;

inline double HeaderToSec(const std_msgs::Header& header) {
  return header.stamp.toSec();
}

inline double HeaderToNanoSec(const std_msgs::Header& header) {
  return header.stamp.toNSec();
}

template<typename... Args>
void RosInfoPrint(const std::string& format, const Args&... args) {
  ROS_INFO(format.c_str(), args...);
}

template<typename... Args>
void RosWarnPrint(const std::string& format, const Args&... args) {
  ROS_WARN(format.c_str(), args...);
}

template<typename... Args>
void RosErrorPrint(const std::string& format, const Args&... args) {
  ROS_ERROR(format.c_str(), args...);
}

inline ros::Time GetTimeNow() { return ros::Time::now(); }

inline double GetTimeNowInSecond() { return ros::Time::now().toSec(); }

inline bool IsRosOK() { return ros::ok(); }

inline void RosSpinOnce() { ros::spinOnce(); }
inline void RosSpin() { ros::spin(); }

inline void RosShutDown() {}

inline double GetQuaternionMsgsX(const QuaternionMsgs &q) { return q.x; }
inline double GetQuaternionMsgsY(const QuaternionMsgs &q) { return q.y; }
inline double GetQuaternionMsgsZ(const QuaternionMsgs &q) { return q.z; }
inline double GetQuaternionMsgsW(const QuaternionMsgs &q) { return q.w; }

inline void SetQuaternionMsgs(double roll, double pitch, double yaw,
                              QuaternionMsgs &q) {
  q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}

#elif defined(USE_ROS2) // 在ros2环境下的依赖
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
//#include "robosense_msgs/msg/rs_image.hpp"
//#include "robosense_msgs/msg/rs_point_cloud.hpp"

using ImuMsgsConstPtr = sensor_msgs::msg::Imu::SharedPtr;
using ImuMsgs = sensor_msgs::msg::Imu;
using PointCloud2MsgsConstPtr = sensor_msgs::msg::PointCloud2::SharedPtr;
using ImageMsgsConstPtr = sensor_msgs::msg::Image::SharedPtr;
using PointCloud2Msgs = sensor_msgs::msg::PointCloud2;
using EmptyMsgs = std_msgs::msg::Empty;
using EmptyMsgsConstPtr = std_msgs::msg::Empty::ConstSharedPtr;

using PathMsgs = nav_msgs::msg::Path;
using OdometryMsgs = nav_msgs::msg::Odometry;
using QuaternionMsgs = tf2::Quaternion;
using PoseStampedMsgs = geometry_msgs::msg::PoseStamped;

using PointCloud2Publisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
using OdometryPublisher = rclcpp::Publisher<nav_msgs::msg::Odometry>;
using PathPublisher = rclcpp::Publisher<nav_msgs::msg::Path>;
using TfBroadcaster = tf2_ros::TransformBroadcaster;

using RosRate = rclcpp::Rate;

inline double HeaderToSec(const std_msgs::msg::Header& header) {
  return rclcpp::Time(header.stamp).seconds();
}

inline double HeaderToNanoSec(const std_msgs::msg::Header& header) {
  return rclcpp::Time(header.stamp).nanoseconds();
}

template<typename... Args>
void RosInfoPrint(const std::string& format, const Args&... args) {
  RCLCPP_INFO(rclcpp::get_logger("global_logger"), format.c_str(), args...);
}

template<typename... Args>
void RosWarnPrint(const std::string& format, const Args&... args) {
  RCLCPP_WARN(rclcpp::get_logger("global_logger"), format.c_str(), args...);
}

template<typename... Args>
void RosErrorPrint(const std::string& format, const Args&... args) {
  RCLCPP_ERROR(rclcpp::get_logger("global_logger"), format.c_str(), args...);
}

inline rclcpp::Time GetTimeNow() { return rclcpp::Clock().now(); }

inline double GetTimeNowInSecond() { return rclcpp::Clock().now().seconds(); }

inline bool IsRosOK() { return rclcpp::ok(); }

rclcpp::Node::SharedPtr ros2_node;

inline void RosSpinOnce() { rclcpp::spin_some(ros2_node); }
inline void RosSpin() { rclcpp::spin(ros2_node); }

inline void RosShutDown() {
  ros2_node.reset();
  rclcpp::shutdown(); // 关闭ROS 2
}

inline double GetQuaternionMsgsX(const QuaternionMsgs &q) { return q.x(); }
inline double GetQuaternionMsgsY(const QuaternionMsgs &q) { return q.y(); }
inline double GetQuaternionMsgsZ(const QuaternionMsgs &q) { return q.z(); }
inline double GetQuaternionMsgsW(const QuaternionMsgs &q) { return q.w(); }

inline void SetQuaternionMsgs(double roll, double pitch, double yaw,
                              QuaternionMsgs &q) {
  q.setRPY(roll, pitch, yaw);
}

#endif // 在ros环境下的依赖