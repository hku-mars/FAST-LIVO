#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H

#include <opencv2/opencv.hpp>
#include "livo_initial/common_data.hpp"
#include "livo_initial/camera.hpp"

namespace livo_initial
{
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);
class FeatureTracker
{
public:
  FeatureTracker() = default;
  ~FeatureTracker() = default;
  void setCameraParam(const int _width, const int _height, const std::vector<double> &_intrinsics, const std::vector<double> &_distortion_coeffs, CameraType _camera_type);
  bool addImage(const double _time, cv::Mat _img);
  ImageInfo getTrackResult();

  static void resetStatics();
private:
  int width_{0};
  int height_{0};
  double fx_{0.};
  double fy_{0.};
  double cx_{0.};
  double cy_{0.};
  std::vector<double> distortion_coeffs_;
  std::shared_ptr<CameraBase> camera_ptr_;

  int pub_count_{1};
  bool first_image_flag_{true};
  double first_image_time_{0.};
  double last_image_time_{0.};
  cv::Mat last_img_bgr_;
  bool init_pub_{false};

  double ROW{0.};
  double COL{0.};
  bool PUB_THIS_FRAME{false};
  bool EQUALIZE{true};
  int FREQ{10};
  double F_THRESHOLD{1.0};
  int MIN_DIST{50};
  int MAX_CNT{250};


  void readImage(const cv::Mat &_img,double _cur_time);
  void setMask();
  void addPoints();
  bool updateID(unsigned int i);
  void rejectWithF();
  void undistortedPoints();

  cv::Mat mask;
  cv::Mat fisheye_mask;
  cv::Mat prev_img, cur_img, forw_img;
  vector<cv::Point2f> n_pts;
  vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
  vector<cv::Point2f> prev_un_pts, cur_un_pts;
  vector<cv::Point2f> pts_velocity;
  vector<int> ids;
  vector<int> track_cnt;
  map<int, cv::Point2f> cur_un_pts_map;
  map<int, cv::Point2f> prev_un_pts_map;
  double cur_time;
  double prev_time;

  static FeatureIdType n_id;
};
}

#endif // FEATURE_TRACKER_H// 设置相机的参数
