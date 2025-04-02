#include "livo_initial/feature_tracker.h"

namespace livo_initial
{
FeatureIdType FeatureTracker::n_id = 0;

void FeatureTracker::resetStatics()
{
  n_id = 0;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void FeatureTracker::setCameraParam(const int _width, const int _height, const std::vector<double> &_intrinsics, const std::vector<double> &_distortion_coeffs, CameraType _camera_type)
{
  ROW = _height;
  COL = _width;
  width_ = _width;
  height_ = _height;
  switch(_camera_type)
  {
    case PINHOLE:
    {
      fx_ = _intrinsics[0];
      fy_ = _intrinsics[1];
      cx_ = _intrinsics[2];
      cy_ = _intrinsics[3];
      distortion_coeffs_ = _distortion_coeffs;
      camera_ptr_ = std::make_shared<PinholeCamera>(_width, _height, _intrinsics, _distortion_coeffs);
      break;
    }
    default:
    {
      std::cout << "undifined camera type " << _camera_type << std::endl;
      exit(0);
      break;
    }
  }
  
}
bool FeatureTracker::addImage(const double _time, cv::Mat _img)
{
  if(first_image_flag_)
  {
    first_image_flag_ = false;
    first_image_time_ = _time;
    last_image_time_ = _time;
    last_img_bgr_ = _img;
    return PUB_THIS_FRAME;
  }
  last_image_time_ = _time;
  last_img_bgr_ = _img;
  // frequency control
  if (round(1.0 * pub_count_ / (_time - first_image_time_)) <= FREQ)
  {
    PUB_THIS_FRAME = true;
    // reset the frequency control
    if (abs(1.0 * pub_count_ / (_time - first_image_time_) - FREQ) < 0.01 * FREQ)
    {
      first_image_time_ = _time;
      pub_count_ = 0;
    }
  }
  else
    PUB_THIS_FRAME = false;
  
  cv::Mat img_gray;
  cv::cvtColor(_img, img_gray, cv::COLOR_BGR2GRAY);
  
  readImage(img_gray, _time);

  for (unsigned int i = 0;; i++)
  {
      bool completed = false;
      completed |= updateID(i);
              
      if (!completed)
          break;
  }

  if(PUB_THIS_FRAME)
  {
    pub_count_++;
    if(!init_pub_)
    {
      PUB_THIS_FRAME = false;
      init_pub_ = 1;
    }
    else
    {
      std::map<FeatureIdType, Eigen::Vector4d> featureId_xyuv;
      for(unsigned int j = 0; j < ids.size(); j++)
      {
        if(track_cnt[j] > 1)
        {
          FeatureIdType p_id = ids[j];
          Eigen::Vector4d xyuv;
          xyuv(0) = cur_un_pts[j].x;
          xyuv(1) = cur_un_pts[j].y;
          xyuv(2) = cur_pts[j].x;
          xyuv(3) = cur_pts[j].y;
          featureId_xyuv.emplace(p_id, xyuv);
        }
      }
    }
  }

  return PUB_THIS_FRAME;
}
ImageInfo FeatureTracker::getTrackResult()
{
  std::map<FeatureIdType, Eigen::Vector4d> featureId_xyuv;
  for(unsigned int j = 0; j < ids.size(); j++)
  {
    if(track_cnt[j] > 1)
    {
      FeatureIdType p_id = ids[j];
      Eigen::Vector4d xyuv;
      xyuv(0) = cur_un_pts[j].x;
      xyuv(1) = cur_un_pts[j].y;
      xyuv(2) = cur_pts[j].x;
      xyuv(3) = cur_pts[j].y;
      featureId_xyuv.emplace(p_id, xyuv);
    }
  }
  ImageInfo img_info_out(last_image_time_, last_img_bgr_, featureId_xyuv);
  return std::move(img_info_out);
}

void FeatureTracker::setMask()
{
  mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

  // prefer to keep features that are tracked for long time
  vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned int i = 0; i < forw_pts.size(); i++)
      cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

  sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
    {
      return a.first > b.first;
    });

  forw_pts.clear();
  ids.clear();
  track_cnt.clear();

  for (auto &it : cnt_pts_id)
  {
    if (mask.at<uchar>(it.second.first) == 255)
    {
      forw_pts.push_back(it.second.first);
      ids.push_back(it.second.second);
      track_cnt.push_back(it.first);
      cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
    }
  }
}

void FeatureTracker::addPoints()
{
  for (auto &p : n_pts)
  {
    forw_pts.push_back(p);
    ids.push_back(-1);
    track_cnt.push_back(1);
  }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
  cv::Mat img;
  cur_time = _cur_time;

  if (EQUALIZE)
  {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(_img, img);
  }
  else
    img = _img;

  if (forw_img.empty())
  {
    prev_img = cur_img = forw_img = img;
  }
  else
  {
    forw_img = img;
  }

  forw_pts.clear();

  if (cur_pts.size() > 0)
  {
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

    for (int i = 0; i < int(forw_pts.size()); i++)
      if (status[i] && !camera_ptr_->pixelInBorder(forw_pts[i].x, forw_pts[i].y))
        status[i] = 0;
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(forw_pts, status);
    reduceVector(ids, status);
    reduceVector(cur_un_pts, status);
    reduceVector(track_cnt, status);
  }

  for (auto &n : track_cnt)
    n++;

  if (PUB_THIS_FRAME)
  {
    rejectWithF();
    setMask();
    cv::resize(mask, mask, forw_img.size());
    int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
    if (n_max_cnt > 0)
    {
      if(mask.empty())
        std::cout << "mask is empty " << std::endl;
      if (mask.type() != CV_8UC1)
        std::cout << "mask type wrong " << std::endl;
      if (mask.size() != forw_img.size())
        std::cout << "wrong size " << std::endl;
      cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
    }
    else
      n_pts.clear();

    addPoints();
  }
  prev_img = cur_img;
  prev_pts = cur_pts;
  prev_un_pts = cur_un_pts;
  cur_img = forw_img;
  cur_pts = forw_pts;
  undistortedPoints();
  prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
  if (forw_pts.size() >= 8)
  {
    std::cout << "FM ransac begins" << std::endl;
    vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
      Eigen::Vector2d tmp_p;
      camera_ptr_->pixelToUnDistNormalized(cur_pts[i].x, cur_pts[i].y, tmp_p.x(), tmp_p.y());
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() + COL / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() + ROW / 2.0;
      un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      camera_ptr_->pixelToUnDistNormalized(forw_pts[i].x, forw_pts[i].y, tmp_p.x(), tmp_p.y());
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() + COL / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() + ROW / 2.0;
      un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
    int size_a = cur_pts.size();
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(forw_pts, status);
    reduceVector(cur_un_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
    std::cout << "FM ransac: " << size_a << " -> " << cur_pts.size() << ": " << 1. * forw_pts.size() / size_a << std::endl;
  }
}

bool FeatureTracker::updateID(unsigned int i)
{
  if (i < ids.size())
  {
    if (ids[i] == -1)
      ids[i] = n_id++;
    return true;
  }
  else
    return false;
}

void FeatureTracker::undistortedPoints()
{
  cur_un_pts.clear();
  cur_un_pts_map.clear();
  //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
  for (unsigned int i = 0; i < cur_pts.size(); i++)
  {
    Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
    Eigen::Vector2d b;
    camera_ptr_->pixelToUnDistNormalized(a.x(), a.y(), b.x(), b.y());
    cur_un_pts.push_back(cv::Point2f(b.x(), b.y()));
    cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x(), b.y())));
  }
  // caculate points velocity
  if (!prev_un_pts_map.empty())
  {
    double dt = cur_time - prev_time;
    pts_velocity.clear();
    for (unsigned int i = 0; i < cur_un_pts.size(); i++)
    {
      if (ids[i] != -1)
      {
        std::map<int, cv::Point2f>::iterator it;
        it = prev_un_pts_map.find(ids[i]);
        if (it != prev_un_pts_map.end())
        {
          double v_x = (cur_un_pts[i].x - it->second.x) / dt;
          double v_y = (cur_un_pts[i].y - it->second.y) / dt;
          pts_velocity.push_back(cv::Point2f(v_x, v_y));
        }
        else
        {
          pts_velocity.push_back(cv::Point2f(0, 0));
        }
      }
      else
      {
        pts_velocity.push_back(cv::Point2f(0, 0));
      }
    }
  }
  else
  {
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
      pts_velocity.push_back(cv::Point2f(0, 0));
    }
  }
  prev_un_pts_map = cur_un_pts_map;
}

}