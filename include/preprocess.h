#pragma once
#include "common/common.h"

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

enum LID_TYPE{RS_META = 0, AVIA = 1, VELO16, OUST64, XT32}; //{1, 2, 3}
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype
{
  double range;
  double dista; 
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

class Preprocess
{
public:
  Preprocess(const std::string cfg_path);
  ~Preprocess();
  void oust64_handler(const pcl::PointCloud<ouster_ros::Point> &msg);
  void velodyne_handler(pcl::PointCloud<robosense::Point> &msg,double ts);
  void xt32_handler(const pcl::PointCloud<xt32_ros::Point> &msg);

  CloudPtr pcl_out;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;
  double blind, blind_max;
  bool feature_enabled,given_offset_time;

private:
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
