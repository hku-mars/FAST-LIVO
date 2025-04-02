#ifndef LIDAR_SELECTION_H_
#define LIDAR_SELECTION_H_

#include <common_lib.h>
#include <vikit/abstract_camera.h>
#include <frame.h>
#include <map.h>
#include <feature.h>
#include <point.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <pcl/common/common.h>

namespace lidar_selection {

typedef std::shared_ptr<Point> PointPtr;

class VOXEL_POINTS {
public:
  std::vector<PointPtr> voxel_points;
  int count;
  std::map<int, int> id_index;
  // bool is_visited;

  VOXEL_POINTS(int num) : count(num) {}
};

struct SubSparseMap {
  vector<float> align_errors;
  vector<float> propa_errors; // 先验误差
  vector<float> errors;       // 先验误差
  vector<int> index;
  vector<vector<float>> patch;
  vector<int> search_levels;
  vector<PointPtr> voxel_points;
  int valid_cnt{0};

  SubSparseMap() {
    this->propa_errors.reserve(500);
    this->search_levels.reserve(500);
    this->errors.reserve(500);
    this->index.reserve(500);
    this->patch.reserve(500);
    this->voxel_points.reserve(500);
  };

  void reset() {
    this->propa_errors.clear();
    this->search_levels.clear();
    this->errors.clear();
    this->index.clear();
    this->patch.clear();
    this->voxel_points.clear();
  }
};

class Warp {
public:
  Matrix2d A_cur_ref;
  int search_level;
  // bool is_visited;

  Warp(int level, Matrix2d warp_matrix)
      : search_level(level), A_cur_ref(warp_matrix) {}
};

struct LidarSelectorParams {

};

struct LidarSelectorResult {
  LidarSelectorResult(const std::string _log_dir) { 
    log_dir = _log_dir; 
    log_file_.open(log_dir + "/vio_detect_log.txt");
    ftime_.open(log_dir + "/t_VIO.txt", std::ios::out);
    futm_V_opt_Cam_pose_W_.open(log_dir + "/utm_V_opt_Cam_pose_W.txt", std::ios::out);
    futm_V_opt_I_pose_W_.open(log_dir + "/utm_V_opt_I_pose_W.txt", std::ios::out);
  }
  ~LidarSelectorResult() {
    if(log_file_.is_open()) {
      log_file_.close();
    }
    if(ftime_.is_open()) {
      ftime_.close();
    }
    if(futm_V_opt_Cam_pose_W_.is_open()) {
      futm_V_opt_Cam_pose_W_.close();
    }
    if(futm_V_opt_I_pose_W_.is_open()) {
      futm_V_opt_I_pose_W_.close();
    }
  }
  std::string log_dir = "";
  double update_ts = -1;
  std::ofstream log_file_;
  std::ofstream ftime_;  // 从static改为成员变量
  std::ofstream futm_V_opt_Cam_pose_W_;
  std::ofstream futm_V_opt_I_pose_W_;

  // combine_cloud
  int valid_scan_size = -1;
  int valid_map_size = -1;
  int valid_combine_cloud_size = -1;
  // ProjCloudToImageToGetFeature
  int downsample_cloud_size = -1;
  int cloud_occupied_voxel_size = -1; 
  int sub_sparse_map_size = -1;
  int light_bad_cnt = -1;
  // FilterFeature

  // ComputeJ
  int total_points = -1;
  int null_cnt = -1;
  int invalid_cnt = -1;
  int out_border_cnt = -1;
  int valid_cnt = -1;
  int iteration = -1;
  VD(DIM_STATE) solution;
  double ekf_time = -1;

  // updateFrameState

  // ProjScanToImageAndAddFeatureToMap

  // addObservation

  void Print() {
    LDEBUG << "==============LidarSelectorResult==============" << REND;
    LDEBUG << "update_ts: " << fixed << update_ts << REND;
    LDEBUG << "---------------combine_cloud---------------" << REND;
    LDEBUG << "valid_scan_size: " << valid_scan_size << REND;
    LDEBUG << "valid_map_size: " << valid_map_size << REND;
    LDEBUG << "valid_combine_cloud_size: " << valid_combine_cloud_size << REND;
    LDEBUG << "---------------ProjCloudToImageToGetFeature---------------" << REND;
    LDEBUG << "downsample_cloud_size: " << downsample_cloud_size << REND;
    LDEBUG << "cloud_occupied_voxel_size: " << cloud_occupied_voxel_size << REND;
    LDEBUG << "sub_sparse_map_size: " << sub_sparse_map_size << REND;
    LDEBUG << "light_bad_cnt: " << light_bad_cnt << REND;
    LDEBUG << "---------------ComputeJ---------------" << REND;
    LDEBUG << "total_points: " << total_points << " null_cnt: " << null_cnt
           << " invalid_cnt: " << invalid_cnt
           << " out_border_cnt: " << out_border_cnt
           << " valid_cnt: " << valid_cnt << REND;
    LDEBUG << "iteration: " << iteration << REND;
    LDEBUG << "solution: " << solution.transpose() << REND;
    LDEBUG << "ekf_time: " << ekf_time << REND;
  }
  void SaveLog(){
    if (!log_file_.is_open()) {
      log_file_.open(log_dir + "/vio_detect_log.txt");
    }
    log_file_ << "==============LidarSelectorResult==============" << endl;
    log_file_ << "update_ts: " << fixed << update_ts << endl;
    log_file_ << "---------------combine_cloud---------------" << endl;
    log_file_ << "valid_scan_size: " << valid_scan_size << endl;
    log_file_ << "valid_map_size: " << valid_map_size << endl;
    log_file_ << "valid_combine_cloud_size: " << valid_combine_cloud_size << endl;
    log_file_ << "---------------ProjCloudToImageToGetFeature---------------" << endl;
    log_file_ << "downsample_cloud_size: " << downsample_cloud_size << endl;
    log_file_ << "cloud_occupied_voxel_size: " << cloud_occupied_voxel_size << endl;
    log_file_ << "sub_sparse_map_size: " << sub_sparse_map_size << endl;
    log_file_ << "light_bad_cnt: " << light_bad_cnt << endl;
    log_file_ << "---------------ComputeJ---------------" << endl;
    log_file_ << "total_points: " << total_points << " null_cnt: " << null_cnt
              << " invalid_cnt: " << invalid_cnt
              << " out_border_cnt: " << out_border_cnt
              << " valid_cnt: " << valid_cnt << endl;
    log_file_ << "iteration: " << iteration << endl;
    log_file_ << "solution: " << solution.transpose() << endl;
    log_file_ << "ekf_time: " << ekf_time << endl;
  }
};

class LidarSelector {
  public:
    LidarSelector(const int grid_size, string _loc_dir,
                  std::shared_ptr<LidarSelectorResult> _result);
    ~LidarSelector();
    void SetTIL(const V3D &t, const M3D &R) {
      Pli = -R.transpose() * t;
      Rli = R.transpose();
    }
    void SetTCL(const Eigen::Matrix4d &T_C_L) {
      Rcl = T_C_L.block<3, 3>(0, 0);
      Pcl = T_C_L.block<3, 1>(0, 3);
    }
    void init();
    void detect(double ts, cv::Mat img, CloudPtr pg, CloudPtr map);
    V3F getpixel(cv::Mat img, V2D pc);

    /// RS
    std::shared_ptr<LidarSelectorResult> result_; 
    std::string log_dir;
    double light_min_thresh;
    double sub_sparse_map_error_thresh;
    double depth_continuous_thresh;
    double global_map_voxel_size;
    int max_layer;
    int layer_step_x, layer_step_y;
    int max_obs_num;
    bool map_sliding_en{false};
    double half_map_length;
    double sliding_thresh;
    V3D center_last = Eigen::Vector3d::Zero();
    double vis_down_leaf;
    double VIO_freq_ratio;
    CloudPtr noise_cloud;
    CloudPtr all_cloud;
    CloudPtr all_cloud_L;
    bool pub_noise_cloud, pub_all_cloud;
    double depth_color_range;
    // 退化
    double vis_degenerate_thresh;
    bool is_degenerate{false};
    double lidar_degenerate_score;
    double lidar_degenerate_thresh;
    bool is_lidar_degenerate;
    // 图像过滤
    int map_value_thresh;
    int remove_down_pixel;
    double uniform_feature;
    int exp_num_per_grid;
    int patch_num_max;
    // 
    double weight2_ref_depth;
    // 
    int lidar_iter_num, img_iter_num;
    int lidar_feat_num, patch_num, H_dim;
    // 
    double img_scaling_ratio;
    CloudPtr VIO_map;

    /// official
    int grid_size;
    vk::AbstractCamera* cam;
    double fx,fy,cx,cy;
    int width, height; // 像素
    int grid_n_width, grid_n_height, length; // 2D视觉地图栅格数

    // 外参
    M3D Rli, Rci, Rcw, Rcl; // Rli means trans lidar frame to imu frame
    V3D Pli, Pci, Pcw, Pcl;

    // 地图
    unordered_map<VOXEL_KEY, VOXEL_POINTS*> feat_map; // 全局视觉特征点栅格地图
    unordered_map<VOXEL_KEY, float> sub_feat_map; // 当前帧（点云FOV）视觉栅格地图
    SubSparseMap* sub_sparse_map; // 当前帧patch地图
    unordered_map<int, Warp*> Warp_map; // reference frame id, A_cur_ref and search_level

    // 配置
    pcl::VoxelGrid<PointType> downSizeFilter;
    CloudPtr pg_down; 

    // ProjScanToImageAndAddFeatureToMap
    int* grid_type;
    int* map_index;
    float* map_dist;
    float* map_value;
    float* map_value_max;
    float* patch_with_border_;
    vector<PointPtr> voxel_points_;
    vector<V3D> add_voxel_points_;
    vector<V3D> add_voxel_points_normal_;
    std::vector<float> add_curvature_;
    vector<float> weights_;
    FramePtr new_frame_;
    deque<PointPtr> sub_map_cur_frame_; // 视觉地图 用于可视化

    // 视觉特征过滤
    bool ncc_en;
    vector<float> patch_cache;
    double outlier_threshold, ncc_thre;

    // display_keypatch
    cv::Mat img_raw, img_cp, img_rgb, img_noise, img_cluster, img_all_cloud;

    // ComputeJ 优化
    StatesGroup *state;
    StatesGroup *state_propagat;
    M3D Jdphi_dR, Jdp_dt, Jdp_dR;

    double f_mean_;  // 用于替代UpdateState中的static变量
    int debug, patch_size, patch_size_total, patch_size_half;
    int NUM_MAX_ITERATIONS;
    vk::robust_cost::WeightFunctionPtr weight_function_;
    float weight_scale_;
    double img_point_cov;
    size_t n_meas_;                //!< Number of measurements
    double computeH, ekf_time;
    double ave_total = 0.0;
    int frame_count = 0;

    Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
    MatrixXd H_sub, K;
    
  private:
    void ProjCloudToImageToGetFeature(cv::Mat img, CloudPtr pg);
    void ProjScanToImageAndAddFeatureToMap(cv::Mat img, CloudPtr pg);
    void getpatch(cv::Mat img, V3D pg, float* patch_tmp, int level);
    void getpatch(cv::Mat img, V2D pc, float* patch_tmp, int level);
    void dpi(V3D p, MD(2,3)& J);
    float UpdateState(cv::Mat img, float total_residual, int level);
    double NCC(float* ref_patch, float* cur_patch, int patch_size);

    void FilterFeature(cv::Mat img);
    void ComputeJ(cv::Mat img);
    void Reset2DGridVec();
    void addObservation(cv::Mat img);
    void reset();
    void getWarpMatrixAffine(
        const vk::AbstractCamera &cam, const Vector2d &px_ref,
        const Vector3d &f_ref, const double depth_ref, const SE3 &T_cur_ref,
        const int level_ref, // px_ref对应特征点的金字塔层级
        const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
    void getWarpMatrixAffine(
        const vk::AbstractCamera &cam, const Eigen::Vector3d _normal_ref,
        const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref,
        const SE3 &T_cur_ref,
        const int level_ref, // the corresponding pyrimid level of px_ref
        const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
    void AddFeaturePointToMap(PointPtr pt_new, int& new_grid_num);
    int getBestSearchLevel(const Matrix2d& A_cur_ref, const int max_level);
    void display_keypatch(double img_time, double time);
    void ImgCluster();
    void updateFrameState(StatesGroup state);

    void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref,
                    const Vector2d &px_ref, const int level_ref,
                    const int search_level, const int pyramid_level,
                    const int halfpatch_size, float *patch);
    void mapSliding(Eigen::Vector3d &_center_xyz);
    void clearMemOutOfMap(const int &x_max, const int &x_min, const int &y_max,
                          const int &y_min, const int &z_max, const int &z_min);

    enum Stage {
      STAGE_FIRST_FRAME,
      STAGE_DEFAULT_FRAME
    };
    Stage stage_;
    enum CellType {
      TYPE_MAP = 1,
      TYPE_POINTCLOUD,
      TYPE_UNKNOWN
    };
};

typedef boost::shared_ptr<LidarSelector> LidarSelectorPtr;

inline V3F LidarSelector::getpixel(cv::Mat img, V2D pc) 
{
    auto tmp = pc.cast<int>();
    const float u_ref = pc[0];
    const float v_ref = pc[1];
//    const int u_ref_i = floorf(pc[0]);
    const int u_ref_i = tmp[0];
//    const int v_ref_i = floorf(pc[1]);
    const int v_ref_i = tmp[1];
    const float subpix_u_ref = (u_ref-u_ref_i);
    const float subpix_v_ref = (v_ref-v_ref_i);
    const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
    const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;
    uint8_t* img_ptr = (uint8_t*) img.data + ((v_ref_i)*width + (u_ref_i))*3;
    float B = img_ptr[0];
    float G = img_ptr[1];
    float R = img_ptr[2];
    if (v_ref_i > 0 && v_ref_i < (height - 1) && u_ref_i > 0 && u_ref_i < (width - 1)) {
      B = w_ref_tl*img_ptr[0] + w_ref_tr*img_ptr[0+3] + w_ref_bl*img_ptr[width*3] + w_ref_br*img_ptr[width*3+0+3];
      G = w_ref_tl*img_ptr[1] + w_ref_tr*img_ptr[1+3] + w_ref_bl*img_ptr[1+width*3] + w_ref_br*img_ptr[width*3+1+3];
      R = w_ref_tl*img_ptr[2] + w_ref_tr*img_ptr[2+3] + w_ref_bl*img_ptr[2+width*3] + w_ref_br*img_ptr[width*3+2+3];
    }
//    float B = w_ref_tl*img_ptr[0] + w_ref_tr*img_ptr[0+3] + w_ref_bl*img_ptr[width*3] + w_ref_br*img_ptr[width*3+0+3];
//    float G = w_ref_tl*img_ptr[1] + w_ref_tr*img_ptr[1+3] + w_ref_bl*img_ptr[1+width*3] + w_ref_br*img_ptr[width*3+1+3];
//    float R = w_ref_tl*img_ptr[2] + w_ref_tr*img_ptr[2+3] + w_ref_bl*img_ptr[2+width*3] + w_ref_br*img_ptr[width*3+2+3];
    V3F pixel(B,G,R);
    return pixel;
}


} // namespace lidar_detection

#endif // LIDAR_SELECTION_H_