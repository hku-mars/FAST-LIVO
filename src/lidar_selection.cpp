#include "lidar_selection.h"
// #define PATCH_TEST
namespace lidar_selection {

LidarSelector::LidarSelector(
    const int gridsize, string _loc_dir,
    std::shared_ptr<LidarSelectorResult> _result = nullptr)
    : grid_size(gridsize), log_dir(_loc_dir) {
  downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
  G = Matrix<double, DIM_STATE, DIM_STATE>::Zero();
  H_T_H = Matrix<double, DIM_STATE, DIM_STATE>::Zero();
  Rli = M3D::Identity();
  Rci = M3D::Identity();
  Rcw = M3D::Identity();
  Jdphi_dR = M3D::Identity();
  Jdp_dt = M3D::Identity();
  Jdp_dR = M3D::Identity();
  Pli = V3D::Zero();
  Pci = V3D::Zero();
  Pcw = V3D::Zero();
  width = 800;
  height = 600;

  if (_result != nullptr) {
    result_ = _result;
  } else {
    result_.reset(new LidarSelectorResult(_loc_dir));
  }
}

LidarSelector::~LidarSelector() {
  delete sub_sparse_map;
  delete[] grid_type;
  delete[] map_index;
  delete[] map_value;
  delete[] map_value_max;
  unordered_map<int, Warp *>().swap(Warp_map);
  unordered_map<VOXEL_KEY, float>().swap(sub_feat_map);
  unordered_map<VOXEL_KEY, VOXEL_POINTS *>().swap(feat_map);
}

void LidarSelector::init()
{
    sub_sparse_map = new SubSparseMap;

    Rci = Rcl * Rli;
    Pci = Rcl * Pli + Pcl;
    Jdphi_dR = Rci;
    // Jdp_dR = -Rci * SKEW_SYM_MATRX(-Rci.transpose() * Pci);
    Jdp_dR = -Rci * SKEW_SYM_MATRX(( - Rci.transpose() * Pci));

    width = cam->width();
    height = cam->height();
    LINFO << "camera width: " << width << " height: " << height << REND;
    grid_n_width = static_cast<int>(width/grid_size);
    grid_n_height = static_cast<int>(height/grid_size);
    length = grid_n_width * grid_n_height;
    fx = cam->errorMultiplier2();
    fy = cam->errorMultiplier() / (4. * fx);
    f_mean_ = 0.5*(fx+fy);  // 初始化f_mean_

    grid_type = new int[length];
    map_index = new int[length];
    map_value = new float[length];
    map_value_max = new float[length];
    map_dist = (float*)malloc(sizeof(float)*length);
    memset(grid_type, TYPE_UNKNOWN, sizeof(int)*length);
    memset(map_index, 0, sizeof(int)*length);
    memset(map_value, 0, sizeof(float)*length);
    memset(map_value_max, 0, sizeof(float)*length);
    voxel_points_.reserve(length);
    add_voxel_points_.reserve(length);
    add_voxel_points_normal_.reserve(length);
    add_curvature_.reserve(length);
    weights_.reserve(length);
    patch_size_total = patch_size * patch_size;
    patch_size_half = static_cast<int>(patch_size/2);
    patch_cache.resize(patch_size_total);
    stage_ = STAGE_FIRST_FRAME;
    pg_down.reset(new PointCloudXYZI());
    weight_scale_ = 10;
    weight_function_.reset(new vk::robust_cost::HuberWeightFunction());
    // weight_function_.reset(new vk::robust_cost::TukeyWeightFunction());
}

void LidarSelector::Reset2DGridVec()
{
    memset(grid_type, TYPE_UNKNOWN, sizeof(int)*length);
    memset(map_index, 0, sizeof(int)*length);
    fill_n(map_dist, length, 10000);
    std::vector<PointPtr>(length).swap(voxel_points_);
    std::vector<V3D>(length).swap(add_voxel_points_);
    std::vector<V3D>(length).swap(add_voxel_points_normal_);
    std::vector<float>(length).swap(add_curvature_);
    std::vector<float>(length).swap(weights_);
    voxel_points_.reserve(length);
    add_voxel_points_.reserve(length);
    add_voxel_points_normal_.reserve(length);
    add_curvature_.reserve(length);
    weights_.reserve(length);
}

void LidarSelector::dpi(V3D p, MD(2,3)& J) {
    const double x = p[0];
    const double y = p[1];
    const double z_inv = 1./p[2];
    const double z_inv_2 = z_inv * z_inv;
    J(0,0) = fx * z_inv;
    J(0,1) = 0.0;
    J(0,2) = -fx * x * z_inv_2;
    J(1,0) = 0.0;
    J(1,1) = fy * z_inv;
    J(1,2) = -fy * y * z_inv_2;
}

void LidarSelector::getpatch(cv::Mat img, V2D pc, float* patch_tmp, int level) 
{
    const float u_ref = pc[0];
    const float v_ref = pc[1];
    const int scale =  (1<<level);
    const int scale_m_width = scale * width;
    const int scale_m_width_p_scale = scale_m_width + scale;
    const int patch_size_m_total = patch_size_total*level;
    const int u_ref_i = floorf(pc[0]/scale)*scale; 
    const int v_ref_i = floorf(pc[1]/scale)*scale;
    const float subpix_u_ref = (u_ref-u_ref_i)/scale;
    const float subpix_v_ref = (v_ref-v_ref_i)/scale;
    const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
    const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;
    const int patch_size_half_m_scale = patch_size_half * scale;
    for (int x = 0; x < patch_size; ++x) {
      uint8_t *img_ptr =
          (uint8_t *)img.data +
          (v_ref_i - patch_size_half_m_scale + x * scale) * width +
          (u_ref_i - patch_size_half_m_scale);
      for (int y = 0; y < patch_size; ++y, img_ptr += scale) {
        patch_tmp[patch_size_m_total + x * patch_size + y] =
            w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] +
            w_ref_bl * img_ptr[scale_m_width] +
            w_ref_br * img_ptr[scale_m_width_p_scale];
      }
    }
}

// 更新 feat_map
void LidarSelector::ProjScanToImageAndAddFeatureToMap(cv::Mat img,
                                                      PointCloudXYZI::Ptr pg) {
  // double t0 = omp_get_wtime();
  // Reset2DGridVec(); // 重置2D图像栅格数组
  // double t_b1 = omp_get_wtime() - t0;

  // 1.筛选在图像FOV内的3D激光点, 存入2D栅格数组
  // t0 = omp_get_wtime();
  int in_frame_cnt{0};
  int update_2d_grid_cnt{0};
  for (int i = 0; i < pg->size(); i++) {
    V3D pt(pg->points[i].x, pg->points[i].y, pg->points[i].z);
    V3D normal(pg->points[i].normal_x, pg->points[i].normal_y,
               pg->points[i].normal_z);
    V2D pc(new_frame_->w2c(pt));
    if (new_frame_->cam_->isInFrame(pc.cast<int>(),
                                    (patch_size_half + 1) * 8)) {
      ++in_frame_cnt;
      int index = static_cast<int>(pc[0] / grid_size) * grid_n_height +
                  static_cast<int>(pc[1] / grid_size);
      float cur_value = vk::shiTomasiScore(img, pc[0], pc[1]);

      if (cur_value > map_value_max[index] &&
          (grid_type[index] != TYPE_MAP ||
           map_value_max[index] <= 10)) //! only add in not occupied grid
      {
        map_value_max[index] = cur_value;
        add_voxel_points_[index] = pt;
        add_voxel_points_normal_[index] = normal;
        add_curvature_[index] = pg->points[i].curvature;
        grid_type[index] = TYPE_POINTCLOUD;
        weights_[index] = pg->points[i].intensity; // 优化权重
        ++update_2d_grid_cnt;
      }
    }
  }
  // double t_b2 = omp_get_wtime() - t0;

  // 2.用激光点创建视觉特征点（PointPtr）, 并为其添加观测（FeaturePtr）,
  // 最后将特征点加入全局视觉地图
  //  t0 = omp_get_wtime();
  int added_feat_num{0}, new_grid_num{0};
  for (int i = 0; i < length; i++) {
    if (grid_type[i] == TYPE_POINTCLOUD &&
        (map_value_max[i] >= map_value_thresh)) //! debug
    {
      // 3D激光点
      const V3D &pt = add_voxel_points_[i];
      PointPtr pt_new(new Point(pt));
      pt_new->normal_ = add_voxel_points_normal_[i];
      pt_new->curvature_ = add_curvature_[i];

      // 特征点
      V2D pc(new_frame_->w2c(pt));
      Vector3d f = cam->cam2world(pc);
      FeaturePtr ftr_new(
          new Feature(pc, f, new_frame_->T_f_w_, map_value_max[i], 0));
      ftr_new->img = new_frame_->img_pyr_[0];
      // ftr_new->ImgPyr.resize(5);
      // for(int i=0;i<5;i++) ftr_new->ImgPyr[i] = new_frame_->img_pyr_[i];
#ifndef PATCH_TEST
      ftr_new->id_ = new_frame_->id_;
#endif
      // 为特征点添加观测。 观测属于特征点
      pt_new->addFrameRef(ftr_new);
      pt_new->value = map_value_max[i];
      pt_new->weight_ = weights_[i];
      pt_new->is_valid_ = 1;

      // 过滤特征点
      if (remove_down_pixel > 0) {
        if (pc[1] > remove_down_pixel)
          pt_new->is_valid_ = 0;
      } else {
        if (pc[1] < -remove_down_pixel)
          pt_new->is_valid_ = 0;
      }
      pt_new->pixel_ = pc;
      AddFeaturePointToMap(pt_new, new_grid_num);
      added_feat_num += 1;
    }
  }

  // double t_b3 = omp_get_wtime() - t0;
  // 3.统计特征点
  int total_feat_pt_size{0}, ave_size{0}, min_size{999}, max_size{-1};
  int gird_feat_size;
  if (feat_map.size()) {
    for (const auto &grid : feat_map) {
      gird_feat_size = grid.second->voxel_points.size();
      max_size = gird_feat_size > max_size ? gird_feat_size : max_size;
      min_size = gird_feat_size < min_size ? gird_feat_size : min_size;
      total_feat_pt_size += gird_feat_size;
    }
    ave_size = total_feat_pt_size / feat_map.size();
  }
  LDEBUG << "[ VIO ProjScanToImageAndAddFeatureToMap ] size Lidar scan: "
         << pg->size() << " in_cam_FOV: " << in_frame_cnt
         << " update_2d_grid_cnt: " << update_2d_grid_cnt
         << " add 3D points to map: " << added_feat_num
         << " create new grid: " << new_grid_num
         << " feat_map grid size: " << feat_map.size()
         << " [feat point statistic] total : " << total_feat_pt_size
         << " ave: " << ave_size << " min: " << min_size << " max: " << max_size
         << REND;
  // printf("pg.size: %d \n", pg->size());
  // printf("B1. : %.6lf \n", t_b1);
  // printf("B2. : %.6lf \n", t_b2);
  // printf("B3. : %.6lf \n", t_b3);
}

void LidarSelector::AddFeaturePointToMap(PointPtr pt_new, int& new_grid_num) {
  // 3D点在全局栅格的坐标
  V3D pt_w(pt_new->pos_[0], pt_new->pos_[1], pt_new->pos_[2]);
  float loc_xyz[3];
  for (int j = 0; j < 3; j++) {
    loc_xyz[j] = pt_w[j] / global_map_voxel_size;
    if (loc_xyz[j] < 0) {
      loc_xyz[j] -= 1.0;
    }
  }

  // 存入全局视觉特征栅格地图
  // TODO: round?
  VOXEL_KEY position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                     (int64_t)loc_xyz[2]);
    if(max_layer>=0)
    {
      Eigen::Vector3d pw_0(position.x, position.y, position.z);
      pw_0 *= global_map_voxel_size;
      Eigen::Vector3d delta_pw0 = pt_w - pw_0;
      Eigen::Vector3d delta_pw = delta_pw0*double(layer_step_y)/global_map_voxel_size;
      int id = std::floor(delta_pw.x())*layer_step_x + std::floor(delta_pw.y())*layer_step_y + std::floor(delta_pw.z());
      
      auto iter = feat_map.find(position);
      if(iter != feat_map.end())
      {
        if(iter->second->id_index.count(id)==0)
        {
          iter->second->id_index[id] = iter->second->voxel_points.size();
          iter->second->voxel_points.push_back(pt_new);
          iter->second->count++;
        }
        else
        {
          const int index = iter->second->id_index[id];
          auto &pt_old = iter->second->voxel_points[index];
          if(pt_new->value > pt_old->value)
          {
            iter->second->voxel_points[index] = pt_new;
          }
        }
      }
      else
      {
        VOXEL_POINTS *ot = new VOXEL_POINTS(0);
        ot->id_index[id] = ot->voxel_points.size();
        ot->voxel_points.push_back(pt_new);
        feat_map[position] = ot;
      }
    }
    else
    {
      auto iter = feat_map.find(position);
      if(iter != feat_map.end())
      {
        iter->second->voxel_points.push_back(pt_new);
        iter->second->count++;
      }
      else
      {
        VOXEL_POINTS *ot = new VOXEL_POINTS(0);
        ot->voxel_points.push_back(pt_new);
        feat_map[position] = ot;
      }
    }
}

void LidarSelector::getWarpMatrixAffine(
    const vk::AbstractCamera& cam,
    const Vector2d& px_ref,
    const Vector3d& f_ref,
    const double depth_ref,
    const SE3& T_cur_ref,
    const int level_ref,    // the corresponding pyrimid level of px_ref
    const int pyramid_level,
    const int halfpatch_size,
    Matrix2d& A_cur_ref)
{
  // Compute affine warp matrix A_ref_cur
  const Vector3d xyz_ref(f_ref*depth_ref);
  Vector3d xyz_du_ref(cam.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)*(1<<pyramid_level)));
  Vector3d xyz_dv_ref(cam.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)*(1<<pyramid_level)));
//   Vector3d xyz_du_ref(cam.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)));
//   Vector3d xyz_dv_ref(cam.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)));
  xyz_du_ref *= xyz_ref[2]/xyz_du_ref[2];
  xyz_dv_ref *= xyz_ref[2]/xyz_dv_ref[2];
  const Vector2d px_cur(cam.world2cam(T_cur_ref*(xyz_ref)));
  const Vector2d px_du(cam.world2cam(T_cur_ref*(xyz_du_ref)));
  const Vector2d px_dv(cam.world2cam(T_cur_ref*(xyz_dv_ref)));
  A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
  A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
}

void LidarSelector::getWarpMatrixAffine(
    const vk::AbstractCamera& cam,
    const Eigen::Vector3d _normal_ref,
    const Vector2d& px_ref,
    const Vector3d& f_ref,
    const double depth_ref,
    const SE3& T_cur_ref,
    const int level_ref,    // the corresponding pyrimid level of px_ref
    const int pyramid_level,
    const int halfpatch_size,
    Matrix2d& A_cur_ref)
{
  // std::vector<Eigen::Vector2d> px_out;
  // Compute affine warp matrix A_ref_cur
  const Vector3d xyz_ref(f_ref*depth_ref);
  Vector3d xyz_du_ref(cam.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)*(1<<pyramid_level)));
  Vector3d xyz_dv_ref(cam.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)*(1<<pyramid_level)));

  double z_du = _normal_ref.dot(xyz_ref)/_normal_ref.dot(xyz_du_ref);
  double z_dv = _normal_ref.dot(xyz_ref)/_normal_ref.dot(xyz_dv_ref);

  xyz_du_ref *= z_du;
  xyz_dv_ref *= z_dv;
  const Vector2d px_cur(cam.world2cam(T_cur_ref*(xyz_ref)));
  const Vector2d px_du(cam.world2cam(T_cur_ref*(xyz_du_ref)));
  const Vector2d px_dv(cam.world2cam(T_cur_ref*(xyz_dv_ref)));

  A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
  A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
  // return px_out;
}

void LidarSelector::warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int search_level,
    const int pyramid_level,
    const int halfpatch_size,
    float* patch)
{
  const int patch_size = halfpatch_size*2 ;
  const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
  if(isnan(A_ref_cur(0,0)))
  {
    printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
    return;
  }
//   Perform the warp on a larger patch.
//   float* patch_ptr = patch;
//   const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref) / (1<<pyramid_level);
//   const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
  const int patch_size_total_m_pyramid_level = patch_size_total * pyramid_level;
// #ifdef MP_EN
//     omp_set_num_threads(MP_PROC_NUM);
// #pragma omp parallel for
// #endif
  for (int y=0; y<patch_size; ++y)
  {
    for (int x=0; x<patch_size; ++x)//, ++patch_ptr)
    {
      // P[patch_size_total*level + x*patch_size+y]
      Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
      px_patch *= (1<<search_level);
      px_patch *= (1<<pyramid_level);
      const Vector2f px(A_ref_cur*px_patch + px_ref.cast<float>());
      if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
        patch[patch_size_total_m_pyramid_level + y*patch_size+x] = 0;
      else
        patch[patch_size_total_m_pyramid_level + y*patch_size+x] = (float) vk::interpolateMat_8u(img_ref, px[0], px[1]);
        // patch_vec[patch_size_total_m_pyramid_level + y*patch_size+x] = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]);
                    // }
    }
  }
}

double LidarSelector::NCC(float* ref_patch, float* cur_patch, int patch_size)
{    
    double sum_ref = std::accumulate(ref_patch, ref_patch + patch_size, 0.0);
    double mean_ref =  sum_ref / patch_size;

    double sum_cur = std::accumulate(cur_patch, cur_patch + patch_size, 0.0);
    double mean_curr =  sum_cur / patch_size;

    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (int i = 0; i < patch_size; i++) 
    {
        double n = (ref_patch[i] - mean_ref) * (cur_patch[i] - mean_curr);
        numerator += n;
        demoniator1 += (ref_patch[i] - mean_ref) * (ref_patch[i] - mean_ref);
        demoniator2 += (cur_patch[i] - mean_curr) * (cur_patch[i] - mean_curr);
    }
    return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);
}

int LidarSelector::getBestSearchLevel(
    const Matrix2d& A_cur_ref,
    const int max_level)
{
  // Compute patch level in other image
  int search_level = 0;
  double D = A_cur_ref.determinant();

  while(D > 3.0 && search_level < max_level)
  {
    search_level += 1;
    D *= 0.25;
  }
  return search_level;
}

// 更新 sub_sparse_map
// 将当前帧的稀疏点云投影到图像，从投影点落占据的格子所有 中找到最匹配的图像特征点
// in: feat_map
// out: sub_sparse_map
void LidarSelector::ProjCloudToImageToGetFeature(cv::Mat img, PointCloudXYZI::Ptr pg)
{
    if(feat_map.size()<=0) return;
    double ts0 = omp_get_wtime();

    // TODO: 选视觉特征打分高的点
    // TODO: source点去map中最近邻，筛选map
    // downSizeFilter.setInputCloud(pg);
    // downSizeFilter.filter(*pg_down);
    pg_down = uniformSample<PointType>(pg, vis_down_leaf); 
    result_->downsample_cloud_size = pg_down->size();
    int cloud_size  = pg_down->size();
    
    Reset2DGridVec();

    double t_insert, t_depth, t_position;
    t_insert=t_depth=t_position=0;
    // printf("A0. downsample cloud: %.6lf \n", omp_get_wtime() - ts0);
    // double ts1 = omp_get_wtime();

    // 1. 更新当前帧（点云FOV）视觉栅格地图
    // 遍历世界系点云，投到图像，创建3d voxel 【sub_feat_map】，保留相机前方的深度
    int cloud_occupied_voxel_size{0};
    int loc_xyz[3];
    unordered_map<VOXEL_KEY, float>().swap(sub_feat_map);
    cv::Mat depth_img = cv::Mat::zeros(height, width, CV_32FC1);
    float* it = (float*)depth_img.data;
    for (int i = 0; i < cloud_size; i++) {
      V3D pt_w(pg_down->points[i].x, pg_down->points[i].y,
               pg_down->points[i].z);

      for (int j = 0; j < 3; j++) {
        loc_xyz[j] = floor(pt_w[j] / global_map_voxel_size);
      }
      VOXEL_KEY position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);

      auto iter = sub_feat_map.find(position);
      if (iter == sub_feat_map.end()) {
        if (int(sub_feat_map[position]) != 1) {
          sub_feat_map[position] = 1.0; // critical
          ++cloud_occupied_voxel_size;
        }
      }

      V3D pt_c(new_frame_->w2f(pt_w));
      V2D px;
      if (pt_c[2] > 0) {
        px[0] = fx * pt_c[0] / pt_c[2] + cx;
        px[1] = fy * pt_c[1] / pt_c[2] + cy;

        if (new_frame_->cam_->isInFrame(px.cast<int>(),
                                        (patch_size_half + 1) * 8)) {
          float depth = pt_c[2];
          int col = static_cast<int>(round(px[0]));
          int row = static_cast<int>(round(px[1]));

          // it[width*row+col] = depth; // critical TODO: 保留前景深度？
          if (it[width * row + col] < 1e-4) {
            it[width * row + col] = depth;
          } else {
            it[width * row + col] = std::min(it[width * row + col], depth);
          }
        }
      }
    }
    result_->cloud_occupied_voxel_size = cloud_occupied_voxel_size;

    // // 将深度图归一化到0-255的范围
    // cv::Mat normalizedDepth;
    // cv::normalize(depth_img, normalizedDepth, 0, 255, cv::NORM_MINMAX, CV_8U);
    // cv::imwrite(DEBUG_FILE_DIR("depth_img.jpg"), normalizedDepth);
    // imshow("depth_img", depth_img);
    // printf("A1. update depth img: %.6lf \n", omp_get_wtime() - ts1);
    // printf("A11. calculate pt position: %.6lf \n", t_position);
    // printf("A13. generate depth map: %.6lf \n", t_depth);
    // printf("A. projection: %.6lf \n", omp_get_wtime() - ts0);
    // LDEBUG<<"source cloud occupy cloud_occupied_voxel_size:"<<cloud_occupied_voxel_size<<REND;

    // double t1 = omp_get_wtime();
    // 从地图取特征点
    // 遍历source 3d点，如果落在【feat_map】对应格子，更新格子信息
    memset(map_value, 0, sizeof(float)*length);
    memset(map_value_max, 0, sizeof(float)*length);
    for(auto& iter : sub_feat_map)
    {   
        auto corre_voxel = feat_map.find(iter.first);
        if(corre_voxel != feat_map.end())
        {
            std::vector<PointPtr> &voxel_points = corre_voxel->second->voxel_points;
            int voxel_num = voxel_points.size();
            // 遍历地图点
            for (int i=0; i<voxel_num; i++)
            {
                PointPtr pt = voxel_points[i]; // 包含权重
                // 仅保留相机前方点
                if(pt==nullptr) continue;
                V3D pt_cam(new_frame_->w2f(pt->pos_));
                if(pt_cam[2]<0) continue;

                // 计算投影像素点
                V2D pc(new_frame_->w2c(pt->pos_));

                // 在相机FOV内
                if(new_frame_->cam_->isInFrame(pc.cast<int>(), (patch_size_half+1)*8)) // 20px is the patch size in the matcher
                {
                    int index = static_cast<int>(pc[0]/grid_size)*grid_n_height + static_cast<int>(pc[1]/grid_size);
                    grid_type[index] = TYPE_MAP;
                    Vector3d obs_vec(new_frame_->pos() - pt->pos_);

                    float cur_dist = obs_vec.norm();
                    float cur_value = pt->value;
                    // 记录距离相机pos最近的观测点
                    if (cur_dist <= map_dist[index])   // TODO： 均值？
                    {
                        map_dist[index] = cur_dist;
                        voxel_points_[index] = pt;
                        map_value[index] = cur_value;
                    } 
                    // 记录最大特征值
                    if (cur_value >= map_value_max[index])
                    {
                        map_value_max[index] = cur_value;
                    }
                }
            }    
        } 
    }
        
    // double t2 = omp_get_wtime();

    double t_2, t_3, t_4, t_5;
    t_2=t_3=t_4=t_5=0;

    int light_bad_cnt{0};
    int light_max{0};
    int light_min{255};
    int light_std_max{0};
    int light_std_min{255};
    std::vector<PointPtr> pt_for_vis;
    int error_thresh = outlier_threshold * patch_size_total;

    // patch地图缓存
    // length是图像坐标系格子数
    std::vector<int> search_levels_tmp(length);
    std::vector<float> errors_tmp(length); // 先验误差
    std::vector<int> index_tmp(length);
    std::vector<PointPtr> voxel_points_tmp(length);
    std::vector<vector<float>> patch_tmp(length);
    std::vector<int> is_valid(length, 0);
    unordered_map<int, Warp*>().swap(Warp_map);
    std::map<double, int> value_index_map;

    int total_feat_size{0};
    for (int i=0; i<length; i++) 
    { 
        if (grid_type[i]==TYPE_MAP && map_value_max[i]>=map_value_thresh * 0.5)
        {
            ++total_feat_size;

            // double t_1 = omp_get_wtime();
            PointPtr pt = voxel_points_[i];
            if(pt==nullptr) continue;

            V2D pc(new_frame_->w2c(pt->pos_));
            pt->pixel_ = pc;
            V3D pt_cam(new_frame_->w2f(pt->pos_));
   
            // 跳过深度不连续的点
            bool depth_uncontinous = false;
            for (int u = -patch_size_half; u <= patch_size_half; u++) {
              for (int v = -patch_size_half; v <= patch_size_half; v++) {
                if (u == 0 && v == 0)
                  continue;

                float depth = it[width * (v + static_cast<int>(round(pc[1]))) +
                                 u + static_cast<int>(round(pc[0]))];
                if (depth == 0.)
                  continue;

                double delta_dist = abs(pt_cam[2] - depth);
                if (delta_dist > depth_continuous_thresh) {
                  depth_uncontinous = true;
                  break;
                }
              }
              if (depth_uncontinous)
                break;
            }
            if(depth_uncontinous) continue;
            // t_2 += omp_get_wtime() - t_1;

            // t_1 = omp_get_wtime();
            // 拿到当前source帧视角最接近的观测点
            FeaturePtr ref_ftr;
#ifndef PATCH_TEST
            if(!pt->getCloseViewObs(new_frame_->pos(), ref_ftr, pc)) continue;
#else
            // if(!pt->getBestViewObs(new_frame_->pos(), ref_ftr, pc)) continue;
            if(!pt->getCloseViewObs(new_frame_->pos(), ref_ftr, pc)) continue;
#endif
            // t_3 += omp_get_wtime() - t_1;

            // t_1 = omp_get_wtime();
            std::vector<float> patch_wrap(patch_size_total * 3, 0);
            int search_level;
            Matrix2d A_cur_ref_zero;
            auto iter_warp = Warp_map.find(ref_ftr->id_);
            if(iter_warp != Warp_map.end())
            {
                search_level = iter_warp->second->search_level;
                A_cur_ref_zero = iter_warp->second->A_cur_ref;
            }
            else
            {
#ifndef PATCH_TEST
                getWarpMatrixAffine(*cam, ref_ftr->px, ref_ftr->f, (ref_ftr->pos() - pt->pos_).norm(), 
                new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse(), 0, 0, patch_size_half, A_cur_ref_zero);
#else
                getWarpMatrixAffine(*cam, ref_ftr->T_f_w_.rotation_matrix()*pt->normal_, ref_ftr->px, ref_ftr->f, (ref_ftr->pos() - pt->pos_).norm(), 
                new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse(), 0, 0, patch_size_half, A_cur_ref_zero);
#endif
                search_level = getBestSearchLevel(A_cur_ref_zero, 2);

                Warp *ot = new Warp(search_level, A_cur_ref_zero);
                Warp_map[ref_ftr->id_] = ot;
            }
            // t_4 += omp_get_wtime() - t_1;

            // double t_1 = omp_get_wtime();
            for(int pyramid_level=0; pyramid_level<=2; pyramid_level++)
            {                
                warpAffine(A_cur_ref_zero, ref_ftr->img, ref_ftr->px, ref_ftr->level, search_level, pyramid_level, patch_size_half, patch_wrap.data());
            }
            // t_4 += omp_get_wtime() - t_1;

            // t_1 = omp_get_wtime();
            getpatch(img, pc, patch_cache.data(), 0);
            // t_5 += omp_get_wtime() - t_1;

            if(ncc_en)
            {
                double ncc = NCC(patch_wrap.data(), patch_cache.data(), patch_size_total);
                if(ncc < ncc_thre) continue;
            }

            float error = 0.0;
            for (int ind=0; ind<patch_size_total; ind++) 
            {
                error += (patch_wrap[ind]-patch_cache[ind]) * (patch_wrap[ind]-patch_cache[ind]);
            }
            if(error > error_thresh) continue;

            // 过滤无效视觉特征
            // 丢弃邻域光度协方差过小点
            double light_cov = 0.0;
            double light_mean = 0;
            for (int i = 0; i < patch_size_total; i++) {
              light_mean += patch_wrap[i];
              light_max = light_max > patch_wrap[i] ? light_max : patch_wrap[i];
              light_min = light_min < patch_wrap[i] ? light_min : patch_wrap[i];
            }
            light_mean /= patch_size_total;
            for (int i = 0; i < patch_size_total; i++) {
              light_cov +=
                  (patch_wrap[i] - light_mean) * (patch_wrap[i] - light_mean);
            }
            light_cov /= patch_size_total;

            double std_dev = sqrt(light_cov);
            light_std_max = light_std_max > std_dev ? light_std_max : std_dev;
            light_std_min = light_std_min < std_dev ? light_std_min : std_dev;
            if (light_min_thresh>0&&  std_dev >= 0 && std_dev < light_min_thresh) {
              light_bad_cnt++;
              continue;
            }

            search_levels_tmp[i] = search_level;
            errors_tmp[i] = error;
            index_tmp[i] = i;
            voxel_points_tmp[i] = pt;
            patch_tmp[i] = std::move(patch_wrap);
            is_valid[i] = 1;

            if (value_index_map.find(map_value[i]) == value_index_map.end()) {
              value_index_map[map_value[i]] = i;
            } else {
              value_index_map[map_value[i] + 0.00001] = i;
            }
        }
    }

    // 选取响应值最大的前若干点
    int valid_size = value_index_map.size();

    std::vector<int> selected_index;
    selected_index.reserve(valid_size);
    for (auto it = value_index_map.rbegin(); it != value_index_map.rend();
         ++it) {
      selected_index.push_back(it->second);
      if (selected_index.size() >= patch_num_max)
        break;
    }
    int non_selected_num = valid_size - selected_index.size();

    // 填充patch地图
    int size = selected_index.size();
    deque<PointPtr>().swap(sub_map_cur_frame_);
    sub_sparse_map->reset();
    sub_sparse_map->align_errors.reserve(size);
    sub_sparse_map->propa_errors.reserve(size);
    sub_sparse_map->search_levels.reserve(size);
    sub_sparse_map->errors.reserve(size);
    sub_sparse_map->index.reserve(size);
    sub_sparse_map->voxel_points.reserve(size);
    sub_sparse_map->patch.reserve(size);

    for (auto i : selected_index) {
      sub_map_cur_frame_.push_back(voxel_points_tmp[i]); // for visualization
      sub_sparse_map->align_errors.push_back(errors_tmp[i]);
      sub_sparse_map->propa_errors.push_back(errors_tmp[i]);
      sub_sparse_map->search_levels.push_back(search_levels_tmp[i]);
      sub_sparse_map->errors.push_back(errors_tmp[i]);
      sub_sparse_map->index.push_back(index_tmp[i]);
      sub_sparse_map->voxel_points.push_back(voxel_points_tmp[i]);
      sub_sparse_map->patch.push_back(patch_tmp[i]);
    }

    // double t3 = omp_get_wtime();
    // cout<<"update feat_map: "<<t2-t1<<endl;
    // cout<<"C. addSubSparseMap: "<<t3-t2<<endl;
    // cout<<"depthcontinuous: C1 "<<t_2<<" C2 "<<t_3<<" C3 "<<t_4<<" C4 "<<t_5<<endl;
    // printf("[ VIO ]: choose %d points from sub_sparse_map. light_bad_cnt: %d, "
    //        "max: %d, min: %d, std_max: %d, std_min: %d\n",
    //        int(sub_sparse_map->index.size()), light_bad_cnt, light_max,
    //        light_min, light_std_max, light_std_min);
    LDEBUG << "[ VIO ProjCloudToImageToGetFeature] select patch, total: "
           << total_feat_size << " valid: " << valid_size
           << " non_selected_num: " << non_selected_num
           << " selected num: " << sub_sparse_map->index.size() << REND;

    //  log
    result_->sub_sparse_map_size = sub_sparse_map->index.size();
    result_->light_bad_cnt = light_bad_cnt;
}

float LidarSelector::UpdateState(cv::Mat img, float total_residual, int level) 
{
    computeH = ekf_time = 0;
    int total_points = sub_sparse_map->index.size();
    if (total_points<20) return 0.; // TODO: < 10?
    StatesGroup old_state = (*state);
    
    bool EKF_end = false;
    /* Compute J */
    float error=0.0, last_error=total_residual, last_patch_error=0.0, propa_error=0.0;
    // MatrixXd H;
    bool z_init = true;
    const int H_DIM = total_points * patch_size_total;
    VectorXd z;
    z.resize(H_DIM);
    z.setZero();

    // H.resize(H_DIM, DIM_STATE);
    // H.setZero();
    H_sub.resize(H_DIM, 6);
    H_sub.setZero();

    // 深度调权， 权重归一化
    if (weight2_ref_depth > 0) {
      float weight_2_max{0.};
      for (int i = 0; i < total_points; ++i) {
        PointPtr pt = sub_sparse_map->voxel_points[i];
        if(pt==nullptr) continue;
        if(pt->is_valid_!=1) continue;
        V3D pf = Rcw * pt->pos_ + Pcw;
        pt->weight_2_ = pf.norm() / weight2_ref_depth;
        pt->weight_2_ = pt->weight_2_ > 1.0 ? 1.0 : pt->weight_2_;
        weight_2_max = std::max(weight_2_max, pt->weight_2_);
      }
      for (int i = 0; i < total_points; ++i) {
        PointPtr pt = sub_sparse_map->voxel_points[i];
        pt->weight_2_ /= weight_2_max;
      }
    } else if (weight2_ref_depth < -5) {
      // static const double f_mean = 0.5*(fx+fy);
      std::vector<float> weight_list;
      for (int i = 0; i < total_points; ++i) {
        PointPtr pt = sub_sparse_map->voxel_points[i];
        if(pt==nullptr) continue;
        if(pt->is_valid_!=1) continue;
        V3D pf = Rcw * pt->pos_ + Pcw;

        pt->weight_2_ = pt->curvature_*f_mean_/pf.norm()-patch_size_half/2;
        if(pt->weight_2_<1.) 
        {
          pt->weight_2_ = 1.;
        }
        pt->weight_2_ = 1./pt->weight_2_;
        // weight_list.emplace_back(pt->weight_2_);
        if(pt->curvature_<0.)
        {
          LERROR << "pt->curvature_: " << pt->curvature_ << REND;
          exit(1);
        }
      }
      // std::sort(weight_list.begin(), weight_list.end());
      // LWARNING << "============== weight_list2 ==============" << REND;
      // for(int i=0; i<20; i++)
      // {
      //   std::cout << "i: " << i << ", weight_list: " << weight_list[weight_list.size()*0.05*i] << " ";
      //   if(i%2==1)
      //   {
      //     LWARNING << REND;
      //   }
      // }
      // LWARNING << "i: " << 20 << ", weight_list: " << weight_list.back() << REND;
    } else {
      for (int i = 0; i < total_points; ++i) {
        PointPtr pt = sub_sparse_map->voxel_points[i];
        pt->weight_2_ = 1.;
      }
    }

    std::vector<int> valid_cnt_vec(total_points, 0);
    for (int iteration=0; iteration<NUM_MAX_ITERATIONS; iteration++) 
    {
        img_iter_num = iteration + 1;
        double t1 = omp_get_wtime();
        double count_outlier = 0;

        std::vector<float> errors(total_points,0);
        std::vector<int> meas_(total_points,0);
        error = 0.0;
        propa_error = 0.0;
        n_meas_ =0;
        M3D Rwi(state->rot_end);
        V3D Pwi(state->pos_end);
        Rcw = Rci * Rwi.transpose();
        Pcw = -Rci*Rwi.transpose()*Pwi + Pci;
        Jdp_dt = Rci * Rwi.transpose();

        // 检查越界
        if (level == 2 && iteration == 0) {
          result_->null_cnt = result_->invalid_cnt = result_->out_border_cnt =
              result_->valid_cnt = 0;
          result_->total_points = total_points;
          V3D pf;
          V2D pc;
          for (int i = 0; i < total_points; ++i) {
            PointPtr pt = sub_sparse_map->voxel_points[i];
            if (pt == nullptr) {
              ++result_->null_cnt;
              continue;
            }
            if (pt->is_valid_ != 1) {
              ++result_->invalid_cnt;
              continue;
            }
            pf = Rcw * pt->pos_ + Pcw;
            pc = cam->world2cam(pf);
            if (!cam->isInFrame(pc.cast<int>(), (patch_size_half + 1) * 16)) {
              ++result_->out_border_cnt;
            }
          }
        }

#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i=0; i<total_points; ++i) 
        {
            MD(2,3) Jdpi;
            M3D p_hat;
    
            int search_level = sub_sparse_map->search_levels[i];
            int pyramid_level = level + search_level;
            const int scale =  (1<<pyramid_level);
            
            PointPtr pt = sub_sparse_map->voxel_points[i];
            if(pt==nullptr) continue;
            if(pt->is_valid_!=1) continue;

            V3D pf = Rcw * pt->pos_ + Pcw;
            V2D pc = cam->world2cam(pf);
            if(!cam->isInFrame(pc.cast<int>(),(patch_size_half+1)*scale)) continue;
            // if((level==2 && iteration==0) || (level==1 && iteration==0) || level==0)
            {
                dpi(pf, Jdpi);
                p_hat << SKEW_SYM_MATRX(pf);
            }
            const float u_ref = pc[0];
            const float v_ref = pc[1];
            const int u_ref_i = floorf(pc[0]/scale)*scale; 
            const int v_ref_i = floorf(pc[1]/scale)*scale;
            const float subpix_u_ref = (u_ref-u_ref_i)/scale;
            const float subpix_v_ref = (v_ref-v_ref_i)/scale;
            const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
            const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
            const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
            const float w_ref_br = subpix_u_ref * subpix_v_ref;
            
            const int patch_size_half_m_scale = patch_size_half*scale;
            const int scale_m_width = scale*width;
            const int scale_m_width_p_scale = scale_m_width+scale;
            const int scale_m_width_min_scale = scale_m_width-scale;
            const int scale_m_2 = scale*2;
            const int scale_m_width_2 = scale_m_width*2;

            vector<float> P = sub_sparse_map->patch[i];
            std::vector<std::vector<double>> patch_error_vec(patch_size,std::vector<double>(patch_size,0));
            for (int x=0; x<patch_size; ++x) 
            {
                uint8_t* img_ptr = (uint8_t*) img.data + (v_ref_i+x*scale-patch_size_half_m_scale)*width + u_ref_i-patch_size_half_m_scale;
                for (int y=0; y<patch_size; ++y, img_ptr+=scale) 
                {
                    MD(1,2) Jimg;
                    MD(1,3) Jdphi, Jdp, JdR, Jdt; 
                    // if((level==2 && iteration==0) || (level==1 && iteration==0) || level==0)
                    //{
                    float du = 0.5f * ((w_ref_tl*img_ptr[scale] + w_ref_tr*img_ptr[scale_m_2] + w_ref_bl*img_ptr[scale_m_width_p_scale] + w_ref_br*img_ptr[scale_m_width+scale_m_2])
                                -(w_ref_tl*img_ptr[-scale] + w_ref_tr*img_ptr[0] + w_ref_bl*img_ptr[scale_m_width_min_scale] + w_ref_br*img_ptr[scale_m_width]));
                    float dv = 0.5f * ((w_ref_tl*img_ptr[scale_m_width] + w_ref_tr*img_ptr[scale+scale_m_width] + w_ref_bl*img_ptr[scale_m_width_2] + w_ref_br*img_ptr[scale_m_width_2+scale])
                                -(w_ref_tl*img_ptr[-scale_m_width] + w_ref_tr*img_ptr[-scale_m_width_min_scale] + w_ref_bl*img_ptr[0] + w_ref_br*img_ptr[scale]));
                    Jimg << du, dv;
                    Jimg = Jimg * (1.0/scale) * pt->weight_ * pt->weight_2_;
                    Jdphi = Jimg * Jdpi * p_hat;
                    Jdp = -Jimg * Jdpi;
                    JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR;
                    Jdt = Jdp * Jdp_dt;
                    //}
                    int patch_index = x*patch_size+y;
                    double res = w_ref_tl*img_ptr[0] + w_ref_tr*img_ptr[scale] + w_ref_bl*img_ptr[scale_m_width] + w_ref_br*img_ptr[scale_m_width_p_scale]  - P[patch_size_total*level + patch_index];
                    int index = i*patch_size_total+patch_index;
                    z(index) = res;
                    // float weight = 1.0;
                    // if(iteration > 0)
                    //     weight = weight_function_->value(res/weight_scale_); 
                    // R(index) = weight;       
                    patch_error_vec[x][y] = res*res;
                    meas_[i] = 1;
                    // H.block<1,6>(index,0) << JdR*weight, Jdt*weight;
                    // if((level==2 && iteration==0) || (level==1 && iteration==0) || level==0)
                    H_sub.block<1,6>(index,0) << JdR, Jdt;
                }
            }

            float patch_error = 0.0;
            for (int x = 0; x < patch_size; ++x) {
              for (int y = 0; y < patch_size; ++y) {
                patch_error += patch_error_vec[x][y];
              }
            }
            valid_cnt_vec[i] = 1;
            sub_sparse_map->errors[i] = patch_error;
            errors[i] = patch_error;
        } // end source loop
        patch_num = total_points;
        H_dim = H_DIM;
        computeH += (omp_get_wtime() - t1);

        for (int i = 0; i < total_points; i++) {
          error += errors[i];
          if (meas_[i])
            n_meas_++;
        }
        error = error/n_meas_;

        if (level == 2 && iteration == 0) {
          for (auto i : valid_cnt_vec) {
            if (i) {
              ++result_->valid_cnt;
              ++sub_sparse_map->valid_cnt;
            }
          }
        }

        double t3 = omp_get_wtime();
        if (error <= last_error) 
        {
            old_state = (*state);
            last_error = error;

            // K = (H.transpose() / img_point_cov * H + state->cov.inverse()).inverse() * H.transpose() / img_point_cov;
            // auto vec = (*state_propagat) - (*state);
            // G = K*H;
            // (*state) += (-K*z + vec - G*vec);

            auto &&H_sub_T = H_sub.transpose();
            H_T_H.block<6,6>(0,0) = H_sub_T * H_sub;
            MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state->cov / img_point_cov).inverse()).inverse();
            auto &&HTz = H_sub_T * z;
            // K = K_1.block<DIM_STATE,6>(0,0) * H_sub_T;
            auto vec = (*state_propagat) - (*state);
            G.block<DIM_STATE,6>(0,0) = K_1.block<DIM_STATE,6>(0,0) * H_T_H.block<6,6>(0,0);
            auto solution = - K_1.block<DIM_STATE,6>(0,0) * HTz + vec - G.block<DIM_STATE,6>(0,0) * vec.block<6,1>(0,0);
            (*state) += solution;
            auto &&rot_add = solution.block<3,1>(0,0);
            auto &&t_add   = solution.block<3,1>(3,0);

            if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f))
            {
                EKF_end = true;
            }
        }
        else
        {
            (*state) = old_state;
            EKF_end = true;
        }
        ekf_time += (omp_get_wtime() - t3);
        result_->iteration = iteration;
        result_->ekf_time = ekf_time;

        if (iteration==NUM_MAX_ITERATIONS || EKF_end) 
        {
            break;
        }
    }

    // LDEBUG << "computeH(ms):" << computeH*1000 << " ekf_time(ms):" << ekf_time*1000  <<" H_DIM: "<<H_DIM<< REND;
    return last_error;
} 

void LidarSelector::updateFrameState(StatesGroup state)
{
    M3D Rwi(state.rot_end);
    V3D Pwi(state.pos_end);
    Rcw = Rci * Rwi.transpose();
    Pcw = -Rci*Rwi.transpose()*Pwi + Pci;
    new_frame_->T_f_w_ = SE3(Rcw, Pcw);
}

// 更新观测. 根据相机位置更新特征点的观测, 当观测超过20, 删除视角差最大的
void LidarSelector::addObservation(cv::Mat img)
{
    int total_points = sub_sparse_map->index.size();
    if (total_points==0) return;

    for (int i=0; i<total_points; i++) 
    {
        PointPtr pt = sub_sparse_map->voxel_points[i];
        if(pt==nullptr) continue;
        V2D pc(new_frame_->w2c(pt->pos_));
        SE3 pose_cur = new_frame_->T_f_w_;
        bool add_flag = false;
        // if (sub_sparse_map->errors[i]<= 100*patch_size_total && sub_sparse_map->errors[i]>0)
        {
            //TODO: condition: distance and view_angle 
            // Step 1: time
            FeaturePtr last_feature =  pt->obs_.back();
            // if(new_frame_->id_ >= last_feature->id_ + 20) add_flag = true;

            // Step 2: delta_pose
            SE3 pose_ref = last_feature->T_f_w_;
            SE3 delta_pose = pose_ref * pose_cur.inverse();
            double delta_p = delta_pose.translation().norm();
            double delta_theta = (delta_pose.rotation_matrix().trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (delta_pose.rotation_matrix().trace() - 1));            
            if(delta_p > 0.5 || delta_theta > 10) add_flag = true;

            // Step 3: pixel distance
            Vector2d last_px = last_feature->px;
            double pixel_dist = (pc-last_px).norm();
            if(pixel_dist > 40) add_flag = true;
            
            // Maintain the size of 3D Point observation features.
            if(pt->obs_.size()>=max_obs_num)
            {
                FeaturePtr ref_ftr;
                pt->getFurthestViewObs(new_frame_->pos(), ref_ftr);
                pt->deleteFeatureRef(ref_ftr);
                // ROS_WARN("ref_ftr->id_ is %d", ref_ftr->id_);
            } 
            if(add_flag)
            {
                pt->value = vk::shiTomasiScore(img, pc[0], pc[1]);
                Vector3d f = cam->cam2world(pc);
                FeaturePtr ftr_new(new Feature(pc, f, new_frame_->T_f_w_, map_value_max[i], sub_sparse_map->search_levels[i])); 
                ftr_new->img = new_frame_->img_pyr_[0];
                ftr_new->id_ = new_frame_->id_;
                // ftr_new->ImgPyr.resize(5);
                // for(int i=0;i<5;i++) ftr_new->ImgPyr[i] = new_frame_->img_pyr_[i];
                pt->addFrameRef(ftr_new);      
            }
        }
    }
}

void LidarSelector::FilterFeature(cv::Mat img)
{
    // 图像特征均匀化
    if(uniform_feature <= 0) return;
    int total_points = sub_sparse_map->index.size();
    std::map<V2I, std::map<double, PointPtr>, less_vec<2>> dense_feat_map;
    int length = patch_size_half * uniform_feature;
    for (int i = 0; i < total_points; ++i) {
      PointPtr pt = sub_sparse_map->voxel_points[i];
      V2I pixel_idx{int(pt->pixel_.y() / length), int(pt->pixel_.x() / length)};
      auto &point_map = dense_feat_map[pixel_idx];
      if (point_map.find(pt->value) != point_map.end()) {
        point_map[pt->value + 0.00001] = pt;
      } else {
        point_map[pt->value] = pt;
      }
    }

    int save_cnt = 0;
    for (auto &pair : dense_feat_map) {
      save_cnt = 0;
      auto &point_map = pair.second;
      if (point_map.size() > 1) {
        for (auto it = point_map.rbegin(); it != point_map.rend(); ++it) {
          if (save_cnt < exp_num_per_grid) {
            save_cnt++;
            it->second->is_valid_ = 1;
          } else {
            it->second->is_valid_ = -1;
          }
        }
      } else {
        for (auto it = point_map.rbegin(); it != point_map.rend(); ++it) {
          it->second->is_valid_ = 1;
        }
      }
    }
}

void LidarSelector::ComputeJ(cv::Mat img) 
{
    int total_points = sub_sparse_map->index.size();
    if (total_points==0) return;
    float error = 1e10;
    float now_error = error;

    for (int level=2; level>=0; level--) 
    {
        auto t1 = omp_get_wtime();
        now_error = UpdateState(img, error, level);
        // LDEBUG << "level " << level << " error is " << now_error
        //        << " cost(t):" << omp_get_wtime() - t1 << REND;
    }
    if (now_error < error)
    {
        state->cov -= G*state->cov;
    }
}

void LidarSelector::display_keypatch(double img_time, double time)
{
    // 画栅格
    int l_thickness = 1;
    int step = patch_size_half * 4;
    for (int i = 0; i < img_cp.cols; i += step) {
      cv::line(img_cp, cv::Point(i, 0), cv::Point(i, img_cp.rows),
               cv::Scalar(0, 0, 0), l_thickness);
    }
    for (int i = 0; i < img_cp.rows; i += step) {
      cv::line(img_cp, cv::Point(0, i), cv::Point(img_cp.cols, i),
               cv::Scalar(0, 0, 0), l_thickness);
    }

    // 判断视觉退化
    cv::Mat bin_img(img_cp.size(), CV_8UC1, cv::Scalar(0));
    int max_u{-1},min_u{-1},max_v{255},min_v{255};

    // 梯度
    cv::Mat grad_x, grad_y;
    // double t1 = omp_get_wtime();
    // cv::Sobel(img_cp, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    // cv::Sobel(img_cp, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
    // LERROR<<omp_get_wtime()-t1<<REND;
    double mean_x_grad{0},mean_y_grad{0};

    int total_points = sub_sparse_map->index.size();
    if (total_points==0) return;
    double thresh = sub_sparse_map_error_thresh;
    int low_weight_cnt{0};
    double ave_weight{0};
    // 这部分约5ms
    for(int i=0; i<total_points; i++)
    {
        PointPtr pt = sub_sparse_map->voxel_points[i];
        V2D pc(new_frame_->w2c(pt->pos_));
        cv::Point2f pf;
        pf = cv::Point2f(pc[0], pc[1]); 

        // 
        float error_i = sub_sparse_map->errors[i]/sub_sparse_map->valid_cnt;
        float weight_i = sub_sparse_map->voxel_points[i]->weight_;
        if (weight_i < 1.) {
          low_weight_cnt++;
          ave_weight+=weight_i;
        }
        int radius = weight_i * 6;
        radius = patch_size_half;
        int c_thickness = (weight_i < 1) ? 2 : -1;
        c_thickness = int(img_scaling_ratio * 2 * pt->value / 150);
        c_thickness = c_thickness > int(10*img_scaling_ratio) ? int(10*img_scaling_ratio) : c_thickness;
        c_thickness = c_thickness < 1 ? 1 : c_thickness;
        // 近处点
        if (pt->is_valid_ == 0)
          cv::circle(img_cp, pf, radius, cv::Scalar(255, 255, 255), c_thickness,
                     cv::LINE_AA);
        else if (pt->is_valid_ == -1)
          cv::circle(img_cp, pf, radius, cv::Scalar(128, 255, 255), c_thickness,
                     cv::LINE_AA);
        else
        {
          if (error_i <thresh) // 5.5
              cv::circle(img_cp, pf, radius, cv::Scalar(0, 255, 0), c_thickness, cv::LINE_AA); // Green Sparse Align tracked
          else if (error_i < 2 * thresh)
              cv::circle(img_cp, pf, radius, cv::Scalar(255, 0, 0), c_thickness, cv::LINE_AA); // Blue Sparse Align tracked
          else
              cv::circle(img_cp, pf, radius, cv::Scalar(0, 0, 255), c_thickness, cv::LINE_AA); // Red Sparse Align tracked 
        }
        
        // 梯度
#if 0
        mean_x_grad += grad_x.at<uchar>(pc[1], pc[0]);
        mean_y_grad += grad_y.at<uchar>(pc[1], pc[0]);
#endif
        // source点云溅射（占据）面积
        cv::circle(bin_img, pf, 3* patch_size_half, cv::Scalar(255), -1, cv::LINE_AA);
        // cv::circle(img_cp, pf, 36, cv::Scalar(255), -1, 8);

        // bbox边界
        max_u = max_u >= pc[0] ? max_u : pc[0];
        min_u = min_u <= pc[0] ? min_u : pc[0];
        max_v = max_v >= pc[1] ? max_v : pc[1];
        min_v = min_v <= pc[1] ? min_v : pc[1];
    }

    if(low_weight_cnt)
      ave_weight /= low_weight_cnt;

    sub_sparse_map->valid_cnt = 0;
    // 添加文字
    std::string text;
    cv::Point2f origin;
    int line_type = cv::LINE_AA;
    double scale = 1.2 * img_scaling_ratio;
    int thickness = int(3 * img_scaling_ratio);
    thickness = thickness < 1 ? 1 : thickness;
    int txt_row_step = std::ceil(50 * img_scaling_ratio);
    // 帧率
    if(VIO_freq_ratio > 0)
    {
      text = std::to_string(int(VIO_freq_ratio * 1 / time)) +
             " HZ, iter L=" + std::to_string(lidar_iter_num) +
             " V=" + std::to_string(img_iter_num) +
             ", feat L=" + std::to_string(lidar_feat_num) +
             " V=" + std::to_string(patch_num) + " " + std::to_string(H_dim) + " cam:" + std::to_string(img_time) + " " + std::to_string(img_cp.cols) + "x" + std::to_string(img_cp.rows);
      origin.x = 50 * img_scaling_ratio;
      origin.y = txt_row_step;
      cv::putText(img_cp, text, origin, cv::FONT_HERSHEY_COMPLEX, scale, cv::Scalar(0, 0, 255), thickness, line_type, 0);
    }
    // 权重
    text = std::to_string(low_weight_cnt) + " low weight, ratio=" +
           std::to_string(double(low_weight_cnt) / total_points) +
           ", ave w=" + std::to_string(ave_weight);
    origin.y += txt_row_step;
    cv::putText(img_cp, text, origin, cv::FONT_HERSHEY_COMPLEX, scale, cv::Scalar(255, 255, 255), thickness, line_type, 0);
    // 梯度
    int xg = int(mean_x_grad /= total_points);
    int yg = int(mean_y_grad /= total_points);
    if(int(mean_x_grad /= total_points))
    {
      text = "x_grad=" + std::to_string(int(mean_x_grad /= total_points)) +
            " y_grad=" + std::to_string(int(mean_y_grad /= total_points));
      origin.y += txt_row_step;
      cv::putText(img_cp, text, origin, cv::FONT_HERSHEY_COMPLEX, scale, cv::Scalar(255, 255, 255), thickness, line_type, 0);
    }
    // 内点率
    text = "V score=" + std::to_string(double(cv::countNonZero(bin_img)) / (bin_img.rows * bin_img.cols));
    origin.y += txt_row_step;
    cv::putText(img_cp, text, origin, cv::FONT_HERSHEY_COMPLEX, scale, cv::Scalar(255, 255, 255), thickness, line_type, 0);
    // 打分=source点云占据面积/bbox面积
    text = "V score2=" + std::to_string(double(cv::countNonZero(bin_img)) / ((max_u-min_u) * (max_v-min_v)));
    origin.y += txt_row_step;
    cv::Scalar color = cv::Scalar(255, 255, 255);
    // 打分小于阈值则退化
    if( double(cv::countNonZero(bin_img)) / ((max_u-min_u) * (max_v-min_v)) <vis_degenerate_thresh)
    {
        is_degenerate = true;
        color = cv::Scalar(0, 0, 255);
    }
    cv::putText(img_cp, text, origin, cv::FONT_HERSHEY_COMPLEX, scale, color, thickness, line_type, 0);
    // LIO退化打分
    text = "L score=" + std::to_string(lidar_degenerate_score);
    origin.y += txt_row_step;
    color = cv::Scalar(255, 255, 255);
    if(is_lidar_degenerate)
    {
      color = cv::Scalar(0, 0, 255);
    }
    cv::putText(img_cp, text, origin, cv::FONT_HERSHEY_COMPLEX, scale, color, thickness, line_type, 0);
    // noise
    if(pub_noise_cloud)
    {
      int noise_size = noise_cloud->size();
      for(int i=0; i<noise_size; i++)
      {
          PointType pt = noise_cloud->points[i];
          V2D pc(new_frame_->w2c(V3D{pt.x, pt.y, pt.z}));
          cv::Point2f pf;
          pf = cv::Point2f(pc[0], pc[1]); 
          if (pt.intensity == 1 ) 
              cv::circle(img_noise, pf, 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA); // Red Sparse Align tracked 
          else if (pt.intensity == 2)
              cv::circle(img_noise, pf, 6, cv::Scalar(255, 0, 0), -1, cv::LINE_AA); // Blue Sparse Align tracked
          else
              cv::circle(img_noise, pf, 6, cv::Scalar(0, 255, 0), -1, cv::LINE_AA); // Green Sparse Align tracked
      }
    }
    // all
    // 这部分约10ms
    if (pub_all_cloud) {
      int all_size = all_cloud->size();
      pcl::PointXYZINormal min_pt, max_pt;
      pcl::getMinMax3D (*all_cloud_L, min_pt, max_pt);
      int z_range = depth_color_range;
      for (int i = 0; i < all_size; i++) {
        const auto& pt = all_cloud->points[i];
        V2D pc(new_frame_->w2c(V3D{pt.x, pt.y, pt.z}));
        cv::Point2f pf;
        pf = cv::Point2f(pc[0], pc[1]);

        auto& pt_L = all_cloud_L->points[i];
        int color_index = 255 * ((pt_L.x - min_pt.x) / z_range - 0.1);
        if(color_index > 255) color_index = 255;
        if(color_index < 1) color_index = 1;
        color_index = 255 - color_index;
        cv::circle(img_all_cloud, pf, 5, cv::Scalar(0,255-color_index,color_index), -1, cv::LINE_AA);
      }
    }
    //  weight
}

int detect_cnt{0};
void LidarSelector::detect(double ts, cv::Mat img, PointCloudXYZI::Ptr pg, PointCloudXYZI::Ptr map) 
{
    result_->update_ts = ts;

    double t1, t2, t3, t4, t5, t6;

    if (detect_cnt == 0) {
      LDEBUG << "\n\n\n" << REND;
    }
    LDEBUG<<"[VIO] detect_cnt:"<<++detect_cnt<<" pg size:"<<pg->size() << " map:"<<map->size()<< " stage:"<<int(stage_) << " c:"<<img.cols << " r:"<< img.rows<<REND;
    if(width!=img.cols || height!=img.rows)
    {
        std::cout<<"Resize the img scale !!!"<<std::endl;
        double scale = 0.5;
        cv::resize(img,img,cv::Size(img.cols*scale,img.rows*scale),0,0,cv::INTER_LINEAR);
    }
    img_raw = img.clone();
    img_rgb = img.clone();
    img_cp = img.clone();
    if (pub_noise_cloud)
      img_noise = img.clone();
    if (pub_all_cloud) {
      img_all_cloud = img.clone();
    }
    cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);

    new_frame_.reset(new Frame(cam, img.clone()));
    updateFrameState(*state);

    // 
    PointCloudXYZI::Ptr combine_cloud(new PointCloudXYZI());
    if (map != nullptr && map->size())
      *combine_cloud = *pg + *map;
    else
      *combine_cloud = *pg;
    result_->valid_scan_size = pg->size();
    result_->valid_map_size = map->size();
    result_->valid_combine_cloud_size = combine_cloud->size();

    if(stage_ == STAGE_FIRST_FRAME && pg->size()>10)
    {
        LTITLE<<"==setKeyframe=="<<REND;
        new_frame_->setKeyframe();
        stage_ = STAGE_DEFAULT_FRAME;
    }

    t1 = omp_get_wtime();
    ProjCloudToImageToGetFeature(img, combine_cloud);
    t2 = omp_get_wtime();

    FilterFeature(img);
    if (is_degenerate == false) {
      ComputeJ(img);
      updateFrameState(*state);
      t4 = omp_get_wtime();
    } else {
      is_degenerate = false;
      *state = *state_propagat;
      updateFrameState(*state);
      t4 = omp_get_wtime();
    }

    ProjScanToImageAndAddFeatureToMap(img, pg);
    t3 = omp_get_wtime();

    addObservation(img);
    t5 = omp_get_wtime();

    frame_count ++;
    ave_total = ave_total * (frame_count - 1) / frame_count + (t5 - t1) / frame_count;

    display_keypatch(ts, t5-t1);
    t6 = omp_get_wtime();

    printf("[ VIO detect ]: t(s): ProjCloudToImageToGetFeature: %.3f FilterFeature+ComputeJ: %.3f ProjScanToImageAndAddFeatureToMap: %.3f addObservation: %.3f total time: %.3f display: %.3f ave_total: %.3f.\n"
    , t2-t1, t4-t2, t3-t4, t5-t3, t5 - t1, t6-t5, ave_total);
    result_->ftime_ <<std::fixed<< std::setprecision(6)<< ts<< " " << t2-t1 << " " << t3-t4 << " " << t4-t2 << " " << t5-t3 << " " << t5-t1 << " " << ave_total << std::endl;

    // R_W_C = R_W_I * R_I_C
    M3D Rwi(state->rot_end);
    V3D Pwi(state->pos_end);
    Pose T_W_I(Rwi, Pwi, 0);
    result_->futm_V_opt_I_pose_W_ << std::fixed << std::setprecision(6) << ts << " "
                 << T_W_I.xyz.x() << " " << T_W_I.xyz.y() << " "
                 << T_W_I.xyz.z() << " " << T_W_I.q.x() << " " << T_W_I.q.y()
                 << " " << T_W_I.q.z() << " " << T_W_I.q.w() << "\n";

    Pose T_C_I(Rci, Pci, 0);
    Pose T_I_C = T_C_I.inverse();

    Pose T_W_C = T_W_I;
    T_W_C.updatePoseRight(T_I_C);

    result_->futm_V_opt_Cam_pose_W_ << std::fixed << std::setprecision(6) << ts << " "
                 << T_W_C.xyz.x() << " " << T_W_C.xyz.y() << " "
                 << T_W_C.xyz.z() << " " << T_W_C.q.x() << " " << T_W_C.q.y()
                 << " " << T_W_C.q.z() << " " << T_W_C.q.w() << "\n";
    if(map_sliding_en)
    {
      mapSliding(Pwi);
    }
}

void LidarSelector::mapSliding(Eigen::Vector3d &_center_xyz)
{
  if((_center_xyz - center_last).norm() < sliding_thresh)
  {
    return;
  }

  //get global id now
  center_last = _center_xyz;
  Eigen::Vector3d xyz_min = center_last;
  xyz_min.array() -= half_map_length;
  Eigen::Vector3d xyz_max = center_last;
  xyz_max.array() += half_map_length;
  float loc_xyz_min[3], loc_xyz_max[3];
  for(int i=0; i<3; i++)
  {
    loc_xyz_min[i] = xyz_min[i] / global_map_voxel_size;
    if(loc_xyz_min[i] < 0)
    {
      loc_xyz_min[i] -= 1.0;
    }
    loc_xyz_max[i] = xyz_max[i] / global_map_voxel_size;
    if(loc_xyz_max[i] < 0)
    {
      loc_xyz_max[i] -= 1.0;
    }
  }
  
  // VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);//discrete global
  clearMemOutOfMap((int64_t)loc_xyz_max[0], (int64_t)loc_xyz_min[0],
                    (int64_t)loc_xyz_max[1], (int64_t)loc_xyz_min[1],
                    (int64_t)loc_xyz_max[2], (int64_t)loc_xyz_min[2]);
}

void LidarSelector::clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min )
{
  int delete_voxel_cout = 0;
  // double delete_time = 0;
  // double last_delete_time = 0;
  for (auto it = feat_map.begin(); it != feat_map.end(); )
  {
    const VOXEL_KEY& loc = it->first;
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
    if (should_remove){
      // last_delete_time = omp_get_wtime();
      delete it->second;
      feat_map.erase(it++);
      // delete_time += omp_get_wtime() - last_delete_time;
      delete_voxel_cout++;
    } else {
      ++it;
    }
  }
  std::cout<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels"<<"\n";
  // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}

} // namespace lidar_selection
