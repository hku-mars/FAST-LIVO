# 介绍
## hku_data_config
存放官方数据集对应配置

## rs_data_config
存放RS自采META数据集对应配置

- RS_META.yaml : 传感器外参，算法配置
- camera_pinhole_rs.yaml : 相机内参

- 命名规则： 数据日期_序号_配置日期, 例： 1115_hitsz_1210
- 数据集: /media/sti/deeplearningoflidar/NewBee/rumble_liu/robot/data
- 运行结果: 见飞书文档https://robosense.feishu.cn/docx/XUiTdNwJeoc3KfxDaYDcdVaundb

### quick start
#### 运行
以运行哈工大外墙数据为例

```
roslaunch fast_livo mapping_meta.launch
rosbag play /media/sti/deeplearningoflidar/NewBee/rumble_liu/robot/data/2024-11-15/hitsz_giraff.bag
```
#### 查看结果
- PCD
  - rgb_scan_all.pcd : 所有帧累计的RGB点云
  - intensity_sacn_all.pcd: LIO点云 （只在关闭VIO时生成）
- Log
  - t_LIO.txt : LIO耗时
  - t_VIO.txt : VIO耗时
  - utm_LV_opt_I_pose_W.txt: utm格式世界系LIO和VIO优化后IMU位姿
  - utm_V_opt_Cam_pose_W.txt: utm格式相机系VIO优化后Cam位姿
  - utm_V_opt_I_pose_W.txt: utm格式相机系VIO优化后IMU位姿
  - utm_L_opt_I_pose_W.txt: utm格式世界系LIO优化后IMU位姿
  - sensor_buffer.txt: 传感器数据缓存情况
