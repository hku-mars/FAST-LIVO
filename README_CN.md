# RS-FAST-LIVO

[English Version](README.md) 

## 1. 介绍

本工程将最先进的LiDAR-惯性-视觉里程计系统`FAST-LIVO`适配了速腾聚创激光的新产品Active Camera 第一代 (AC1).

**FAST-LIVO** 是一个快速的LiDAR惯性视觉里程计系统，它建立在两个紧密耦合和直接的里程计子系统之上：VIO子系统和LIO子系统。LIO子系统将新一帧点云（而不是边缘或平面特征点）配准到增量构建的点云地图中。地图点还关联了图像patch，VIO子系统通过最小化patch的直接光度误差来匹配新图像，而无需提取任何视觉特征（例如ORB或FAST角特征）。

<div align="center">
    <img src="img/Framework.svg" width = 100% >
</div>

如果您需要有关`FAST-LIVO`算法的更多信息，可以参考官方仓库和维护者：
- **仓库**: <https://github.com/hku-mars/FAST-LIVO>
- **维护者**: [Chunran Zheng 郑纯然](https://github.com/xuankuzcr)， [Qingyan Zhu 朱清岩](https://github.com/ZQYKAWAYI)， [Wei Xu 徐威](https://github.com/XW-HKU)

## 2. 样例

本节展示了使用AC1在室内和室外建图的效果。ROS2的demo数据可以在 [wiki页面](https://robosense-wiki-cn.readthedocs.io/zh-cn/latest/index.html)下载。

<div align="center">   
    <img src="img/hitsz.png" alt="mesh" /> 
    <p style="margin-top: 2px;">"HIT SZ Wall" 数据集. 左图: 原始图像, 右图: 建图结果</p>
</div>

<div align="center">   
    <img src="img/hitsz_full.png" alt="mesh" /> 
    <p style="margin-top: 2px;">"HIT SZ Wall" 数据集. 完整建图结果</p>
</div>

<div align="center">   
    <img src="img/robosense_logo.png" alt="mesh" /> 
    <p style="margin-top: 2px;"> "indoor robosense logo" 数据集. 左图: 原始图像, 右图: 完整建图结果 </p>
</div>

## 3. 依赖

### 3.1 ROS (ROS 和 ROS2)

根据您的操作系统选择 [官方教程](https://fishros.org/doc/ros2/humble/Installation.html) 中的指定内容进行执行

### 3.2 Sophus

Sophus 需要安装 non-templated/double-only 版本.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

## 4. 安装编译

### ROS1

将本工程代码拉取到一个`ros1`工作空间中，然后在终端中执行以下命令编译安装：

```bash
cd <your workspace>
catkin_make
```

### ROS2


将本工程代码拉取到一个`ros2`工作空间中，然后在终端中执行以下命令编译安装：

```bash
cd <your workspace>
colcon build --symlink-install 
```


## 4. 运行

### 4.1 重要参数

编辑 `config/RS_META.yaml` 来设置一些重要参数：

#### 4.1.1 算法

- `lid_topic`: LiDAR 话题
- `imu_topic`: IMU 话题
- `img_topic`: camera 话题
- `img_enable`: 开启 vio 子系统
- `lidar_enable`: 开启 lio 子系统
- `outlier_threshold`: 单个像素的光度误差（平方）的异常阈值。建议暗场景使用“50~250”，亮场景使用“500~1000”。该值越小，vio子模块的速度越快，但抗退化力越弱。
- `img_point_cov`: 每像素光度误差的协方差。
- `laser_point_cov`: 每个点的点对平面重新方差。 
- `filter_size_surf`：对新扫描中的点进行降采样。建议室内场景为`0.05~0.15`，室外场景为`0.3~0.5`。
- `filter_size_map`：对LiDAR全局地图中的点进行降采样。建议室内场景为`0.15~0.3`，室外场景为`0.4~0.5`。
- `pcd_save_en`：如果为`true`，则将点云保存到pcd文件夹。如果`img_enable`为`1`，则保存RGB彩色点；如果`img_enable`是`0`，则按点云强度存储彩色点。
- `delta_time`：相机和激光雷达之间的时间偏移，用于校正时间戳错位。

在此之后，您可以直接在数据集上运行**RS-FAST-LIVO**。

#### 4.1.2 外参、内参

AC1的传感器外参和内参在```config/calibration.yaml```配置。一般情况下您不需要修改这些参数。

### 4.2 运行

#### ROS

```bash
source devel/setup.bash
roslaunch slam mapping_meta.launch
```

#### ROS2

```bash
source install/setup.bash
ros2 run slam slam_node
```

## 5. 致谢

感谢 [FAST-LIVO](https://github.com/hku-mars/FAST-LIVO)、 [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) 和 [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)

## 6. License

该仓库在 [**GPLv2**](http://www.gnu.org/licenses/) 协议下开源