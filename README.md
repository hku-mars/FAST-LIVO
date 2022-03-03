# FAST-LIVO
## Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry

**FAST-LIVO** is a fast LiDAR-Inertial-Visual odometry system, which builds on two tightly-coupled and direct odometry subsystems: a VIO subsystem and a LIO subsystem. The LIO subsystem registers raw points (instead of feature points on e.g., edges or planes) of a new scan to an incrementally-built point cloud map. The map points are additionally attached with image patches, which are then used in the VIO subsystem to align a new image by minimizing the direct photometric errors without extracting any visual features (e.g., ORB or FAST corner features).

**Contributors**: [Chunran Zheng 郑纯然](https://github.com/xuankuzcr)， [Qingyan Zhu 朱清岩](https://github.com/ZQYKAWAYI)， [Wei Xu 徐威](https://github.com/XW-HKU)

<div align="center">
    <img src="img/Framework.svg" width = 90% >
</div>

### Related Paper

Our related paper is now available on **arXiv**:  [FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/abs/2203.00893).

### Related Video
Click below images to open our accompanying video.

<div align="center">
<a href="https://drive.google.com/file/d/1PM8R4JFvYo_PjE4LzdS5WYh-rnj1Ceuw/view?usp=sharing" target="_blank"><img src="img/cover.bmp" alt="video" width="60%" /></a>

</div>


### Codes & Datasets & Our Hard Sychronized Equipment

Our code, private datasets and hard sychronized handheld device would be released asap. Thanks for your patience.

### Acknowledgments
Thanks for [Fast-LIO2](https://github.com/hku-mars/FAST_LIO) and [SVO2.0](https://github.com/uzh-rpg/rpg_svo_pro_open).

Thanks for [Livox Technology](https://www.livoxtech.com/) for equipment support.

