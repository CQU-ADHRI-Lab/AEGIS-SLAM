# 基于语义图增强的鲁棒 LiDAR SLAM

本仓库是在 [SG-SLAM](https://github.com/nubot-nudt/SG-SLAM) 基础上的改进实现，增加了语义-几何仲裁、持久节点增强、描述子融合等模块，旨在提升语义 SLAM 在标签噪声下的鲁棒性。

## 分支说明

| 分支 | 说明 |
|------|------|
| `master` | 基础版本 |
| `dcc` | DCC 参数配置，支持 MulRAN 数据集 |
| `mislabel` | 误标签鲁棒性实验（含多序列/多比例对比） |
| `ultimate` | **完整版**：前端仲裁 + 回环增强（持久节点 + 描述子融合） |

## 1. 环境要求

- **操作系统**：Ubuntu 20.04
- **ROS**：ROS1 Noetic
- **编译器**：GCC 9+ / C++17

## 2. 依赖安装

以下依赖已集成在 `src/SG-SLAM/cpp/semgraph_slam/3rdparty/` 中，无需手动安装：
- [Ceres](https://github.com/ceres-solver/ceres-solver)
- [Eigen](https://github.com/PX4/eigen)
- [Sophus](https://github.com/strasdat/Sophus)
- [tsl_robin](https://github.com/Tessil/robin-map)

需要手动安装的依赖：

```bash
# TBB（线程并行库）
sudo apt install libtbb-dev

# GTSAM 4.0.3（位姿图优化）
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev

# ROS 相关
sudo apt install ros-noetic-desktop-full
```

## 3. 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果使用 `catkin build`：

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

## 4. 数据准备

### 4.1 KITTI

从 [KITTI 官网](https://www.cvlibs.net/datasets/kitti/) 下载点云数据，从 [SemanticKITTI 官网](http://semantic-kitti.org/) 下载语义标签。推荐使用 [SegNet4D](https://github.com/nubot-nudt/SegNet4D) 的预测标签。

目录结构示例：

```
/data/kitti/sequences/
├── 00/
│   ├── velodyne/       # .bin 点云文件
│   ├── labels/         # .label 语义标签文件
│   ├── calib.txt       # 标定文件
│   └── 00.txt          # 真值位姿（用于评估）
├── 02/
│   └── ...
└── 08/
    └── ...
```

### 4.2 MulRAN

从 [MulRAN 官网](https://sites.google.com/view/mulran-pr/home) 下载数据集。

## 5. 运行

### 5.1 修改路径

运行前需要修改 launch 文件中的数据路径：

```bash
# 编辑 launch 文件
vim src/SG-SLAM/ros/launch/semgraph_slam_kitti.launch
```

修改以下参数为你的实际路径：
- `lidar_path`：点云数据路径
- `label_path`：语义标签路径
- `result_path`：里程计输出路径
- `pgo_result_path`：SLAM 输出路径

### 5.2 启动 SLAM

```bash
source devel/setup.bash

# KITTI 数据集
roslaunch semgraph_slam semgraph_slam_kitti.launch

# MulRAN 数据集
roslaunch semgraph_slam semgraph_slam_mulran.launch
```

### 5.3 关键参数说明

在 launch 文件中可调节的核心参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arbitration_enable` | `true` | 启用语义-几何仲裁 |
| `arbitration_alpha` | `0.5` | 语义权重与几何得分的融合系数 |
| `persistent_node_augment_enable` | `true` | 启用持久节点增强 |
| `descriptor_fusion_enable` | `true` | 启用描述子融合 |
| `loop_closure_enable` | `true` | 启用回环检测 |
| `relocalization_enable` | `true` | 启用重定位 |
| `max_range` | `100.0` | LiDAR 最大感知距离 (m) |
| `min_range` | `5.0` | LiDAR 最小感知距离 (m) |
| `voxel_size` | `1.0` | 体素降采样尺寸 (m) |

## 6. 轨迹评估

```bash
cd src/SG-SLAM/eval

python traj_eval.py \
  --gt_file /data/kitti/sequences/08/08.txt \
  --pred_file ../ros/save/kitti_slam_08.txt \
  --calib_file /data/kitti/sequences/08/calib.txt
```

## 7. 致谢

本项目基于以下开源工作：

- [SG-SLAM](https://github.com/nubot-nudt/SG-SLAM)：语义图增强 SLAM 框架
- [KISS-ICP](https://github.com/PRBonn/kiss-icp)：点到点 ICP 配准
- [CVC-Cluster](https://github.com/wangx1996/Lidar-Segementation)：曲线体素聚类

## 许可证

本项目基于 MIT 许可证发布。详见 [LICENSE](./src/SG-SLAM/LICENSE)。
