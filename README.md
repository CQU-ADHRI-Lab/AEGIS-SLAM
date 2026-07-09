# Robust LiDAR SLAM with Semantic Graph Enhancement

An improved implementation built upon [SG-SLAM](https://github.com/nubot-nudt/SG-SLAM), featuring semantic-geometry arbitration, persistent node augmentation, and descriptor fusion modules to enhance robustness of semantic SLAM under noisy labels.

## Branches

| Branch | Description |
|--------|-------------|
| `master` | Base version |
| `dcc` | DCC parameter configuration with MulRAN dataset support |
| `mislabel` | Mislabel robustness experiments (multi-sequence / multi-ratio comparisons) |
| `ultimate` | **Full version**: frontend arbitration + loop closure enhancement (persistent node + descriptor fusion) |

## 1. Requirements

- **OS**: Ubuntu 20.04
- **ROS**: ROS1 Noetic
- **Compiler**: GCC 9+ / C++17

## 2. Dependencies

The following libraries are already integrated in `src/SG-SLAM/cpp/semgraph_slam/3rdparty/` and require no manual installation:
- [Ceres](https://github.com/ceres-solver/ceres-solver)
- [Eigen](https://github.com/PX4/eigen)
- [Sophus](https://github.com/strasdat/Sophus)
- [tsl_robin](https://github.com/Tessil/robin-map)

Dependencies that need to be installed manually:

```bash
# TBB (threading library)
sudo apt install libtbb-dev

# GTSAM 4.0.3 (pose graph optimization)
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev

# ROS
sudo apt install ros-noetic-desktop-full
```

## 3. Build

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Alternatively, using `catkin build`:

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

## 4. Data Preparation

### 4.1 KITTI

Download point cloud data from the [KITTI website](https://www.cvlibs.net/datasets/kitti/) and semantic labels from the [SemanticKITTI website](http://semantic-kitti.org/). We recommend using predicted labels from [SegNet4D](https://github.com/nubot-nudt/SegNet4D).

Expected directory structure:

```
/data/kitti/sequences/
├── 00/
│   ├── velodyne/       # .bin point cloud files
│   ├── labels/         # .label semantic label files
│   ├── calib.txt       # calibration file
│   └── 00.txt          # ground truth poses (for evaluation)
├── 02/
│   └── ...
└── 08/
    └── ...
```

### 4.2 MulRAN

Download the dataset from the [MulRAN website](https://sites.google.com/view/mulran-pr/home).

## 5. Running

### 5.1 Configure Paths

Before running, modify the data paths in the launch file:

```bash
vim src/SG-SLAM/ros/launch/semgraph_slam_kitti.launch
```

Update the following parameters to your actual paths:
- `lidar_path`: path to point cloud data
- `label_path`: path to semantic labels
- `result_path`: odometry output path
- `pgo_result_path`: SLAM output path

### 5.2 Launch SLAM

```bash
source devel/setup.bash

# KITTI dataset
roslaunch semgraph_slam semgraph_slam_kitti.launch

# MulRAN dataset
roslaunch semgraph_slam semgraph_slam_mulran.launch
```

### 5.3 Key Parameters

Core parameters configurable in the launch file:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `arbitration_enable` | `true` | Enable semantic-geometry arbitration |
| `arbitration_alpha` | `0.5` | Blending coefficient between semantic weight and geometric score |
| `persistent_node_augment_enable` | `true` | Enable persistent node augmentation |
| `descriptor_fusion_enable` | `true` | Enable descriptor fusion |
| `loop_closure_enable` | `true` | Enable loop closure detection |
| `relocalization_enable` | `true` | Enable relocalization |
| `max_range` | `100.0` | Maximum LiDAR sensing range (m) |
| `min_range` | `5.0` | Minimum LiDAR sensing range (m) |
| `voxel_size` | `1.0` | Voxel downsampling size (m) |

## 6. Evaluation

```bash
cd src/SG-SLAM/eval

python traj_eval.py \
  --gt_file /data/kitti/sequences/08/08.txt \
  --pred_file ../ros/save/kitti_slam_08.txt \
  --calib_file /data/kitti/sequences/08/calib.txt
```

## 7. Acknowledgments

This project builds upon the following open-source works:

- [SG-SLAM](https://github.com/nubot-nudt/SG-SLAM): Semantic graph enhanced SLAM framework
- [KISS-ICP](https://github.com/PRBonn/kiss-icp): Point-to-point ICP registration
- [CVC-Cluster](https://github.com/wangx1996/Lidar-Segementation): Curved-voxel clustering

## License

This project is released under the MIT License. See [LICENSE](./src/SG-SLAM/LICENSE) for details.
