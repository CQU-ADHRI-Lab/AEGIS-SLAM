// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>


#pragma once

#include <Eigen/Core>
#include <tuple>
#include <vector>

// SG-SLAM
#include "semgraph_slam/frontend/Deskew.hpp"
#include "semgraph_slam/frontend/Threshold.hpp"
#include "semgraph_slam/frontend/VoxelHashMap.hpp"
#include "semgraph_slam/frontend/SemanticCluster.hpp"
#include "semgraph_slam/frontend/SemGraph.hpp"
#include "semgraph_slam/core/Coreutils.h"
#include "semgraph_slam/frontend/GraphMap.hpp"


namespace graph_slam{



struct SemGraphSLAMConfig {
    // map params
    double voxel_size = 1.0;
    double voxel_size_cluster = 0.2;
    double max_range = 100.0;
    double min_range = 5.0;
    int max_points_per_voxel = 20;

    // th parms
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;

    double model_deviation_trans = 0.1; // translation deviation of constant velocity model
    double model_deviation_rot = 0.01; // rotation deviation of constant velocity model

    double inlier_rate_th = 0.4; // inlier rate threshold for relocalzation

    // Motion compensation
    bool deskew = false;

    // Relocalization
    bool relocalization_enable = false;

    // Cluster
    double deltaA = 2;
    double deltaR = 1;
    double deltaP = 2;

    // Graph
    double edge_dis_th = 60;        // edge_th for building graph
    double subgraph_edge_th = 20;   // for sub triangle edge threshold
    int subinterval = 30;           // subinterval for building graph
    int graph_node_dimension = 30;  // dimension of vector in graph adjacency matrix

    double nearest_neighbor_vehicle_disth = 1.0; // nearest neighbor distance for NMS vehicle in local graph map
    double nearest_neighbor_pole_disth = 1.0;   // nearest neighbor distance for NMS pole-like objects in local graph map

    double max_local_graph_map_range = 60.0; // max range for local graph map

    // Semantic-geometry arbitration for registration
    bool arbitration_enable = true;
    int arbitration_warmup_iterations = 3;
    double arbitration_alpha = 0.5;
    double arbitration_dynamic_alpha = 0.3;
    double arbitration_geo_sigma = 0.5;
    double arbitration_anchor_voxel_size = 0.6;
    double arbitration_anchor_downsample_voxel_size = 1.2;
    double arbitration_anchor_residual_max = 0.8;
    int arbitration_anchor_k = 4;
    int arbitration_min_anchor_count = 30;
    double arbitration_min_anchor_ratio = 0.03;
    int arbitration_min_correspondence_count = 300;
    double arbitration_geo_score_floor = 0.4;
    double arbitration_dynamic_geo_score_floor = 0.02;
    double arbitration_dynamic_geo_reject_threshold = 0.25;
    double arbitration_dynamic_residual_reject_threshold = 0.8;
    bool arbitration_dynamic_hard_reject = true;
    bool arbitration_strict_dynamic_suppression = true;
    bool arbitration_debug_log = false;
    int arbitration_debug_print_every_n = 1;
    bool arbitration_debug_print_weight_changed_points = true;
    double arbitration_debug_weight_change_eps = 0.05;
    bool arbitration_debug_print_anchor_points = true;
    int arbitration_debug_max_anchor_points = 200;

};

class SemGraphSLAM {
public:

    // Data structure abbreviation 
    using V3d_i_pair = std::pair<V3d_i,V3d_i>;
    using V3d_i_pair_graph = std::tuple<V3d_i,V3d_i,Graph>;
    using V3d_i_tuple = std::tuple<V3d_i,V3d_i,V3d_i>;
    using V3d_pair = std::pair<V3d, V3d>;

public:
    explicit SemGraphSLAM(const SemGraphSLAMConfig &config)
        : config_(config),
          local_map_(config.voxel_size, config.max_range, config.max_points_per_voxel),
          local_graph_map_(config.edge_dis_th, config.subgraph_edge_th,config.subinterval, config.graph_node_dimension, config.nearest_neighbor_vehicle_disth,
                                config.nearest_neighbor_pole_disth,config.max_local_graph_map_range),
          adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range) {}

    SemGraphSLAM() : SemGraphSLAM(SemGraphSLAMConfig{}) {}

public:

    V3d_i_pair_graph mainProcess(const V3d &frame,const std::vector<int> &frame_label,const std::vector<double> &timestamps,std::string dataset);
    
    V3d_i_tuple VoxelizeSemantic(const V3d_i &frame) const;

    double GetAdaptiveThreshold();
    bool HasMoved();

    V4d FusePointsAndLabels(const V3d_i &frame);
    Sophus::SE3d GetPredictionModel() const;
    std::pair<V3d, V3d> relocalization_corr;  // for visualization
    Sophus::SE3d initial_guess_for_relocalization;  // for visualization

public:
    V4d LocalMap() const { return local_map_.Pointcloud(); };
    Graph LocalGraphMap() const { return local_graph_map_.InsGraphMap; };
    std::vector<Sophus::SE3d> poses() const { return poses_; };

private:
    std::vector<Sophus::SE3d> poses_;
    SemGraphSLAMConfig config_;


    VoxelHashMap local_map_;
    GraphMap local_graph_map_;
    AdaptiveThreshold adaptive_threshold_;
};

}   // namespace graph_slam::pipeline
