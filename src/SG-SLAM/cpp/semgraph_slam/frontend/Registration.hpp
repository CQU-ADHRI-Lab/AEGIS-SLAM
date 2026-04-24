/* 
    SemGraphSLAM is heavily inspired by the framework of KISS-ICP (https://github.com/PRBonn/kiss-icp).
    We are deeply appreciative of Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss for their contributions to the open-source community.
*/

// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>
#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
#include <unordered_map>

#include "VoxelHashMap.hpp"
#include "semgraph_slam/core/Coreutils.h"

namespace  {

// Semantic weights indexed by mapped label (0-25).
// Discrete rigid objects (trunk, pole, sign) get higher weight;
// planar/sliding-prone surfaces (road, sidewalk, terrain) get lower weight;
// dynamic classes get zero.
inline double GetSemanticWeightByLabel(int label) {
    static const double kTable[] = {
        /* 0  unlabeled     */ 0.8,
        /* 1  car           */ 0.9,
        /* 2  bicycle       */ 0.7,
        /* 3  motorcycle    */ 0.7,
        /* 4  truck         */ 0.9,
        /* 5  other-vehicle */ 0.9,
        /* 6  person        */ 0.0,
        /* 7  bicyclist     */ 0.0,
        /* 8  motorcyclist  */ 0.0,
        /* 9  road          */ 0.8,
        /* 10 parking       */ 0.8,
        /* 11 sidewalk      */ 0.8,
        /* 12 other-ground  */ 0.8,
        /* 13 building      */ 1.0,
        /* 14 fence         */ 0.9,
        /* 15 vegetation    */ 0.85,
        /* 16 trunk         */ 1.3,
        /* 17 terrain       */ 0.8,
        /* 18 pole          */ 1.3,
        /* 19 traffic-sign  */ 1.3,
        /* 20 moving-car    */ 0.0,
        /* 21 moving-bicycl */ 0.0,
        /* 22 moving-person */ 0.0,
        /* 23 moving-moto   */ 0.0,
        /* 24 moving-other  */ 0.0,
        /* 25 moving-truck  */ 0.0,
    };
    if (label < 0 || label >= static_cast<int>(sizeof(kTable) / sizeof(kTable[0])))
        return 0.8;
    return kTable[label];
}

}

namespace graph_slam {

struct RegistrationArbitrationConfig {
    bool enable = true;
    int warmup_iterations = 3;  // disable arbitration for first N ICP iterations
    double alpha = 0.6;  // W = W_sem * (alpha + (1 - alpha) * S_geo)
    double dynamic_alpha = 0.35;
    double geo_sigma = 0.5;
    double anchor_voxel_size = 0.6;
    double anchor_downsample_voxel_size = 1.2;
    double anchor_residual_max = 0.8;
    int anchor_k = 4;
    int min_anchor_count = 20;
    double min_anchor_ratio = 0.02;
    int min_correspondence_count = 200;
    double geo_score_floor = 0.2;
    double dynamic_geo_score_floor = 0.05;
    double dynamic_geo_reject_threshold = 0.25;
    double dynamic_residual_reject_threshold = 0.8;
    bool dynamic_hard_reject = true;
    bool strict_dynamic_suppression = true;
    bool debug_log = false;
    int debug_print_every_n = 1;
    bool debug_print_weight_changed_points = true;
    // print points with |w_after - w_before| > debug_weight_change_eps
    double debug_weight_change_eps = 0.05;
    bool debug_print_anchor_points = true;
    int debug_max_anchor_points = 200;
};

Sophus::SE3d RegisterFrameSemantic(const std::vector<Eigen::Vector4d> &frame,
                           const VoxelHashMap &voxel_map,
                           const Sophus::SE3d &initial_guess,
                           double max_correspondence_distance,
                           double kernel,
                           const RegistrationArbitrationConfig &arbitration_config =
                               RegistrationArbitrationConfig{});
}   // namespace graph_slam
