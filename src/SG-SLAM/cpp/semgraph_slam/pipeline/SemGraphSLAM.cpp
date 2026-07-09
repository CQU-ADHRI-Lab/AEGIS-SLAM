// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#include <algorithm>
#include <unordered_set>
#include <tbb/parallel_for.h>

#include "SemGraphSLAM.hpp"

#include "semgraph_slam/frontend/Deskew.hpp"
#include "semgraph_slam/frontend/Preprocessing.hpp"
#include "semgraph_slam/frontend/Registration.hpp"
#include "semgraph_slam/frontend/VoxelHashMap.hpp"
#include "semgraph_slam/loopclosure/LoopClosure.hpp"

namespace graph_slam{

SemGraphSLAM::V3d_i_pair_graph SemGraphSLAM::mainProcess(const V3d &frame, const std::vector<int> &frame_label, const std::vector<double> &timestamps,std::string dataset){

    V3d deskew_frame = frame;

    // Deskew
    if(config_.deskew&&!timestamps.empty()){
        const size_t N = poses().size();
        if(N>2){
            const auto &start_pose = poses_[N - 2];
            const auto &finish_pose = poses_[N - 1];
            deskew_frame = DeSkewScan(frame, timestamps, start_pose, finish_pose);
        }
    }

    if(dataset=="kitti"){
        // Correct KITTI scan
        deskew_frame = CorrectKITTIScan(frame);
    }
        
    // Preprocess frame
    const auto &cropped_frame_label = PreprocessSemantic(deskew_frame,frame_label, config_.max_range, config_.min_range);

    // Voxel downsample
    const auto &[source, frame_downsample, frame_downsample_cluster] = VoxelizeSemantic(cropped_frame_label);

    // Cluster
    V3d_i foreground_points;
    V3d_i background_points;
    auto cluster_box = ClusterPoints(frame_downsample_cluster.first, frame_downsample_cluster.second,
                                     background_points, foreground_points,
                                     config_.deltaA, config_.deltaR, config_.deltaP);

    // Build frame graph
    auto graph = BuildGraph(cluster_box,config_.edge_dis_th,config_.subinterval,config_.graph_node_dimension,config_.subgraph_edge_th);
    graph.back_points = background_points;
    graph.front_points = foreground_points;

    // Node tracking: find instance node match
    const auto frame2map_match =  local_graph_map_.FindInsMatch(graph);
    
    
    // Get motion prediction and adaptive_threshold
    const double sigma = GetAdaptiveThreshold();

    // Compute initial_guess for ICP
    const auto prediction = GetPredictionModel();
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess = last_pose * prediction;

    initial_guess_for_relocalization = initial_guess;

    // Fuse (vector3d point, label) -> vector 4d point4 for subsequent processing
    const auto source_4d = FusePointsAndLabels(source);
    const auto frame_downsample_4d = FusePointsAndLabels(frame_downsample);

    RegistrationArbitrationConfig arbitration_config;
    arbitration_config.enable = config_.arbitration_enable;
    arbitration_config.warmup_iterations = config_.arbitration_warmup_iterations;
    arbitration_config.alpha = config_.arbitration_alpha;
    arbitration_config.dynamic_alpha = config_.arbitration_dynamic_alpha;
    arbitration_config.geo_sigma = config_.arbitration_geo_sigma;
    arbitration_config.anchor_voxel_size = config_.arbitration_anchor_voxel_size;
    arbitration_config.anchor_downsample_voxel_size =
        std::max(config_.arbitration_anchor_voxel_size * 2.0,
                 config_.arbitration_anchor_voxel_size + 1e-6);
    arbitration_config.anchor_residual_max = config_.arbitration_anchor_residual_max;
    arbitration_config.anchor_k = config_.arbitration_anchor_k;
    arbitration_config.min_anchor_count = config_.arbitration_min_anchor_count;
    arbitration_config.min_anchor_ratio = config_.arbitration_min_anchor_ratio;
    arbitration_config.min_correspondence_count = config_.arbitration_min_correspondence_count;
    arbitration_config.geo_score_floor = config_.arbitration_geo_score_floor;
    arbitration_config.dynamic_geo_score_floor = config_.arbitration_dynamic_geo_score_floor;
    arbitration_config.dynamic_geo_reject_threshold = config_.arbitration_dynamic_geo_reject_threshold;
    arbitration_config.dynamic_residual_reject_threshold =
        config_.arbitration_dynamic_residual_reject_threshold;
    arbitration_config.dynamic_hard_reject = config_.arbitration_dynamic_hard_reject;
    arbitration_config.strict_dynamic_suppression = config_.arbitration_strict_dynamic_suppression;
    arbitration_config.debug_log = config_.arbitration_debug_log;
    arbitration_config.debug_print_every_n = config_.arbitration_debug_print_every_n;
    arbitration_config.debug_print_weight_changed_points =
        config_.arbitration_debug_print_weight_changed_points;
    arbitration_config.debug_weight_change_eps = config_.arbitration_debug_weight_change_eps;
    arbitration_config.debug_print_anchor_points =
        config_.arbitration_debug_print_anchor_points;
    arbitration_config.debug_max_anchor_points =
        config_.arbitration_debug_max_anchor_points;

    // Registration
    Sophus::SE3d new_pose = RegisterFrameSemantic(source_4d,         // the current point cloud
                                                          local_map_,     // the local pc map
                                                          initial_guess,  // initial guess
                                                          3.0 * sigma,    // max_correspondence_distance
                                                          sigma / 3.0,    // kernel
                                                          arbitration_config);

    // The deviation between the initial guess and the final pose
    auto model_deviation = initial_guess.inverse() * new_pose;

    // Relocalization
    relocalization_corr = std::make_pair(V3d(),V3d());
    if(poses_.size()>2 && config_.relocalization_enable){ // relocalization enable

        // Check model deviation
        if(model_deviation.translation().norm()>config_.model_deviation_trans || model_deviation.so3().log().norm()>config_.model_deviation_rot){

            std::cout<<YELLOW<<"[ Relo. ] relocalization"<<std::endl;
            const auto [initial_guess_graph, estimate_poses_flag] = local_graph_map_.Relocalization(graph, frame2map_match,config_.inlier_rate_th);

            std::cout<<YELLOW<<"[ Relo. ] estimate_poses_flag:"<<estimate_poses_flag<<std::endl;
            if(estimate_poses_flag){ // successful relocalization
                // Regisration with relocalized poses
                new_pose = RegisterFrameSemantic(source_4d,         //
                                                    local_map_,     //
                                                    initial_guess_graph,  // the relocalized poses
                                                    3.0 * sigma,    //
                                                    sigma / 3.0,
                                                    arbitration_config);
                model_deviation = Sophus::SE3d();
                relocalization_corr = local_graph_map_.relo_corr;
            }
            else{
                relocalization_corr = std::make_pair(V3d(),V3d());
            }
        }

    }
     
    // Update constant motion model
    adaptive_threshold_.UpdateModelDeviation(model_deviation);

    // Update local point cloud map
    local_map_.Update(frame_downsample_4d, new_pose);

    // Update local graph map
    local_graph_map_.Update(graph, frame2map_match, new_pose);

    // 持久节点增强：补充因误标签丢失的节点
    Graph graph_for_backend = graph;
    if (config_.persistent_node_augment_enable && !local_graph_map_.instance_in_localmap.empty()) {
        graph_for_backend = AugmentGraphWithPersistentNodes(graph, frame2map_match, new_pose);
    }

    // 描述子融合：将帧描述子与局部图描述子按匹配率加权融合
    if (config_.descriptor_fusion_enable && !local_graph_map_.instance_in_localmap.empty()) {
        FuseDescriptorWithLocalMap(graph_for_backend, frame2map_match, new_pose);
    }

    // Push new pose to global poses
    poses_.push_back(new_pose);
    return {frame_downsample_cluster, source, graph_for_backend};
}

// Voxel downsample the semantic frame
SemGraphSLAM::V3d_i_tuple SemGraphSLAM::VoxelizeSemantic(const V3d_i &frame) const {
    const auto voxel_size = config_.voxel_size;
    const auto voxel_size_cluster = config_.voxel_size_cluster;

    const auto frame_downsample_cluster = VoxelDownsampleSemantic(frame, voxel_size_cluster);
    const auto frame_downsample = VoxelDownsampleSemantic(frame, voxel_size * 0.5);
    const auto source = VoxelDownsampleSemantic(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample,frame_downsample_cluster};
}

//Fusing the points and labels, time comsumption: ~0.02ms
V4d  SemGraphSLAM::FusePointsAndLabels(const V3d_i &frame){
    assert(frame.first.size()==frame.second.size());
    V4d points(frame.first.size());

    tbb::parallel_for(size_t(0),frame.first.size(), [&](size_t i){
        points[i].head<3>() =  frame.first[i];
        points[i](3) = frame.second[i];
    });
    return points;
}


Graph SemGraphSLAM::AugmentGraphWithPersistentNodes(
    const Graph &frame_graph,
    const VTbii &frame2map_match,
    const Sophus::SE3d &pose) {

    const int total_frame_nodes = static_cast<int>(frame_graph.node_labels.size());
    if (total_frame_nodes == 0) return frame_graph;

    // 统计当前帧节点的匹配率
    int matched_frame_nodes = 0;
    std::unordered_set<int> matched_localmap_ids;
    for (const auto &[flag, id_frame, id_local] : frame2map_match) {
        if (flag) {
            matched_frame_nodes++;
            if (id_local >= 0) matched_localmap_ids.insert(id_local);
        }
    }

    double match_rate = static_cast<double>(matched_frame_nodes) / total_frame_nodes;

    // 自适应判断：匹配率高说明当前帧与局部图一致（标签基本正确），无需增强
    if (match_rate > config_.persistent_node_augment_match_rate_th) {
        return frame_graph;
    }

    // 当前帧节点的世界坐标位置和标签
    struct FrameNodeInfo {
        Eigen::Vector3d world_pos;
        int label;
    };
    std::vector<FrameNodeInfo> frame_nodes_world;
    for (size_t i = 0; i < frame_graph.node_labels.size(); i++) {
        frame_nodes_world.push_back({
            pose * frame_graph.node_centers[i],
            frame_graph.node_labels[i]
        });
    }

    // 当前帧的节点作为基础（转到世界坐标）
    std::vector<InsNode> combined_nodes;
    for (size_t i = 0; i < frame_graph.node_labels.size(); i++) {
        InsNode node;
        node.pose = frame_nodes_world[i].world_pos;
        node.label = frame_graph.node_labels[i];
        node.dimension = frame_graph.node_dimensions[i];
        node.points_num = frame_graph.points_num[i];
        node.id = static_cast<int>(i);
        combined_nodes.push_back(node);
    }

    const auto &persistent_nodes = local_graph_map_.instance_in_localmap;
    const Eigen::Vector3d origin = pose.translation();
    const double near_range = config_.persistent_node_augment_near_range;
    const double cover_dist_th = config_.persistent_node_augment_cover_dist;
    const int max_augmented = config_.persistent_node_augment_max_nodes;
    const int min_points = config_.persistent_node_augment_min_points;
    int augmented_count = 0;

    for (size_t i = 0; i < persistent_nodes.size() && augmented_count < max_augmented; i++) {
        if (matched_localmap_ids.count(static_cast<int>(i))) continue;

        const auto &pnode = persistent_nodes[i];
        if (pnode.points_num < min_points) continue;

        double dist = (pnode.pose - origin).norm();
        if (dist > near_range) continue;

        bool covered = false;
        for (const auto &fn : frame_nodes_world) {
            if (fn.label == pnode.label &&
                (fn.world_pos - pnode.pose).norm() < cover_dist_th) {
                covered = true;
                break;
            }
        }
        if (covered) continue;

        combined_nodes.push_back(pnode);
        augmented_count++;
    }

    if (augmented_count == 0) return frame_graph;

    Graph augmented = ReBuildGraph(combined_nodes,
                                  config_.edge_dis_th,
                                  config_.subinterval,
                                  config_.graph_node_dimension,
                                  config_.subgraph_edge_th);

    Sophus::SE3d pose_inv = pose.inverse();
    for (auto &center : augmented.node_centers) {
        center = pose_inv * center;
    }

    augmented.back_points = frame_graph.back_points;
    augmented.front_points = frame_graph.front_points;
    augmented.new_instance = frame_graph.new_instance;

    return augmented;
}

void SemGraphSLAM::FuseDescriptorWithLocalMap(
    Graph &graph_for_backend,
    const VTbii &frame2map_match,
    const Sophus::SE3d &pose) {

    const auto &persistent_nodes = local_graph_map_.instance_in_localmap;
    const Eigen::Vector3d origin = pose.translation();
    const double range = config_.descriptor_fusion_local_map_range;

    // 收集距离范围内的局部图持久节点
    std::vector<InsNode> local_nodes;
    for (const auto &pn : persistent_nodes) {
        if ((pn.pose - origin).norm() <= range) {
            local_nodes.push_back(pn);
        }
    }

    if (static_cast<int>(local_nodes.size()) < config_.descriptor_fusion_min_local_nodes) {
        return;
    }

    // 用局部图节点重建图结构
    Graph local_map_graph = ReBuildGraph(local_nodes,
                                         config_.edge_dis_th,
                                         config_.subinterval,
                                         config_.graph_node_dimension,
                                         config_.subgraph_edge_th);
    // 背景 SSC 部分来自当前帧（背景点受误标签影响较小）
    local_map_graph.back_points = graph_for_backend.back_points;

    // 生成两个描述子
    auto frame_desc = GenScanDescriptors(graph_for_backend, config_.edge_dis_th, config_.subinterval);
    auto local_desc = GenScanDescriptors(local_map_graph, config_.edge_dis_th, config_.subinterval);

    if (frame_desc.size() != local_desc.size() || frame_desc.empty()) {
        return;
    }

    // 计算匹配率作为融合权重
    const int total = static_cast<int>(graph_for_backend.node_labels.size());
    int matched = 0;
    for (const auto &[flag, id_f, id_l] : frame2map_match) {
        if (flag) matched++;
    }
    // alpha 高 → 信任帧描述子；alpha 低 → 信任局部图描述子
    double alpha = (total > 0) ? static_cast<double>(matched) / total : 1.0;
    alpha = std::max(0.2, std::min(1.0, alpha));

    // 加权融合并重新 L2 归一化
    std::vector<float> fused(frame_desc.size());
    for (size_t i = 0; i < fused.size(); i++) {
        fused[i] = static_cast<float>(alpha) * frame_desc[i]
                 + static_cast<float>(1.0 - alpha) * local_desc[i];
    }
    float norm_sq = std::inner_product(fused.begin(), fused.end(), fused.begin(), 0.0f);
    float norm = std::sqrt(norm_sq);
    if (norm < 1e-6f) norm = 1e-6f;
    for (auto &v : fused) v /= norm;

    graph_for_backend.precomputed_descriptor = std::move(fused);
}

double SemGraphSLAM::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_threshold;
    }
    return adaptive_threshold_.ComputeThreshold();
    // return config_.initial_threshold;
}

bool SemGraphSLAM::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}

Sophus::SE3d SemGraphSLAM::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

}
