/* 
    SemGraphSLAM is heavily inspired by the framework of KISS-ICP (https://github.com/PRBonn/kiss-icp).
    We are deeply appreciative of Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss for their contributions to the open-source community.
*/

// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#include "Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>
#include <chrono>
#include <iostream>
#include <fstream>
#include <unordered_map>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace {

inline double square(double x) { return x * x; }

struct Int3 {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const Int3 &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct Int3Hash {
    size_t operator()(const Int3 &v) const {
        const size_t hx = static_cast<size_t>(v.x) * 73856093u;
        const size_t hy = static_cast<size_t>(v.y) * 19349663u;
        const size_t hz = static_cast<size_t>(v.z) * 83492791u;
        return hx ^ hy ^ hz;
    }
};

struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple &other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        this->total_points += other.total_points;
        this->used_points += other.used_points;
        this->suppressed_semantic += other.suppressed_semantic;
        this->hard_reject_dynamic += other.hard_reject_dynamic;
        this->dynamic_points += other.dynamic_points;
        this->geo_score_sum += other.geo_score_sum;
        this->semantic_weight_sum += other.semantic_weight_sum;
        this->final_weight_sum += other.final_weight_sum;
        return *this;
    }

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
    size_t total_points = 0;
    size_t used_points = 0;
    size_t suppressed_semantic = 0;
    size_t hard_reject_dynamic = 0;
    size_t dynamic_points = 0;
    double geo_score_sum = 0.0;
    double semantic_weight_sum = 0.0;
    double final_weight_sum = 0.0;
};

struct ArbitrationDebugInfo {
    bool arbitration_active = false;
    size_t correspondence_count = 0;
    size_t anchor_count = 0;
    size_t reliable_anchor_count = 0;
    size_t min_anchor_required = 0;
    size_t used_points = 0;
    size_t suppressed_semantic = 0;
    size_t hard_reject_dynamic = 0;
    size_t dynamic_points = 0;
    double mean_geo_score = 1.0;
    double mean_semantic_weight = 1.0;
    double mean_final_weight = 1.0;
};

struct AnchorPointInfo {
    size_t index = 0;
    int semantic_label = 0;
    bool reliable = false;
    double residual = 0.0;
    Eigen::Vector3d source = Eigen::Vector3d::Zero();
};

struct WeightChangePoint {
    size_t index = 0;
    int semantic_label = 0;
    bool is_dynamic = false;
    bool hard_reject = false;
    double residual = 0.0;
    double w_before = 1.0;
    double w_after = 1.0;
    double delta = 0.0;
    double w_semantic = 1.0;
    double w_geo = 1.0;
};

void TransformPoints4D(const Sophus::SE3d &T, std::vector<Eigen::Vector4d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const Eigen::Vector4d &point) { 
                    Eigen::Vector4d pc_out;
                    pc_out.head<3>() = T * point.head<3>();
                    pc_out(3) = point(3);
                    return pc_out; });
}



std::pair<std::vector<Eigen::Vector3d>,std::vector<int>> SeparatePointsAndLabels(const std::vector<Eigen::Vector4d> &frame){
    std::vector<Eigen::Vector3d> pointcloud(frame.size());
    std::vector<int> label(frame.size());
    tbb::parallel_for(size_t(0),frame.size(), [&](size_t i){
        pointcloud[i] =  frame[i].head<3>();
        label[i] = (int)frame[i](3);
    });
    return std::make_pair(pointcloud,label);
}

double GetSemanticWeight(const int semantic_label) {
    return GetSemanticWeightByLabel(semantic_label);
}

bool IsAnchorSemantic(const int semantic_label) {
    // static and spatially structured classes
    return semantic_label == 13 || semantic_label == 16 ||
           semantic_label == 18 || semantic_label == 19;
}

bool IsPotentialDynamicSemantic(const int semantic_label) {
    switch (semantic_label) {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 20:
        case 21:
        case 22:
        case 23:
        case 24:
        case 25:
            return true;
        default:
            return false;
    }
}

Int3 ToVoxel(const Eigen::Vector3d &point, const double voxel_size) {
    return {static_cast<int>(std::floor(point.x() / voxel_size)),
            static_cast<int>(std::floor(point.y() / voxel_size)),
            static_cast<int>(std::floor(point.z() / voxel_size))};
}

std::vector<int> BuildBoundaryAnchors(const std::vector<Eigen::Vector3d> &source,
                                      const std::vector<int> &labels,
                                      const graph_slam::RegistrationArbitrationConfig &config) {
    std::vector<int> anchors;
    if (source.size() != labels.size()) return anchors;
    if (config.anchor_voxel_size <= 0.0 || config.anchor_downsample_voxel_size <= 0.0) return anchors;

    std::unordered_map<Int3, std::vector<int>, Int3Hash> voxels;
    for (size_t i = 0; i < source.size(); ++i) {
        if (!IsAnchorSemantic(labels[i])) continue;
        if (GetSemanticWeight(labels[i]) <= 0.0) continue;
        voxels[ToVoxel(source[i], config.anchor_voxel_size)].push_back(static_cast<int>(i));
    }
    if (voxels.empty()) return anchors;

    static const Int3 kNbr6[6] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
        {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

    std::vector<int> boundary_candidates;
    boundary_candidates.reserve(source.size() / 8 + 1);
    for (const auto &[voxel, point_indices] : voxels) {
        bool is_boundary = false;
        for (const auto &offset : kNbr6) {
            const Int3 neighbor{voxel.x + offset.x, voxel.y + offset.y, voxel.z + offset.z};
            if (voxels.find(neighbor) == voxels.end()) {
                is_boundary = true;
                break;
            }
        }
        if (!is_boundary) continue;
        boundary_candidates.insert(boundary_candidates.end(), point_indices.begin(), point_indices.end());
    }

    std::unordered_map<Int3, int, Int3Hash> downsampled_anchors;
    downsampled_anchors.reserve(boundary_candidates.size());
    for (const int idx : boundary_candidates) {
        const Int3 key = ToVoxel(source[idx], config.anchor_downsample_voxel_size);
        if (downsampled_anchors.find(key) == downsampled_anchors.end()) {
            downsampled_anchors.emplace(key, idx);
        }
    }

    anchors.reserve(downsampled_anchors.size());
    for (const auto &[_, idx] : downsampled_anchors) {
        anchors.push_back(idx);
    }
    return anchors;
}

std::vector<double> ComputeGeoConsensusScores(
    const std::vector<Eigen::Vector3d> &source,
    const std::vector<Eigen::Vector3d> &target,
    const std::vector<int> &anchor_indices,
    const graph_slam::RegistrationArbitrationConfig &config) {
    std::vector<double> geo_scores(source.size(), 1.0);
    if (source.size() != target.size()) return geo_scores;
    if (anchor_indices.size() < static_cast<size_t>(std::max(1, config.min_anchor_count))) return geo_scores;

    const int k = std::max(1, config.anchor_k);
    const double sigma = std::max(1e-6, config.geo_sigma);
    const double denom = 2.0 * sigma * sigma;

    tbb::parallel_for(size_t(0), source.size(), [&](size_t i) {
        std::vector<std::pair<double, int>> knn(static_cast<size_t>(k),
                                                {std::numeric_limits<double>::max(), -1});

        for (const int anchor_idx : anchor_indices) {
            if (anchor_idx < 0 || anchor_idx >= static_cast<int>(source.size())) continue;
            if (anchor_idx == static_cast<int>(i)) continue;
            const double dist2 = (source[i] - source[anchor_idx]).squaredNorm();

            int worst_slot = 0;
            for (int m = 1; m < k; ++m) {
                if (knn[m].first > knn[worst_slot].first) worst_slot = m;
            }
            if (dist2 < knn[worst_slot].first) {
                knn[worst_slot] = {dist2, anchor_idx};
            }
        }

        double score_sum = 0.0;
        int valid_num = 0;
        for (const auto &[dist2_src, anchor_idx] : knn) {
            if (anchor_idx < 0) continue;
            const double d_src = std::sqrt(dist2_src);
            const double d_tgt = (target[i] - target[anchor_idx]).norm();
            const double delta = std::abs(d_src - d_tgt);
            score_sum += std::exp(-(delta * delta) / denom);
            ++valid_num;
        }
        if (valid_num > 0) geo_scores[i] = score_sum / static_cast<double>(valid_num);
    });

    return geo_scores;
}

std::vector<int> FilterReliableAnchors(
    const std::vector<Eigen::Vector3d> &source,
    const std::vector<Eigen::Vector3d> &target,
    const std::vector<int> &anchor_indices,
    const double max_anchor_residual) {
    if (source.size() != target.size()) return {};
    if (max_anchor_residual <= 0.0) return anchor_indices;

    const double max_anchor_residual2 = max_anchor_residual * max_anchor_residual;
    std::vector<int> reliable;
    reliable.reserve(anchor_indices.size());
    for (const int anchor_idx : anchor_indices) {
        if (anchor_idx < 0 || anchor_idx >= static_cast<int>(source.size())) continue;
        const double residual2 = (source[anchor_idx] - target[anchor_idx]).squaredNorm();
        if (residual2 <= max_anchor_residual2) {
            reliable.push_back(anchor_idx);
        }
    }
    return reliable;
}

Sophus::SE3d AlignClouds(const std::vector<Eigen::Vector4d> &source4d,
                         const std::vector<Eigen::Vector4d> &target4d,
                         double th,
                         const graph_slam::RegistrationArbitrationConfig &arbitration_config,
                         bool arbitration_enabled_this_iter,
                         ArbitrationDebugInfo *debug_info = nullptr,
                         std::vector<WeightChangePoint> *changed_points = nullptr,
                         std::vector<AnchorPointInfo> *anchor_points = nullptr) {
    
    const auto source_pl =  SeparatePointsAndLabels(source4d);
    const auto target_pl =  SeparatePointsAndLabels(target4d);

    const std::vector<Eigen::Vector3d> source = source_pl.first;
    const std::vector<Eigen::Vector3d> target = target_pl.first;
    const std::vector<int> label = source_pl.second;

    bool arbitration_active = arbitration_enabled_this_iter &&
                             arbitration_config.enable &&
                             source.size() >= static_cast<size_t>(
                                                  std::max(1, arbitration_config.min_correspondence_count));
    std::vector<double> geo_scores(source.size(), 1.0);
    size_t anchor_count = 0;
    size_t reliable_anchor_count = 0;
    size_t min_anchor_required = 0;
    if (arbitration_active) {
        const auto anchor_indices = BuildBoundaryAnchors(source, label, arbitration_config);
        anchor_count = anchor_indices.size();
        const size_t min_anchor_by_ratio = static_cast<size_t>(
            std::ceil(arbitration_config.min_anchor_ratio * static_cast<double>(source.size())));
        const size_t min_anchor_num =
            static_cast<size_t>(std::max(arbitration_config.min_anchor_count, static_cast<int>(min_anchor_by_ratio)));
        min_anchor_required = min_anchor_num;
        if (anchor_indices.size() >= min_anchor_num) {
            const auto reliable_anchor_indices = FilterReliableAnchors(
                source, target, anchor_indices, arbitration_config.anchor_residual_max);
            reliable_anchor_count = reliable_anchor_indices.size();
            if (anchor_points != nullptr) {
                anchor_points->clear();
                anchor_points->reserve(anchor_indices.size());
                std::vector<char> reliable_mask(source.size(), 0);
                for (const int ridx : reliable_anchor_indices) {
                    if (ridx >= 0 && ridx < static_cast<int>(reliable_mask.size())) reliable_mask[ridx] = 1;
                }
                for (const int aidx : anchor_indices) {
                    if (aidx < 0 || aidx >= static_cast<int>(source.size())) continue;
                    const double residual = (source[aidx] - target[aidx]).norm();
                    anchor_points->push_back(
                        {static_cast<size_t>(aidx), label[aidx], reliable_mask[aidx] != 0, residual, source[aidx]});
                }
            }
            if (reliable_anchor_indices.size() >= min_anchor_num) {
                geo_scores = ComputeGeoConsensusScores(source, target, reliable_anchor_indices, arbitration_config);
            } else {
                arbitration_active = false;
            }
        } else {
            arbitration_active = false;
        }
    }

    if (changed_points != nullptr) {
        changed_points->clear();
        if (arbitration_active) {
            changed_points->reserve(source.size() / 4 + 1);
            auto Weight = [&](double residual2) { return square(th) / square(th + residual2); };
            const double eps = std::max(0.0, arbitration_config.debug_weight_change_eps);
            for (size_t i = 0; i < source.size(); ++i) {
                const Eigen::Vector3d residual_vec = source[i] - target[i];
                const double residual_norm = residual_vec.norm();
                const double residual2 = residual_vec.squaredNorm();
                const int semantic_label = label[i];
                const bool is_dynamic_semantic = IsPotentialDynamicSemantic(semantic_label);
                const double w_semantic = GetSemanticWeight(semantic_label);
                if (w_semantic <= 1e-9) continue;

                const double w_geo = arbitration_active ? geo_scores[i] : 1.0;
                const bool hard_reject =
                    arbitration_config.dynamic_hard_reject && is_dynamic_semantic &&
                    w_geo < arbitration_config.dynamic_geo_reject_threshold &&
                    residual_norm > arbitration_config.dynamic_residual_reject_threshold;

                const double alpha_used =
                    is_dynamic_semantic ? arbitration_config.dynamic_alpha : arbitration_config.alpha;
                const double score_floor = is_dynamic_semantic
                                               ? arbitration_config.dynamic_geo_score_floor
                                               : arbitration_config.geo_score_floor;
                const double w_geo_safe = std::clamp(w_geo, score_floor, 1.0);
                const double w_arbitrated =
                    w_semantic * (alpha_used + (1.0 - alpha_used) * w_geo_safe);
                const double w_before = w_semantic * Weight(residual2);
                const double w_final =
                    hard_reject ? 0.0 : std::max(0.0, Weight(residual2) * w_arbitrated);
                const double delta = std::abs(w_final - w_before);

                if (hard_reject || delta > eps) {
                    changed_points->push_back(
                        {i, semantic_label, is_dynamic_semantic, hard_reject, residual_norm, w_before, w_final,
                         delta, w_semantic, w_geo});
                }
            }
        }
    }

    auto compute_jacobian_and_residual = [&](auto i) {
        const Eigen::Vector3d residual = source[i] - target[i];
        const int semantic_label = label[i];
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source[i]); 
        return std::make_tuple(J_r, residual,semantic_label);
    };

    // Key insight: follow original SG-SLAM pattern —
    // w_robust goes on JTJ (Hessian), arbitrated weight goes on JTr (gradient) only.
    // This preserves Hessian conditioning while letting arbitration steer the solution.
    const auto reduced = tbb::parallel_reduce(
        tbb::blocked_range<size_t>{0, source.size()},
        ResultTuple(),
        [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
            auto Weight = [&](double residual2) { return square(th) / square(th + residual2); };
            auto &JTJ_private = J.JTJ;
            auto &JTr_private = J.JTr;
            for (auto i = r.begin(); i < r.end(); ++i) {
                const auto &[J_r, residual, semantic_label] = compute_jacobian_and_residual(i);
                const double w_robust = Weight(residual.squaredNorm());
                const double w_semantic = GetSemanticWeight(semantic_label);
                const bool is_dynamic_semantic = IsPotentialDynamicSemantic(semantic_label);
                J.total_points++;
                J.semantic_weight_sum += w_semantic;
                if (is_dynamic_semantic) J.dynamic_points++;

                // All points contribute to Hessian (maintains conditioning)
                JTJ_private.noalias() += J_r.transpose() * w_robust * J_r;

                // Zero-weight semantics (moving-*) contribute nothing to gradient
                if (w_semantic <= 1e-9) {
                    J.suppressed_semantic++;
                    continue;
                }

                const double w_geo = arbitration_active ? geo_scores[i] : 1.0;
                J.geo_score_sum += w_geo;

                // Hard reject: dynamic + low geo + high residual -> skip gradient
                if (arbitration_active && arbitration_config.dynamic_hard_reject && is_dynamic_semantic &&
                    w_geo < arbitration_config.dynamic_geo_reject_threshold &&
                    residual.norm() > arbitration_config.dynamic_residual_reject_threshold) {
                    J.hard_reject_dynamic++;
                    continue;
                }

                const double alpha_used =
                    is_dynamic_semantic ? arbitration_config.dynamic_alpha : arbitration_config.alpha;
                const double score_floor = is_dynamic_semantic
                                               ? arbitration_config.dynamic_geo_score_floor
                                               : arbitration_config.geo_score_floor;
                const double w_geo_safe = std::clamp(w_geo, score_floor, 1.0);
                const double w_arbitrated = arbitration_active
                                                ? (w_semantic * (alpha_used +
                                                                 (1.0 - alpha_used) * w_geo_safe))
                                                : w_semantic;

                J.used_points++;
                const double w_grad = std::max(0.0, w_robust * w_arbitrated);
                J.final_weight_sum += w_grad;
                JTr_private.noalias() += J_r.transpose() * w_grad * residual;
            }
            return J;
        },
        [&](ResultTuple a, const ResultTuple &b) -> ResultTuple { return a + b; }); 
    const auto &JTJ = reduced.JTJ;
    const auto &JTr = reduced.JTr;
    
    if (debug_info != nullptr) {
        debug_info->arbitration_active = arbitration_active;
        debug_info->correspondence_count = source.size();
        debug_info->anchor_count = anchor_count;
        debug_info->reliable_anchor_count = reliable_anchor_count;
        debug_info->min_anchor_required = min_anchor_required;
        debug_info->used_points = reduced.used_points;
        debug_info->suppressed_semantic = reduced.suppressed_semantic;
        debug_info->hard_reject_dynamic = reduced.hard_reject_dynamic;
        debug_info->dynamic_points = reduced.dynamic_points;
        if (reduced.total_points > 0) {
            debug_info->mean_geo_score = reduced.geo_score_sum / static_cast<double>(reduced.total_points);
            debug_info->mean_semantic_weight =
                reduced.semantic_weight_sum / static_cast<double>(reduced.total_points);
        }
        if (reduced.used_points > 0) {
            debug_info->mean_final_weight =
                reduced.final_weight_sum / static_cast<double>(reduced.used_points);
        }
    }

    const Eigen::Vector6d x = JTJ.ldlt().solve(-JTr);
    return Sophus::SE3d::exp(x);  
}

constexpr int MAX_NUM_ITERATIONS_ = 500;
constexpr double ESTIMATION_THRESHOLD_ = 0.0001;

}  // namespace

namespace graph_slam {


void writePointCloud(const std::vector<Eigen::Vector4d>& points, const std::string& filename){
    struct PointXYZL {
        float x;
        float y;
        float z;
        float label;
    };
    std::ofstream file(filename, std::ios::out | std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing\n";
        return;
    }
    std::vector<PointXYZL> converted_points(points.size());
    for(size_t i=0; i< points.size(); i++){
        converted_points[i].x =  static_cast<float>(points[i](0));
        converted_points[i].y =  static_cast<float>(points[i](1));
        converted_points[i].z =  static_cast<float>(points[i](2));
        converted_points[i].label =  static_cast<float>(points[i](3));
    };
    file.write(reinterpret_cast<char*>(converted_points.data()), sizeof(PointXYZL)*points.size());
    file.close();
}


Sophus::SE3d RegisterFrameSemantic(const std::vector<Eigen::Vector4d> &frame,
                           const VoxelHashMap &voxel_map,
                           const Sophus::SE3d &initial_guess,
                           double max_correspondence_distance,
                           double kernel,
                           const RegistrationArbitrationConfig &arbitration_config) {
    if (voxel_map.Empty()) return initial_guess;

    // Transform the source points to the local frame using the initial guess
    std::vector<Eigen::Vector4d> source = frame;
    TransformPoints4D(initial_guess, source);
    static std::atomic<uint64_t> reg_call_counter{0};
    const uint64_t reg_call_id = reg_call_counter.fetch_add(1) + 1;
    const int print_every = std::max(1, arbitration_config.debug_print_every_n);
    const bool should_debug =
        arbitration_config.debug_log && (reg_call_id % static_cast<uint64_t>(print_every) == 0);
    const bool should_print_weight_changed_points =
        arbitration_config.debug_print_weight_changed_points &&
        (reg_call_id % static_cast<uint64_t>(print_every) == 0);
    const bool should_print_anchor_points =
        arbitration_config.debug_print_anchor_points &&
        (reg_call_id % static_cast<uint64_t>(print_every) == 0);
    ArbitrationDebugInfo debug_info;
    std::vector<WeightChangePoint> changed_points;
    std::vector<AnchorPointInfo> anchor_points;
    int icp_iterations = 0;
    bool converged = false;
    const int warmup_iters = std::max(0, arbitration_config.warmup_iterations);
    // ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        icp_iterations = j + 1;
        const bool arb_enabled = (j >= warmup_iters);
        // find correspondences: point-to-local pc map
        const auto &[src, tgt] = voxel_map.GetCorrespondences(source, max_correspondence_distance);
        // main computation
        ArbitrationDebugInfo *debug_ptr = (should_debug && j == warmup_iters) ? &debug_info : nullptr;
        std::vector<WeightChangePoint> *changed_ptr =
            (should_print_weight_changed_points && j == warmup_iters) ? &changed_points : nullptr;
        std::vector<AnchorPointInfo> *anchor_ptr =
            (should_print_anchor_points && j == warmup_iters) ? &anchor_points : nullptr;
        auto estimation =
            AlignClouds(src, tgt, kernel, arbitration_config, arb_enabled, debug_ptr, changed_ptr, anchor_ptr);
        // Transform the source points to the global frame using the estimated transformation
        TransformPoints4D(estimation, source);
        // Update iterations
        T_icp = estimation * T_icp;
        // Termination criteria
        if (estimation.log().norm() < ESTIMATION_THRESHOLD_) 
        {
            converged = true;
            break;
        }
    }
    if (should_debug) {
        std::cout << CYAN
                  << "[ ArbDebug ] call=" << reg_call_id
                  << " corr=" << debug_info.correspondence_count
                  << " active=" << debug_info.arbitration_active
                  << " anchor=" << debug_info.anchor_count
                  << " reliable_anchor=" << debug_info.reliable_anchor_count
                  << " min_anchor=" << debug_info.min_anchor_required
                  << " used=" << debug_info.used_points
                  << " dyn=" << debug_info.dynamic_points
                  << " suppress_sem=" << debug_info.suppressed_semantic
                  << " reject_dyn=" << debug_info.hard_reject_dynamic
                  << " mean_geo=" << debug_info.mean_geo_score
                  << " mean_sem=" << debug_info.mean_semantic_weight
                  << " mean_w=" << debug_info.mean_final_weight
                  << " icp_iter=" << icp_iterations
                  << " converged=" << converged
                  << RESET << std::endl;
    }
    if (should_print_weight_changed_points) {
        std::sort(changed_points.begin(), changed_points.end(),
                  [](const WeightChangePoint &a, const WeightChangePoint &b) {
                      return a.delta > b.delta;
                  });
        std::cout << YELLOW << "[ WeightCompare ] call=" << reg_call_id
                  << " threshold=" << arbitration_config.debug_weight_change_eps
                  << " changed_points=" << changed_points.size() << RESET << std::endl;
        for (const auto &p : changed_points) {
            std::cout << YELLOW << "[ WeightCompare ] idx=" << p.index
                      << " label=" << p.semantic_label
                      << " dynamic=" << p.is_dynamic
                      << " hard_reject=" << p.hard_reject
                      << " residual=" << p.residual
                      << " w_before=" << p.w_before
                      << " w_after=" << p.w_after
                      << " delta=" << p.delta
                      << " w_sem=" << p.w_semantic
                      << " w_geo=" << p.w_geo
                      << RESET << std::endl;
        }
    }
    if (should_print_anchor_points) {
        size_t reliable_num = 0;
        for (const auto &a : anchor_points) {
            if (a.reliable) reliable_num++;
        }
        std::cout << GREEN << "[ AnchorFrame ] call=" << reg_call_id
                  << " anchor_total=" << anchor_points.size()
                  << " reliable=" << reliable_num
                  << " max_print=" << arbitration_config.debug_max_anchor_points
                  << RESET << std::endl;
        const size_t print_num = std::min(anchor_points.size(),
                                          static_cast<size_t>(std::max(0, arbitration_config.debug_max_anchor_points)));
        for (size_t i = 0; i < print_num; ++i) {
            const auto &a = anchor_points[i];
            std::cout << GREEN << "[ Anchor ] idx=" << a.index
                      << " label=" << a.semantic_label
                      << " reliable=" << a.reliable
                      << " residual=" << a.residual
                      << " x=" << a.source.x()
                      << " y=" << a.source.y()
                      << " z=" << a.source.z()
                      << RESET << std::endl;
        }
        if (anchor_points.size() > print_num) {
            std::cout << GREEN << "[ Anchor ] ... omitted=" << (anchor_points.size() - print_num)
                      << RESET << std::endl;
        }
    }
    
    // Spit the final transformation
    return T_icp * initial_guess;
}

}   // namespace graph_slam
