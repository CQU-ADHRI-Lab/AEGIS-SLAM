#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from eval_traj_utils import (
    read_kitti_format_poses,
    read_kitti_format_calib,
    apply_kitti_format_calib,
    absolute_error,
    align_traj,
)
import os
import glob

GT_FILE = "/data/kitti/sequences/08/poses.txt"
CALIB_FILE = "/data/kitti/sequences/08/calib.txt"
OUTPUT_DIR = "/data/best_result_plots"

METHODS = {
    "Ground Truth": {
        "file": GT_FILE,
        "needs_calib": False,
        "color": "#2c2c2c",
        "linestyle": "--",
        "linewidth": 2.0,
        "zorder": 1,
    },
    "Ours": {
        "file": "/data/best_result_ours_table/kitti/kitti_08.txt",
        "needs_calib": True,
        "color": "#e74c3c",
        "linestyle": "-",
        "linewidth": 1.8,
        "zorder": 6,
    },
    "SG-SLAM": {
        "file": "/home/user/catkin_ws/src/SG-SLAM-baseline/ros/save/mislabel_exp/kitti08_mislabel0_slam.txt",
        "needs_calib": True,
        "color": "#3498db",
        "linestyle": "-",
        "linewidth": 1.5,
        "zorder": 5,
    },
    "MULLS": {
        "file": "/data/best_result_mulls_repro/kitti08_mulls_pose_l_lo.txt",
        "needs_calib": True,
        "color": "#27ae60",
        "linestyle": "-",
        "linewidth": 1.5,
        "zorder": 4,
    },
    "SuMA++": {
        "file": "/data/best_result_suma_repro/kitti08_suma_semantic_labels.txt",
        "needs_calib": False,
        "color": "#9b59b6",
        "linestyle": "-",
        "linewidth": 1.5,
        "zorder": 3,
    },
    "PIN-SLAM": {
        "file": None,  # 自动查找
        "needs_calib": False,
        "color": "#e67e22",
        "linestyle": "-",
        "linewidth": 1.5,
        "zorder": 2,
    },
}


def find_pin_slam_result():
    candidates = glob.glob(
        "/data/best_result_pin_slam_repro/pin_kitti08_repro_*/slam_poses_kitti.txt"
    )
    if candidates:
        candidates.sort(key=os.path.getmtime, reverse=True)
        return candidates[0]
    return None


def load_trajectory(name, cfg, calib):
    filepath = cfg["file"]
    if filepath is None:
        return None, None
    if not os.path.exists(filepath):
        print(f"  [跳过] {name}: 文件不存在 {filepath}")
        return None, None
    poses = read_kitti_format_poses(filepath)
    n = len(poses)
    if n == 0:
        return None, None
    poses_np = np.array(poses)
    if cfg["needs_calib"]:
        poses_np = np.array(apply_kitti_format_calib(list(poses_np), calib["Tr"]))
    return poses_np, n


def main():
    pin_path = find_pin_slam_result()
    if pin_path:
        METHODS["PIN-SLAM"]["file"] = pin_path
        print(f"找到 PIN-SLAM 结果: {pin_path}")
    else:
        print("PIN-SLAM 结果尚未生成，将跳过")

    calib = read_kitti_format_calib(CALIB_FILE)
    gt_poses = np.array(read_kitti_format_poses(GT_FILE))
    gt_n = gt_poses.shape[0]
    print(f"Ground Truth: {gt_n} 帧")

    trajectories = {}
    ate_values = {}

    for name, cfg in METHODS.items():
        if name == "Ground Truth":
            trajectories[name] = gt_poses
            continue

        poses_np, n = load_trajectory(name, cfg, calib)
        if poses_np is None:
            continue

        if n != gt_n:
            print(f"  [警告] {name}: 帧数 {n} != GT {gt_n}，取前 {min(n, gt_n)} 帧")
            min_n = min(n, gt_n)
            poses_np = poses_np[:min_n]
            gt_eval = gt_poses[:min_n]
        else:
            gt_eval = gt_poses

        _, ate, align_mat = absolute_error(gt_eval, poses_np)
        ate_values[name] = ate
        print(f"  {name}: ATE = {ate:.3f} m ({n} 帧)")

        aligned_poses = []
        for i in range(poses_np.shape[0]):
            aligned_poses.append(align_mat @ poses_np[i])
        trajectories[name] = np.array(aligned_poses)

    # --- 绘图 ---
    plt.rcParams.update({
        "font.size": 13,
        "axes.labelsize": 15,
        "axes.titlesize": 17,
        "legend.fontsize": 12,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
        "lines.antialiased": True,
        "figure.dpi": 150,
        "savefig.facecolor": "white",
    })

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    draw_order = ["Ground Truth", "MULLS", "PIN-SLAM", "SuMA++", "SG-SLAM", "Ours"]

    for name in draw_order:
        if name not in trajectories or name not in METHODS:
            continue
        cfg = METHODS[name]
        traj = trajectories[name]
        x = traj[:, 0, 3]
        z = traj[:, 2, 3]

        label = name
        if name in ate_values:
            label = f"{name} (ATE={ate_values[name]:.2f}m)"

        ax.plot(
            x, z,
            color=cfg["color"],
            linestyle=cfg["linestyle"],
            linewidth=cfg["linewidth"],
            label=label,
            zorder=cfg["zorder"],
            alpha=0.95 if name != "Ground Truth" else 1.0,
        )

    start = gt_poses[0]
    ax.plot(start[0, 3], start[2, 3], "k^", markersize=10, zorder=10, label="Start")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")
    ax.set_title("KITTI Sequence 08 — Trajectory Comparison")
    ax.legend(loc="upper left", framealpha=0.92, edgecolor="#cccccc",
              fancybox=False, borderpad=0.6)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2, linestyle=":")
    ax.tick_params(direction="in")

    for spine in ax.spines.values():
        spine.set_linewidth(0.8)

    plt.tight_layout()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_png = os.path.join(OUTPUT_DIR, "kitti08_trajectory_comparison.png")
    out_pdf = os.path.join(OUTPUT_DIR, "kitti08_trajectory_comparison.pdf")
    plt.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.savefig(out_pdf, bbox_inches="tight")
    print(f"\n图片已保存: {out_png}")
    print(f"PDF 已保存: {out_pdf}")


if __name__ == "__main__":
    main()
