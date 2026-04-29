#!/usr/bin/env python3
"""Parameter sweep: generate launch files, run SLAM, evaluate ATE."""
import subprocess, os, sys, time, shutil, re

LAUNCH_TEMPLATE = "/home/user/catkin_ws/src/SG-SLAM/ros/launch/semgraph_slam_mulran.launch"
SAVE_DIR = "/home/user/catkin_ws/src/SG-SLAM/ros/save"
GT_FILE = "/data/00/kaist01.txt"
EVAL_SCRIPT = "/home/user/catkin_ws/src/SG-SLAM/eval/traj_eval.py"

def read_launch():
    with open(LAUNCH_TEMPLATE) as f:
        return f.read()

def set_param(xml, name, value):
    pattern = rf'(<param\s+name="{name}"\s+value=")([^"]*)"'
    new_xml, n = re.subn(pattern, rf'\g<1>{value}"', xml)
    if n == 0:
        insert_before = '<!-- relozalization params -->'
        new_xml = xml.replace(insert_before,
            f'    <param name="{name}" value="{value}" />\n    {insert_before}')
    return new_xml

def run_experiment(name, params):
    print(f"\n{'='*60}\n  Experiment: {name}\n{'='*60}")
    xml = read_launch()
    out_path = f"{SAVE_DIR}/sweep_{name}.txt"
    xml = set_param(xml, "pgo_result_path", out_path)
    xml = set_param(xml, "result_path", f"{SAVE_DIR}/sweep_{name}_odom.txt")
    for k, v in params.items():
        xml = set_param(xml, k, str(v))
    
    # Disable visualization
    xml = xml.replace('default="true"/>', 'default="false"/>', 1)
    
    tmp_launch = f"/tmp/sweep_{name}.launch"
    with open(tmp_launch, "w") as f:
        f.write(xml)
    
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    
    print(f"  Running SLAM...")
    t0 = time.time()
    proc = subprocess.run(
        ["bash", "-c", f"source /home/user/catkin_ws/devel/setup.bash && "
         f"roslaunch {tmp_launch} --wait 2>&1 | tail -5"],
        timeout=1200, capture_output=True, text=True, env=env)
    elapsed = time.time() - t0
    print(f"  SLAM finished in {elapsed:.0f}s")
    
    if not os.path.exists(out_path):
        print(f"  ERROR: output file not found")
        return None
    
    lines = sum(1 for _ in open(out_path))
    print(f"  Output poses: {lines}")
    
    result = subprocess.run(
        ["python3", EVAL_SCRIPT, "--gt_file", GT_FILE, "--pred_file", out_path],
        capture_output=True, text=True, env=env)
    print(result.stdout)
    
    ate_match = re.search(r"Absoulte Trajectory Error\s+\(m\):\s+([\d.]+)", result.stdout)
    ate = float(ate_match.group(1)) if ate_match else None
    
    trans_match = re.search(r"Average Translation Error\s+\(%\):\s+([\d.]+)", result.stdout)
    trans = float(trans_match.group(1)) if trans_match else None
    
    return {"ate": ate, "trans": trans, "name": name}

configs = {
    # Config B: tighter voxel + more loop candidates
    "B_fine_voxel": {
        "voxel_size": 0.8,
        "max_points_per_voxel": 25,
        "loop_candidate": 8,
        "back_sim_th": 0.25,
        "frame_acc_pgo": 15,
    },
    # Config C: more aggressive arbitration + denser loop
    "C_arb_aggressive": {
        "arbitration_alpha": 0.3,
        "arbitration_dynamic_alpha": 0.15,
        "arbitration_geo_sigma": 0.8,
        "arbitration_anchor_k": 6,
        "arbitration_warmup_iterations": 2,
        "back_sim_th": 0.25,
        "keyframe_interval": 3,
        "frame_acc_pgo": 15,
    },
    # Config D: combine fine voxel + aggressive arb + loop
    "D_combined": {
        "voxel_size": 0.8,
        "max_points_per_voxel": 25,
        "arbitration_alpha": 0.35,
        "arbitration_dynamic_alpha": 0.15,
        "arbitration_geo_sigma": 0.8,
        "arbitration_anchor_k": 6,
        "arbitration_warmup_iterations": 2,
        "back_sim_th": 0.25,
        "loop_candidate": 8,
        "keyframe_interval": 3,
        "frame_acc_pgo": 15,
        "max_distance_for_loop": 0.2,
    },
    # Config E: just improve loop closure, no arb changes
    "E_loop_only": {
        "back_sim_th": 0.22,
        "graph_sim_th": 0.45,
        "loop_candidate": 10,
        "keyframe_interval": 3,
        "frame_acc_pgo": 10,
        "max_distance_for_loop": 0.22,
    },
}

if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else None
    results = []
    
    for name, params in configs.items():
        if target and name != target:
            continue
        r = run_experiment(name, params)
        if r:
            results.append(r)
            print(f"  >>> {name}: ATE = {r['ate']:.4f}, Trans = {r['trans']:.4f}")
    
    if results:
        print(f"\n{'='*60}")
        print("  SUMMARY (sorted by ATE)")
        print(f"{'='*60}")
        for r in sorted(results, key=lambda x: x["ate"] or 999):
            print(f"  {r['name']:25s}  ATE={r['ate']:.4f}  Trans={r['trans']:.4f}")
