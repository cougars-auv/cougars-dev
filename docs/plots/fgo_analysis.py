# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.18.1
#   kernelspec:
#     display_name: .venv
#     language: python
#     name: python3
# ---

# %%
import copy
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader

from evo.tools import file_interface, plot
from evo.tools import settings
from evo.core import sync, metrics
from evo.core.metrics import PoseRelation

# %%
# Configuration
BAG_PATH = "../../bags/name"
TRUTH_TOPIC = "/auv0/odometry/truth"
ESTIMATE_TOPICS = [
    "/auv0/odometry/global",
    "/auv0/odometry/global_tm",
    "/auv0/odometry/global_ekf",
    "/auv0/odometry/global_ukf",
    "/auv0/odometry/global_iekf",
]
OUTPUT_DIR = Path(BAG_PATH)
print(f"Saving results to: {OUTPUT_DIR.resolve()}")


# %%
def load_trajectories(bag_path, truth_topic, estimate_topics):
    data_pairs = {}
    try:
        with Reader(bag_path) as reader:
            traj_ref_raw = file_interface.read_bag_trajectory(reader, truth_topic)

            for est_topic in estimate_topics:
                try:
                    traj_est = file_interface.read_bag_trajectory(reader, est_topic)
                    
                    traj_ref_synced, traj_est_synced = sync.associate_trajectories(traj_ref_raw, traj_est, max_diff=0.1)

                    # No alignment needed when using HoloOcean ground truth
                    # traj_est_synced.align(traj_ref_synced, correct_scale=False, correct_only_scale=False)

                    traj_est_aligned = copy.deepcopy(traj_est_synced)
                    
                    data_pairs[est_topic] = (traj_ref_synced, traj_est_aligned)
                    
                except Exception as e:
                    print(f"Skipping {est_topic}: {e}")
                    continue
    except Exception as e:
        print(f"Error opening bag: {e}")
        
    return data_pairs, traj_ref_raw

def calculate_metrics(data_pairs):
    results = []
    
    for name, (traj_ref, traj_est) in data_pairs.items():
        row = {"Topic": name}
        
        # APE (Absolute Pose Error)
        for rel, metric_name in [(PoseRelation.translation_part, "APE-Trans"), (PoseRelation.rotation_angle_deg, "APE-Rot")]:
            metric = metrics.APE(rel)
            metric.process_data((traj_ref, traj_est))
            row[metric_name] = metric.get_statistic(metrics.StatisticsType.rmse)
            
        # RPE (Relative Pose Error)
        for rel, metric_name in [(PoseRelation.translation_part, "RPE-Trans"), (PoseRelation.rotation_angle_deg, "RPE-Rot")]:
            metric = metrics.RPE(rel)
            metric.process_data((traj_ref, traj_est))
            row[metric_name] = metric.get_statistic(metrics.StatisticsType.rmse)
            
        results.append(row)
        
    return pd.DataFrame(results).set_index("Topic")


# %%
trajectory_pairs, truth_ref_raw = load_trajectories(BAG_PATH, TRUTH_TOPIC, ESTIMATE_TOPICS)
df_metrics = calculate_metrics(trajectory_pairs)

# %%
csv_path = OUTPUT_DIR / "metrics.csv"
df_metrics.to_csv(csv_path)
print(f"Metrics saved to: {csv_path}")
display(df_metrics)

# %%
settings.SETTINGS.plot_seaborn_style = "whitegrid"
settings.SETTINGS.plot_usetex = True
settings.SETTINGS.plot_fontfamily = "serif"
plot.apply_settings(settings.SETTINGS)

plot_mode = plot.PlotMode.xy 
# plot_mode = plot.PlotMode.xz
# plot_mode = plot.PlotMode.yz
# plot_mode = plot.PlotMode.xyz

fig = plt.figure(figsize=(10, 8))
ax = plot.prepare_axis(fig, plot_mode)

plot.traj(ax, plot_mode, truth_ref_raw, style="--", color="black", label=TRUTH_TOPIC)
est_trajectories = {k: v[1] for k, v in trajectory_pairs.items()}
plot.trajectories(ax, est_trajectories, plot_mode)

plot_path = OUTPUT_DIR / "trajectories.png"
plt.savefig(plot_path, dpi=300)
print(f"Plot saved to: {plot_path.resolve()}")
display(fig)