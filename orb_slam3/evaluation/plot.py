"""
Plot two 3D trajectories from text files.
Default files: ground_truth.txt and odometry.txt (columns: ts, x, y, z, ...)
Uses columns 2-4 (indices 1,2,3).
If plotly is installed it will open an interactive browser view, otherwise it falls back to matplotlib (interactive window).
Run:
  python3 plot_trajectories.py
Or:
  python3 plot_trajectories.py --gt my_gt.txt --odo my_odo.txt
"""

import argparse
import numpy as np
import os
import webbrowser

parser = argparse.ArgumentParser()
parser.add_argument('--gt', default='ground_truth.txt', help='ground truth file (default: ground_truth.txt)')
parser.add_argument('--odo', default='odometry.txt', help='odometry file (default: odometry.txt)')
parser.add_argument('--out-html', default=None, help='save interactive html (plotly) to file')
args = parser.parse_args()

def load_xyz(path):
    if not os.path.exists(path):
        raise SystemExit(f"File not found: {path}")
    # assume text columns: ts x y z ... -> we want cols 1,2,3 (0-based)
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] <= 3:
        raise SystemExit(f"File {path} has too few columns (need at least 4: ts x y z ...)")
    return data[:, 1:4]

gt = load_xyz(args.gt)
odo = load_xyz(args.odo)

# Try plotly first for a rich movable 3D view
try:
    import plotly.graph_objects as go
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(x=gt[:,0], y=gt[:,1], z=gt[:,2],
                               mode='lines+markers', name='ground_truth',
                               marker=dict(size=2), line=dict(width=2, color='green')))
    fig.add_trace(go.Scatter3d(x=odo[:,0], y=odo[:,1], z=odo[:,2],
                               mode='lines+markers', name='odometry',
                               marker=dict(size=2), line=dict(width=2, color='red')))
    fig.update_layout(
        scene=dict(xaxis_title='X', yaxis_title='Y', zaxis_title='Z', aspectmode='auto'),
        title='Trajectories: ground_truth vs odometry',
        legend=dict(x=0.02, y=0.98),
        margin=dict(l=0, r=0, t=40, b=0),
        width=1000, height=700
    )
    if args.out_html:
        fig.write_html(args.out_html)
        print(f"Saved interactive html to {args.out_html}")
        webbrowser.open(args.out_html)
    else:
        fig.show()
    raise SystemExit(0)
except Exception:
    # fall back to matplotlib
    pass

# Matplotlib fallback
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(10,7))
ax = fig.add_subplot(111, projection='3d')
ax.plot(gt[:,0], gt[:,1], gt[:,2], label='ground_truth', color='green', linewidth=1)
ax.plot(odo[:,0], odo[:,1], odo[:,2], label='odometry', color='red', linewidth=1)
ax.scatter(gt[0,0], gt[0,1], gt[0,2], color='green', marker='o', s=40, label='gt start')
ax.scatter(odo[0,0], odo[0,1], odo[0,2], color='red', marker='o', s=40, label='odo start')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Trajectories: ground_truth vs odometry')
ax.legend()
plt.tight_layout()
plt.show()