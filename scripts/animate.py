## Conventions: scalar-first (w,x,y,z), q rotates body -> inertial.

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

PROJECT_ROOT = Path(__file__).resolve().parents[1]

CORNERS = np.array([[sx, sy, sz] for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)])
FACES = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]

WALL_FPS = 30

def quat_to_matrix(qw, qx, qy, qz):
    s = 1.0 / (qw * qw + qx * qx + qy * qy + qz * qz)
    return np.array([
        [1 - 2*s*(qy*qy + qz*qz), 2*s*(qx*qy - qz*qw),     2*s*(qx*qz + qy*qw)],
        [2*s*(qx*qy + qz*qw),     1 - 2*s*(qx*qx + qz*qz), 2*s*(qy*qz - qx*qw)],
        [2*s*(qx*qz - qy*qw),     2*s*(qy*qz + qx*qw),     1 - 2*s*(qx*qx + qy*qy)],
    ])


parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("csv_filepath", help="attitude log, relative to project root")
parser.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier (default: 1 = real time)")
parser.add_argument("--size", nargs=3, type=float, metavar=("A", "B", "C"), default=[1.0, 0.7, 0.4], help="box side lengths")
parser.add_argument("--inertia", nargs=3, type=float, metavar=("I1", "I2", "I3"), help="principal moments of inertia; enables the L arrow")
parser.add_argument("--save", metavar="NAME", help="write a GIF to docs/plots/NAME instead of showing")
args = parser.parse_args()

csv_path = Path(args.csv_filepath)
if not csv_path.is_absolute():
    csv_path = PROJECT_ROOT / csv_path
if not csv_path.exists():
    parser.error(f"no such file: {csv_path}")

df = pd.read_csv(csv_path, skipinitialspace=True)
df.columns = [c.strip() for c in df.columns]

dt = float(df["t"].iloc[1] - df["t"].iloc[0])
stride = max(1, round(args.speed / (WALL_FPS * dt)))
df = df.iloc[::stride].reset_index(drop=True)
interval_ms = stride * dt * 1000.0 / args.speed

times = df["t"].to_numpy()
quats = df[["qw", "qx", "qy", "qz"]].to_numpy()
corners = CORNERS * (np.array(args.size) / 2.0 / max(args.size))

l_body = None
if args.inertia:
    if not {"wx", "wy", "wz"} <= set(df.columns): parser.error("--inertia needs wx,wy,wz columns in the CSV")
    l_body = df[["wx", "wy", "wz"]].to_numpy() * np.array(args.inertia)
    l_norms = np.linalg.norm(l_body, axis=1)
    l_body = l_body / max(l_norms.max(), 1e-12)

fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(projection="3d")
ax.set(xlim=(-1.2, 1.2), ylim=(-1.2, 1.2), zlim=(-1.2, 1.2), xlabel="x (inertial)", ylabel="y (inertial)", zlabel="z (inertial)")
ax.set_box_aspect([1, 1, 1])

box = Poly3DCollection([], facecolor="tab:orange", edgecolor="dimgray", alpha=0.13, linewidths=0.8, zorder=0)
ax.add_collection3d(box)

axes = [ax.plot([], [], [], color=c, lw=2.5, marker="o", markevery=[-1], label=l, zorder=5)[0] 
        for c, l in zip(("tab:red", "tab:green", "tab:blue"), ("body x", "body y", "body z"))]
l_line = None
if l_body is not None:
    l_line, = ax.plot([], [], [], color="black", lw=2.0, ls="--", marker="o", markevery=[-1], label="L (scaled)", zorder=6)

clock = ax.text2D(0.02, 0.97, "", transform=ax.transAxes, family="monospace", va="top")
ax.legend(loc="upper right", fontsize=9)


def update(k):
    R = quat_to_matrix(*quats[k])
    for j, line in enumerate(axes):
        tip = R[:, j]
        line.set_data([0, tip[0]], [0, tip[1]])
        line.set_3d_properties([0, tip[2]])
    v = corners @ R.T
    box.set_verts([[v[i] for i in face] for face in FACES])

    text = f"t   = {times[k]:8.2f} s"
    if l_line is not None:
        tip = R @ l_body[k]
        l_line.set_data([0, tip[0]], [0, tip[1]])
        l_line.set_3d_properties([0, tip[2]])
        text += f"\n|L| = {l_norms[k]:.12f}"
    clock.set_text(text)
    return ()


anim = FuncAnimation(fig, update, frames=len(df), interval=interval_ms, blit=False)

if args.save:
    out = PROJECT_ROOT / "docs" / "plots" / args.save
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=1000.0 / interval_ms))
    print(f"wrote {out}  ({len(df)} frames)")
else:
    plt.show()
