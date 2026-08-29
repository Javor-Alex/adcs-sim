import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

def from_root(p):
    p = Path(p)
    return p if p.is_absolute() else PROJECT_ROOT / p

parser = argparse.ArgumentParser(description="given a csv file, plot one column against one (or multiple) other column(s)")

parser.add_argument("csv_filepath", type=str, help="filepath of the CSV file relative to adcs-sim directory")
parser.add_argument("-x", type=str, default="t", help="the x-axis values of the plot")
parser.add_argument("-y", nargs="+", default=["qw"], help="one or more y-axis values")

parser.add_argument("-r", "--reference", type=str, help="optional reference curve. e: exp(-t) cos: cos(t/2)")

parser.add_argument("-s", "--save", type=str, default=None, help="type a name and it will be stored in docs\\plots")

parser.add_argument("--xlabel", type=str, default=None, help="x-axis label, e.g. \"t [s]\"")
parser.add_argument("--ylabel", type=str, default=None, help="y-axis label, e.g. \"omega [rad/s]\"")
parser.add_argument("--ylim", nargs=2, type=float, default=None, metavar=("MIN", "MAX"), help="fix the y-axis range so plots are comparable")

args = parser.parse_args()

csv_path = from_root(args.csv_filepath)
if not csv_path.exists():
    parser.error(f"no such file: {csv_path}")
df = pd.read_csv(csv_path)

x = df[args.x]
y = df[args.y]

# change maybe:
plt.style.use("ggplot")

fig, ax = plt.subplots(figsize=(12,6))

for col in args.y:
    ax.plot(x, df[col], marker="o" if len(x) < 500 else None, label=col)

if args.reference:
    if args.reference == "e":
        ax.plot(x, np.exp(-x), "g--", label="exp(t)")
    if args.reference == "cos":
        ax.plot(x, np.cos(x/2), "g--", label="cos(t/2)")

ax.set_xlabel(args.xlabel or f"{args.x} values")
ax.set_ylabel(args.ylabel or ", ".join(args.y) + " values")

if args.ylim:
    ax.set_ylim(*args.ylim)

ax.legend()

ax.grid(True, alpha=0.3)

if args.save:
    save_path = PROJECT_ROOT / "docs" / "plots" / args.save
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
else:
    plt.show()
