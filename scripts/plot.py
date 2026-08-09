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
parser.add_argument("-y", nargs="+", default=["x"], help="one or more y-axis values")

parser.add_argument("-r", "--reference", type=str, help="optional reference curve. e: exp(-t)")

parser.add_argument("-s", "--save", type=str, default=None, help="type a name and it will be stored in docs\\plots")

args = parser.parse_args()

csv_path = from_root(args.csv_filepath)
if not csv_path.exists():
    parser.error(f"no such file: {csv_path}")
df = pd.read_csv(csv_path)

x = df[args.x]
y = df[args.y]

# change maybe:
plt.style.use("ggplot")

fig, ax = plt.subplots(figsize=(8,5))

for col in args.y:
    ax.plot(x, df[col], marker="o" if len(x) < 500 else None, linestyle=":", label=col)

if args.reference:
    if args.reference == "e":
        ax.plot(x, np.exp(-x), "g--", label=args.reference)

ax.set_xlabel(f"{args.x} values")
ax.set_ylabel(f"{", ".join(args.y)} values")
ax.legend()

ax.grid(True, alpha=0.3)

if args.save:
    save_path = PROJECT_ROOT / "docs" / "plots" / args.save
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
else:
    plt.show()
