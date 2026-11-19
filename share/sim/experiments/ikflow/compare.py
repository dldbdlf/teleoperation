"""Merge B9 rows with the B0-B8 per_traj.csv and print the harness summary table.
Usage: compare.py <baselines per_traj.csv> <b9 per_traj.csv> [out.md]"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # share/sim
import run_baselines as rb  # noqa: E402


def read(p):
    with open(p) as f:
        return [{kk: (v if kk in ("group", "file", "method") else float(v)) for kk, v in r.items()}
                for r in csv.DictReader(f)]


rows = read(sys.argv[1]) + read(sys.argv[2])
rb.METHODS = rb.METHODS + ["B9", "B9r"]
names = {m: m for m in rb.METHODS} | {"B9": "B9 IKFlow+투영", "B9r": "B9r IKFlow raw"}
md = rb.summary(rows, names)
print(md)
if len(sys.argv) > 3:
    open(sys.argv[3], "w").write(md)
