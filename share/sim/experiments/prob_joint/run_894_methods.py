#!/usr/bin/env python3
"""Register E0-E3 in the EXISTING EX04 run_baselines harness without altering B0-B8/P/Pc.

Linux Jetson uses fork, so the patched method registry is inherited by the
multiprocessing Pool. Do not use spawn without moving the registration to a
normal module imported by children.

Example from share/sim:
  EX04_PROB_PSI_GRID_DEG=15 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
   python -u experiments/prob_joint/run_894_methods.py \
   outputs/ex04_prob_894/cohort_data --psi-map mirror \
   --methods B7 B8 P B7c Pc E0 E1 E2 E3 \
   --out outputs/ex04_prob_894/full_15deg

Fail explicitly if the user's locally implemented P/Pc is absent. P/Pc code is
not currently on public main: never alias it to another baseline.
"""
from __future__ import annotations
import os
for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[key] = "1"  # BEFORE importing numpy through either module
import multiprocessing
import sys
from pathlib import Path
SIM = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(SIM))
import run_baselines as rb
from prob_methods import ProbJointBranch, METHODS

def main():
    if multiprocessing.get_start_method(allow_none=True) not in (None, "fork"):
        raise SystemExit("Only Linux fork workers are supported by this compatibility wrapper")
    original_make = rb.make
    original_registered = list(rb.METHODS)
    requested = [sys.argv[i+1] for i in range(len(sys.argv)-1) if sys.argv[i]=="--methods"]
    if requested:
        print("Methods are parsed by the original run_baselines.main.",flush=True)
    def extended_make(mid, net):
        if mid in METHODS:
            return ProbJointBranch(mid,float(os.environ.get("EX04_PROB_PSI_GRID_DEG","15")))
        return original_make(mid,net)
    rb.make = extended_make
    rb.METHODS = original_registered + [mid for mid in METHODS if mid not in original_registered]
    # Fail if any of the user's original methods were not actually registered.
    if "--methods" in sys.argv:
        start = sys.argv.index("--methods")+1
        end = next((i for i in range(start,len(sys.argv)) if sys.argv[i].startswith("--")),len(sys.argv))
        names = sys.argv[start:end]
        not_installed=[x for x in names if x not in rb.METHODS]
        if not_installed:
            raise SystemExit(f"Missing local original method implementations: {not_installed}. "
                             "Push/merge the Jetson P and Pc branch before a full fair comparison.")
    rb.main()

if __name__=="__main__":main()
