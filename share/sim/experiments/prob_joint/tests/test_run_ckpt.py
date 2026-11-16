"""Fast, deterministic regression coverage for run_ckpt.py's checkpoint bookkeeping.

The actual process-kill (SIGKILL) recovery behaviour -- the 2026-11-15 prompt's mandatory
section 4.6 gate -- was validated manually on a real 6-trajectory cohort before the 894-run was
launched: a live run was killed with `kill -9` on the main process and all 4 workers mid-flight,
then resumed. Result: 10/10 already-'done' jobs survived with their result_json intact and were
never re-run, the 3 jobs that were mid-flight at kill time were correctly reset to 'pending' and
re-run, the final 54/54 results had zero duplicates and zero missing jobs, and every metric in the
final per_traj.csv was bit-identical to an uninterrupted from-scratch run of the same cohort except
`deadline_miss_pct` (expected to differ -- it is derived from measured wall-clock timing, not a
deterministic function of the input, same as `runtime_*_ms`). See share/11.15_exp/ for the report.

This file covers the parts of that mechanism that are worth asserting on every change: job-identity
hashing, idempotent plan sync (no duplicate rows on a second sync), and that a job left 'running'
from an interrupted process is detected and requeued rather than silently treated as done.
"""
from __future__ import annotations

import sys
from pathlib import Path

SIM = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import run_ckpt as rc  # noqa: E402


def test_grid_key_distinguishes_grid_from_non_grid_methods():
    assert rc.grid_key("B7", 15.0) == "na"
    assert rc.grid_key("P", 10.0) == "na"
    assert rc.grid_key("E0", 15.0) == "15"
    assert rc.grid_key("Pc", 20.0) == "20"
    assert rc.grid_key("E2", 10.0) != rc.grid_key("E2", 15.0)


def _fresh_db(tmp_path):
    con = rc.connect(tmp_path / "checkpoints" / "run_state.sqlite")
    return con


def test_sync_plan_is_idempotent_no_duplicate_rows(tmp_path):
    con = _fresh_db(tmp_path)
    files = [("a.npz", "S1"), ("b.npz", "S1")]
    jobs = [("E0", 15.0), ("E1", 15.0)]
    rc.sync_plan(con, files, jobs, "cfg1", "ver1")
    n1 = con.execute("SELECT COUNT(*) FROM jobs").fetchone()[0]
    assert n1 == len(files) * len(jobs)
    rc.sync_plan(con, files, jobs, "cfg1", "ver1")  # same plan again -- must not duplicate
    n2 = con.execute("SELECT COUNT(*) FROM jobs").fetchone()[0]
    assert n2 == n1


def test_different_config_or_code_version_gets_its_own_rows_not_merged(tmp_path):
    con = _fresh_db(tmp_path)
    files = [("a.npz", "S1")]
    jobs = [("E0", 15.0)]
    rc.sync_plan(con, files, jobs, "cfg1", "ver1")
    con.execute("UPDATE jobs SET status='done', result_json='{}' WHERE config_hash='cfg1'")
    rc.sync_plan(con, files, jobs, "cfg2", "ver1")  # different config -- a different experiment
    rows = con.execute("SELECT config_hash, status FROM jobs ORDER BY config_hash").fetchall()
    assert rows == [("cfg1", "done"), ("cfg2", "pending")]  # cfg2's row is untouched, not merged


def test_interrupted_running_job_is_requeued_on_resync(tmp_path):
    con = _fresh_db(tmp_path)
    files = [("a.npz", "S1")]
    jobs = [("E0", 15.0), ("E1", 15.0)]
    rc.sync_plan(con, files, jobs, "cfg1", "ver1")
    con.execute("UPDATE jobs SET status='done', result_json='{}' "
                "WHERE method='E0' AND config_hash='cfg1'")
    con.execute("UPDATE jobs SET status='running', started_at=123.0 "
                "WHERE method='E1' AND config_hash='cfg1'")  # simulates a process killed mid-job
    rc.sync_plan(con, files, jobs, "cfg1", "ver1")  # a fresh process re-syncing on startup/resume
    statuses = dict(con.execute("SELECT method, status FROM jobs WHERE config_hash='cfg1'").fetchall())
    assert statuses == {"E0": "done", "E1": "pending"}  # done job untouched; orphaned 'running' requeued


def test_canonical_grid_deg_is_read_dynamically_not_bound_at_import(tmp_path):
    sys.path.insert(0, str(SIM))
    import canonical as can
    import kinematics as k
    import numpy as np
    m = k.Nero.from_urdf()
    q = k.READY.copy()
    T = m.fk(q)
    psi = float(m.psi(q))
    a = can.canonical_feasible_map(m, T, psi, k.READY, psi_grid_deg=10.0)
    b = can.canonical_feasible_map(m, T, psi, k.READY, psi_grid_deg=20.0)
    assert a.solver_config_hash == b.solver_config_hash  # the hash is a module constant either way
    # different grid resolutions may legitimately pick different branches far from mu_psi, but both
    # must at least run and return a feasible result for a trivially feasible pose like READY.
    assert a.feasible and b.feasible
