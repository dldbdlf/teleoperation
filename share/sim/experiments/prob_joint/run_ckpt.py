#!/usr/bin/env python3
"""Checkpointed, resumable, ETA-reporting runner for the EX04 894-trajectory revalidation
(2026-11-15 prompt, section 4): B7/B8/P/B7c/Pc (existing) + E0-E3 (prob_methods.py) across
3 psi-grid resolutions, on the exact 894-trajectory cohort from prepare_894.py.

Job identity: (file, method, grid, config_hash, code_version). Every job's result is committed
to a SQLite (WAL) database the instant it finishes, independent of the other jobs in its file and
independent of a clean shutdown -- a SIGKILL only ever loses work that was *in flight* (at most
one job per worker process) at the instant of the kill, never a job that had already completed.
`targets()` (the per-file oracle/shared setup, expensive on its own -- see canonical.py's module
docstring) is computed once per file and reused for every (method, grid) job on that file, since
it does not depend on either.

Usage:
  python3 run_ckpt.py PLAN --cohort <cohort_dir> --out <out_dir> [--workers N] [--resume]
  python3 run_ckpt.py --status --out <out_dir>

PLAN is one of: stage15 (B7 B8 P B7c Pc E0 E1 E2 E3 @ 15deg), stage10 (E0-E3 @ 10deg),
stage20 (E0-E3 @ 20deg), or 'all' (all three, one DB / one progress stream).

--resume is accepted for interface compliance with the 2026-11-15 prompt; it is a no-op because
resuming is the unconditional default (every invocation skips whatever is already 'done' for the
current config_hash/code_version -- there is no separate from-scratch mode).
"""
from __future__ import annotations

import os
for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_k, "1")

import argparse
import collections
import csv
import hashlib
import json
import multiprocessing as mp
import signal
import sqlite3
import subprocess
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import baselines as bl          # noqa: E402
import canonical as can         # noqa: E402
import kinematics as k          # noqa: E402
import run_baselines as rb      # noqa: E402
from prob_methods import ProbJointBranch  # noqa: E402

KST = timezone(timedelta(hours=9))
NON_GRID_METHODS = ("B7", "B8", "P")   # no psi-grid concept at all
GRID_METHODS = ("B7c", "Pc", "E0", "E1", "E2", "E3")
STAGE_PLANS = {
    "stage15": [(m, 15.0) for m in ("B7", "B8", "P", "B7c", "Pc", "E0", "E1", "E2", "E3")],
    "stage10": [(m, 10.0) for m in ("E0", "E1", "E2", "E3")],
    "stage20": [(m, 20.0) for m in ("E0", "E1", "E2", "E3")],
    # E0-E3 only, all 3 grids -- a supplementary pass (new prob_methods.py diagnostics: cand-gen/
    # selection timing split, branch entropy, branch-probability change) added after the main
    # 15198-job "all" run already completed; re-running B7/B8/P/B7c/Pc would gain nothing since
    # they have no branch-posterior concept and their main-run results already stand.
    "probdiag": [(m, g) for g in (10.0, 15.0, 20.0) for m in ("E0", "E1", "E2", "E3")],
    "fast15": [("E1", 15.0)],
    "fast20": [("E1", 20.0)],
}


def git_sha():
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=SIM).decode().strip()
    except Exception:
        return "unknown"


def grid_key(mid, grid):
    return "na" if mid in NON_GRID_METHODS else f"{grid:g}"


def make_method(mid, grid, fast_e1=False):
    if mid in ("E0", "E1", "E2", "E3"):
        return ProbJointBranch(mid, grid, fast_e1=fast_e1)
    if mid == "B7c":
        return bl.BeliefCanonicalIK(psi_grid_deg=grid)
    if mid == "Pc":
        return bl.FeasibleMAPCanonical(psi_grid_deg=grid)
    return rb.make(mid, None)


# -------------------------------------------------------------------------- SQLite checkpoint store
SCHEMA = """
CREATE TABLE IF NOT EXISTS jobs (
  file TEXT, scenario TEXT, method TEXT, grid TEXT, config_hash TEXT, code_version TEXT,
  status TEXT NOT NULL DEFAULT 'pending', attempts INTEGER NOT NULL DEFAULT 0,
  error TEXT, started_at REAL, finished_at REAL, result_json TEXT,
  PRIMARY KEY (file, method, grid, config_hash, code_version)
);
CREATE INDEX IF NOT EXISTS idx_jobs_status ON jobs(status);
CREATE TABLE IF NOT EXISTS completions (ts REAL, method TEXT, grid TEXT, scenario TEXT, duration REAL);
CREATE TABLE IF NOT EXISTS run_meta (key TEXT PRIMARY KEY, value TEXT);
"""


def connect(db_path):
    db_path.parent.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(str(db_path), timeout=60.0, isolation_level=None)
    con.execute("PRAGMA journal_mode=WAL;")
    con.execute("PRAGMA synchronous=NORMAL;")
    con.executescript(SCHEMA)
    return con


def set_meta(con, **kv):
    con.executemany("INSERT INTO run_meta(key,value) VALUES(?,?) "
                     "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
                     [(k_, json.dumps(v)) for k_, v in kv.items()])


def get_meta(con):
    return {k_: json.loads(v) for k_, v in con.execute("SELECT key,value FROM run_meta")}


# -------------------------------------------------------------------------- worker side
_W = {}  # per-worker-process globals


def _winit(queue, psi_map, qd_max, hold_s, fast_e1=False):
    # Workers must NOT inherit the main process's SIGTERM/SIGINT handler (fork duplicates it by
    # default): a custom handler that doesn't exit would make pool.terminate() hang forever
    # waiting for a worker that just keeps running its current job. Reset to default disposition
    # so terminate() actually kills them; only the main process needs to react to the signal.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    _W["q"] = queue
    _W["m"] = k.Nero.from_urdf()
    _W["psi_map"], _W["qd_max"], _W["hold_s"] = psi_map, qd_max, hold_s
    _W["fast_e1"] = bool(fast_e1)


def prob_diagnostics(meth):
    """Section 6 extras that only exist for ProbJointBranch (E0-E3) instances: IK-candidate-
    generation timing and posterior selection timing measured *separately* from the total solver
    time already in rb.metrics() (runtime_p50/p95/p99_ms, which wraps update()+step() together),
    plus branch-posterior entropy, frame-to-frame top-1 probability change, and candidate-bank
    size. B7/B8/P/B7c/Pc instances have none of these attributes, so every key is NaN for them --
    keeps a single CSV schema across all 9 methods without a separate file."""
    nan = float("nan")
    if not hasattr(meth, "entropy_hist"):  # B7/B8/P/B7c/Pc -- no branch-posterior concept at all
        return dict(cand_gen_p50_ms=nan, cand_gen_p95_ms=nan, cand_gen_p99_ms=nan,
                     selection_p50_ms=nan, selection_p95_ms=nan, selection_p99_ms=nan,
                     branch_entropy_p50=nan, branch_entropy_p90=nan,
                     branch_prob_change_p50=nan, branch_prob_change_p90=nan,
                     candidate_count_mean=nan, fallback_pct=nan)
    bank_ms = meth.bank_latency_ms
    sel_ms = meth.selection_latency_ms
    ent = meth.entropy_hist
    dprob = meth.prob_change_hist
    ncand = meth.candidate_count_hist
    if not bank_ms:  # every frame fell back before a bank was ever built (fully-occluded file)
        return dict(cand_gen_p50_ms=nan, cand_gen_p95_ms=nan, cand_gen_p99_ms=nan,
                     selection_p50_ms=nan, selection_p95_ms=nan, selection_p99_ms=nan,
                     branch_entropy_p50=nan, branch_entropy_p90=nan,
                     branch_prob_change_p50=nan, branch_prob_change_p90=nan,
                     candidate_count_mean=nan, fallback_pct=100.0)
    return dict(
        cand_gen_p50_ms=float(np.percentile(bank_ms, 50)), cand_gen_p95_ms=float(np.percentile(bank_ms, 95)),
        cand_gen_p99_ms=float(np.percentile(bank_ms, 99)),
        selection_p50_ms=float(np.percentile(sel_ms, 50)) if sel_ms else nan,
        selection_p95_ms=float(np.percentile(sel_ms, 95)) if sel_ms else nan,
        selection_p99_ms=float(np.percentile(sel_ms, 99)) if sel_ms else nan,
        branch_entropy_p50=float(np.percentile(ent, 50)) if ent else nan,
        branch_entropy_p90=float(np.percentile(ent, 90)) if ent else nan,
        branch_prob_change_p50=float(np.percentile(dprob, 50)) if dprob else nan,
        branch_prob_change_p90=float(np.percentile(dprob, 90)) if dprob else nan,
        candidate_count_mean=float(np.mean(ncand)) if ncand else nan,
        fallback_pct=100.0 * meth.fallback_count / max(1, len(bank_ms) + meth.fallback_count))


def _run_one(file_path, scenario, method, grid, config_hash, code_version, tgt, feas, best, q_oracle, meta_json):
    m, psi_map, qd_max, hold_s = _W["m"], _W["psi_map"], _W["qd_max"], _W["hold_s"]
    t0 = time.perf_counter()
    try:
        d = np.load(file_path, allow_pickle=True)
        meth = make_method(method, grid, fast_e1=_W.get('fast_e1', False))
        Q, Qreq, held, rt, is_update, invalid, branch_hist = rb.run(m, d, meth, psi_map, qd_max, hold_s)
        row = {"group": scenario, "file": Path(file_path).name, "method": method, "grid": grid_key(method, grid),
               **rb.metrics(m, d, Q, Qreq, held, rt, is_update, invalid, branch_hist,
                            tgt, feas, best, q_oracle, json.loads(meta_json), qd_max),
               **prob_diagnostics(meth)}
        dt = time.perf_counter() - t0
        key = (file_path, method, grid_key(method, grid), config_hash, code_version)
        _W["q"].put(("done", key, row, dt))
    except Exception as e:  # noqa: BLE001 -- one method's failure must not drop the rest of the file
        dt = time.perf_counter() - t0
        key = (file_path, method, grid_key(method, grid), config_hash, code_version)
        _W["q"].put(("failed", key, f"{type(e).__name__}: {e}", dt))


def _worker_file(args):
    file_path, scenario, pending, config_hash, code_version = args
    m, psi_map = _W["m"], _W["psi_map"]
    d = np.load(file_path, allow_pickle=True)
    meta_json = str(d["meta"])
    tgt, feas, best, q_oracle = rb.targets(m, d, psi_map)
    for method, grid in pending:
        key = (file_path, method, grid_key(method, grid), config_hash, code_version)
        _W["q"].put(("running", key, None, None))
        _run_one(file_path, scenario, method, grid, config_hash, code_version, tgt, feas, best, q_oracle, meta_json)
    return file_path, len(pending)


# -------------------------------------------------------------------------- planning
def config_hash_of(cohort_manifest_sha, psi_map, qd_max, hold_s, fast_e1=False):
    s = f"psi_map={psi_map} qd_max={qd_max!r} hold_s={hold_s!r} manifest={cohort_manifest_sha}"
    if fast_e1:
        s += " fast_e1=certified_prune_v1"
    return hashlib.sha1(s.encode()).hexdigest()[:12]


def build_plan(cohort_dir, stages):
    manifest = json.loads((cohort_dir / "manifest.json").read_text())
    jobs = STAGE_PLANS["all"] if stages == ["all"] else [j for s in stages for j in STAGE_PLANS[s]]
    files = [(str(cohort_dir / f["file"]), Path(f["file"]).parent.name) for f in manifest["files"]]
    return manifest, files, jobs


STAGE_PLANS["all"] = STAGE_PLANS["stage15"] + STAGE_PLANS["stage10"] + STAGE_PLANS["stage20"]


def sync_plan(con, files, jobs, config_hash, code_version):
    """Insert any (file,method,grid) job not already present for this exact
    (config_hash, code_version) as 'pending'. Never touches rows from a different
    config/code version -- those are a different experiment, not silently merged."""
    now = time.time()
    rows = [(f, scen, mid, grid_key(mid, grid), config_hash, code_version, "pending", 0, None, None, None, None)
            for f, scen in files for mid, grid in jobs]
    con.execute("BEGIN")
    con.executemany("INSERT OR IGNORE INTO jobs(file,scenario,method,grid,config_hash,code_version,"
                     "status,attempts,error,started_at,finished_at,result_json) VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
                     rows)
    # A row left 'running' from a previous crash was interrupted mid-flight -- never silently 'done'.
    con.execute("UPDATE jobs SET status='pending' WHERE status='running' AND config_hash=? AND code_version=?",
                (config_hash, code_version))
    con.execute("COMMIT")
    set_meta(con, plan_built_at=now, n_total=con.execute(
        "SELECT COUNT(*) FROM jobs WHERE config_hash=? AND code_version=?", (config_hash, code_version)
    ).fetchone()[0])


# -------------------------------------------------------------------------- ETA / progress
class Progress:
    def __init__(self, con, config_hash, code_version, out_dir, n_workers):
        self.con, self.config_hash, self.code_version = con, config_hash, code_version
        self.out_dir, self.n_workers = out_dir, n_workers
        self.t_start = time.time()
        self.done_at_start = self.counts()["done"]  # carried-over done count from a previous, resumed run
        self.window = collections.deque(maxlen=50)   # recent completion timestamps
        self.durations = collections.defaultdict(lambda: collections.deque(maxlen=30))  # (method,grid)->durations
        self.last_checkpoint = 0.0
        self.last_print = 0.0

    def counts(self):
        rows = self.con.execute("SELECT status, COUNT(*) FROM jobs WHERE config_hash=? AND code_version=? "
                                 "GROUP BY status", (self.config_hash, self.code_version)).fetchall()
        c = {"pending": 0, "running": 0, "done": 0, "failed": 0}
        c.update(dict(rows))
        return c

    def record(self, method, grid, duration):
        self.window.append(time.time())
        self.durations[(method, grid)].append(duration)

    def eta(self, pending_by_mg):
        if len(self.window) < 5:
            return None, "CALIBRATING"
        recent_rate = (len(self.window) - 1) / max(self.window[-1] - self.window[0], 1e-6)  # jobs/s, all workers
        # per-(method,grid) weighted remaining work, divided across n_workers running in parallel
        work_s = 0.0
        overall_avg = np.mean([d for dq in self.durations.values() for d in dq]) if self.durations else None
        for (mid, g), n in pending_by_mg.items():
            dq = self.durations.get((mid, g))
            avg = float(np.mean(dq)) if dq else (overall_avg or 1.0)
            work_s += n * avg
        eta_s = work_s / max(self.n_workers, 1)
        # sanity cross-check against the plain observed throughput; report the more conservative one
        if recent_rate > 0:
            eta_s_throughput = sum(pending_by_mg.values()) / recent_rate
            eta_s = max(eta_s, eta_s_throughput)
        return eta_s, f"{recent_rate*60:.1f} jobs/min"

    def maybe_checkpoint(self, force=False):
        now = time.time()
        if not force and now - self.last_checkpoint < 60:
            return
        self.last_checkpoint = now
        c = self.counts()
        pending_by_mg = dict(collections.Counter(
            (m, g) for m, g in self.con.execute(
                "SELECT method, grid FROM jobs WHERE config_hash=? AND code_version=? AND status IN ('pending','running')",
                (self.config_hash, self.code_version))))
        eta_s, rate_label = self.eta(pending_by_mg)
        elapsed = now - self.t_start
        total = sum(c.values())
        avg_speed = (c["done"] - self.done_at_start) / elapsed * 60 if elapsed > 1.0 else 0.0
        finish_dt = (datetime.now(KST) + timedelta(seconds=eta_s)) if eta_s is not None else None
        info = {
            "generated_at": datetime.now(KST).isoformat(),
            "total": total, "done": c["done"], "failed": c["failed"],
            "pending": c["pending"], "running": c["running"],
            "progress_pct": round(100 * c["done"] / total, 2) if total else 0.0,
            "elapsed_s": round(elapsed, 1),
            "recent_speed": rate_label,
            "avg_speed_jobs_per_min": round(avg_speed, 2),
            "eta_s": round(eta_s, 1) if eta_s is not None else None,
            "eta_label": "CALIBRATING" if eta_s is None else f"{eta_s/60:.1f} min",
            "expected_finish_kst": finish_dt.strftime("%Y-%m-%d %H:%M KST") if finish_dt else None,
            "eta_is_estimate_not_guaranteed": True,
            "config_hash": self.config_hash, "code_version": self.code_version,
        }
        tmp = self.out_dir / "progress.json.tmp"
        tmp.write_text(json.dumps(info, indent=2))
        tmp.replace(self.out_dir / "progress.json")
        set_meta(self.con, **{k_: v for k_, v in info.items()})
        return info

    def print_banner(self, stage_label):
        info = self.maybe_checkpoint(force=True)
        print("=" * 50)
        print("EX04 FULL REVALIDATION -- JETSON AGX ORIN")
        print("=" * 50)
        print(f"Stage           : {stage_label}")
        print(f"Total Jobs      : {info['total']}")
        print(f"Completed       : {info['done']}")
        print(f"Remaining       : {info['pending'] + info['running']}")
        print(f"Failed          : {info['failed']}")
        print(f"Progress        : {info['progress_pct']:.2f}%")
        print()
        print(f"Elapsed         : {timedelta(seconds=int(info['elapsed_s']))}")
        print(f"Recent Speed    : {info['recent_speed']}")
        print(f"Average Speed   : {info['avg_speed_jobs_per_min']:.1f} jobs/min")
        print()
        print(f"ETA             : {info['eta_label']} (estimate, not guaranteed)")
        print(f"Expected Finish : {info['expected_finish_kst']}")
        print()
        print(f"Last Checkpoint : {datetime.now(KST).strftime('%Y-%m-%d %H:%M KST')}")
        print("Resume Enabled  : YES")
        print("=" * 50, flush=True)


# -------------------------------------------------------------------------- main run loop
_STOP = False


def _handle_signal(signum, frame):
    global _STOP
    _STOP = True
    print(f"\n[run_ckpt] received signal {signum}: finishing in-flight jobs, no new files will be dispatched, "
          f"then exiting cleanly. Already-completed jobs are safe in SQLite regardless.", flush=True)


def write_csv_snapshot(con, config_hash, code_version, out_dir):
    rows = con.execute("SELECT result_json FROM jobs WHERE status='done' AND config_hash=? AND code_version=? "
                        "AND result_json IS NOT NULL", (config_hash, code_version)).fetchall()
    if not rows:
        return
    records = [json.loads(r[0]) for r in rows]
    records.sort(key=lambda r: (r["group"], r["file"], r["method"], r["grid"]))
    tmp = out_dir / "per_traj.csv.tmp"
    with open(tmp, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(records[0]))
        w.writeheader()
        w.writerows(records)
    tmp.replace(out_dir / "per_traj.csv")


def run_plan(cohort_dir, out_dir, stages, psi_map="mirror", qd_max=np.pi, hold_s=0.5, n_workers=8,
             max_attempts=3, stage_label=None, fast_e1=False):
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "logs").mkdir(exist_ok=True)
    manifest, files, jobs = build_plan(cohort_dir, stages)
    code_version = git_sha()
    config_hash = config_hash_of(manifest.get("cohort_origin_sha256", manifest.get("git_sha", "na")),
                                  psi_map, qd_max, hold_s, fast_e1=fast_e1)
    con = connect(out_dir / "checkpoints" / "run_state.sqlite")
    sync_plan(con, files, jobs, config_hash, code_version)
    set_meta(con, current_stage=stage_label or "+".join(stages), config_hash=config_hash,
             code_version=code_version, started_at=datetime.now(KST).isoformat())

    prog = Progress(con, config_hash, code_version, out_dir, n_workers)
    prog.print_banner(stage_label or "+".join(stages))

    manager = mp.Manager()
    queue = manager.Queue()
    pending_rows = con.execute(
        "SELECT file, scenario, method, grid FROM jobs WHERE config_hash=? AND code_version=? "
        "AND status IN ('pending','running') AND attempts < ?",
        (config_hash, code_version, max_attempts)).fetchall()
    by_file = collections.defaultdict(list)
    scen_of = {}
    for f, scen, mid, grid in pending_rows:
        g = next(gg for mm, gg in jobs if mm == mid and grid_key(mm, gg) == grid)
        by_file[f].append((mid, g))
        scen_of[f] = scen
    tasks = [(f, scen_of[f], ms, config_hash, code_version) for f, ms in by_file.items()]
    print(f"[run_ckpt] {len(tasks)} files with pending work, {len(pending_rows)} jobs total", flush=True)

    n_failed_perm = 0
    with mp.Pool(processes=n_workers, initializer=_winit, initargs=(queue, psi_map, qd_max, hold_s, fast_e1)) as pool:
        async_res = pool.map_async(_worker_file, tasks, chunksize=1)
        while True:
            if _STOP:
                break
            if async_res.ready() and queue.empty():
                break
            try:
                kind, key, payload, dt = queue.get(timeout=1.0)
            except Exception:
                prog.maybe_checkpoint()
                continue
            file_path, method, grid, chash, cver = key
            now = time.time()
            if kind == "running":
                con.execute("UPDATE jobs SET status='running', started_at=?, attempts=attempts+1 "
                            "WHERE file=? AND method=? AND grid=? AND config_hash=? AND code_version=?",
                            (now, file_path, method, grid, chash, cver))
            elif kind == "done":
                con.execute("UPDATE jobs SET status='done', finished_at=?, result_json=?, error=NULL "
                            "WHERE file=? AND method=? AND grid=? AND config_hash=? AND code_version=?",
                            (now, json.dumps(payload), file_path, method, grid, chash, cver))
                con.execute("INSERT INTO completions VALUES (?,?,?,?,?)",
                            (now, method, grid, payload["group"], dt))
                prog.record(method, grid, dt)
            elif kind == "failed":
                row = con.execute("SELECT attempts FROM jobs WHERE file=? AND method=? AND grid=? "
                                   "AND config_hash=? AND code_version=?",
                                   (file_path, method, grid, chash, cver)).fetchone()
                attempts = row[0] if row else max_attempts
                status = "failed" if attempts >= max_attempts else "pending"
                con.execute("UPDATE jobs SET status=?, finished_at=?, error=? "
                            "WHERE file=? AND method=? AND grid=? AND config_hash=? AND code_version=?",
                            (status, now, payload, file_path, method, grid, chash, cver))
                if status == "failed":
                    n_failed_perm += 1
                    print(f"[run_ckpt] PERMANENT FAILURE after {attempts} attempts: "
                          f"{Path(file_path).name}/{method}/{grid}: {payload}", flush=True)
            info = prog.maybe_checkpoint()
            if info and time.time() - prog.last_print > 20:
                prog.last_print = time.time()
                print(f"[{info['generated_at']}] {info['done']}/{info['total']} done, "
                      f"{info['failed']} failed, {info['progress_pct']:.1f}%, "
                      f"ETA {info['eta_label']} ({info['expected_finish_kst']})", flush=True)
                write_csv_snapshot(con, config_hash, code_version, out_dir)
        if _STOP:
            # Best-effort: flush whatever results are already sitting in the queue (jobs that
            # finished computing but weren't drained yet) before terminating in-flight workers.
            # Anything still in a worker's _run_one at this instant is lost and will simply be
            # re-run on the next invocation -- it was never marked 'done' in SQLite.
            while True:
                try:
                    kind, key, payload, dt = queue.get_nowait()
                except Exception:
                    break
                file_path, method, grid, chash, cver = key
                now = time.time()
                if kind == "done":
                    con.execute("UPDATE jobs SET status='done', finished_at=?, result_json=?, error=NULL "
                                "WHERE file=? AND method=? AND grid=? AND config_hash=? AND code_version=?",
                                (now, json.dumps(payload), file_path, method, grid, chash, cver))
                    con.execute("INSERT INTO completions VALUES (?,?,?,?,?)",
                                (now, method, grid, payload["group"], dt))
            try:
                pool.terminate()
                for _ in range(100):  # bounded wait (~10s); workers now die immediately on SIGTERM
                    if not any(p.is_alive() for p in pool._pool):  # (default disposition, see _winit)
                        break
                    time.sleep(0.1)
                else:
                    for p in pool._pool:  # last resort: a worker ignored even default SIGTERM somehow
                        if p.is_alive():
                            os.kill(p.pid, signal.SIGKILL)
                pool.join()
            except Exception as e:  # noqa: BLE001 -- shutdown must not crash after a signal; all
                print(f"[run_ckpt] pool shutdown warning (ignored, results already committed): {e}",
                      flush=True)  # completed work is already durably committed to SQLite regardless
        else:
            pool.close()
            pool.join()

    write_csv_snapshot(con, config_hash, code_version, out_dir)
    prog.print_banner(stage_label or "+".join(stages))
    if n_failed_perm:
        print(f"[run_ckpt] {n_failed_perm} jobs permanently failed (see failures.csv)", flush=True)
    failed_rows = con.execute("SELECT file,scenario,method,grid,attempts,error FROM jobs WHERE status='failed' "
                              "AND config_hash=? AND code_version=?", (config_hash, code_version)).fetchall()
    if failed_rows:
        with open(out_dir / "failures.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["file", "scenario", "method", "grid", "attempts", "error"])
            w.writerows(failed_rows)
    con.close()
    return 0 if not _STOP else 130


def cmd_status(out_dir):
    db = out_dir / "checkpoints" / "run_state.sqlite"
    if not db.is_file():
        print(f"NOT STARTED: no checkpoint DB at {db}")
        return 1
    con = connect(db)
    meta = get_meta(con)
    chash, cver = meta.get("config_hash"), meta.get("code_version")
    # Live counts straight from the jobs table -- run_meta's progress/ETA fields are a periodic
    # (<=60s) snapshot written by the running process, fine for the smoothed speed/ETA numbers but
    # stale for "how many are done right now", which this command should answer exactly.
    live = dict(con.execute("SELECT status, COUNT(*) FROM jobs WHERE config_hash=? AND code_version=? "
                             "GROUP BY status", (chash, cver)).fetchall()) if chash else {}
    total = sum(live.values())
    done = live.get("done", 0)
    print(f"Stage           : {meta.get('current_stage', '?')}")
    print(f"Config hash     : {chash}  code_version: {cver}")
    print(f"Total / Done    : {total or meta.get('total', '?')} / {done}")
    print(f"Pending/Running : {live.get('pending', 0)} / {live.get('running', 0)}")
    print(f"Failed          : {live.get('failed', 0)}")
    print(f"Progress        : {100*done/total:.2f}%" if total else "Progress        : ?")
    print(f"Recent Speed    : {meta.get('recent_speed', '?')}  (as of last snapshot below)")
    print(f"ETA             : {meta.get('eta_label', '?')} -> {meta.get('expected_finish_kst', '?')}")
    print(f"Last snapshot   : {meta.get('generated_at', '?')} (speed/ETA refresh at most once/60s; "
          f"counts above are live)")
    print(f"Resume Enabled  : YES (default behaviour of this script)")
    con.close()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("plan", nargs="?", choices=list(STAGE_PLANS) + [None], default=None)
    ap.add_argument("--cohort", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--psi-map", default="mirror")
    ap.add_argument("--qd-max", type=float, default=np.pi)
    ap.add_argument("--hold-s", type=float, default=0.5)
    ap.add_argument("--resume", action="store_true", help="no-op: resuming is always the default")
    ap.add_argument("--fast-e1", action="store_true", help="certified psi-pruned E1; use a separate output dir")
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--max-attempts", type=int, default=3)
    a = ap.parse_args()

    if a.status:
        return cmd_status(a.out)
    if not a.plan or not a.cohort:
        ap.error("PLAN and --cohort are required unless --status is given")
    if a.plan in ("fast15", "fast20") and not a.fast_e1:
        ap.error("fast15/fast20 require --fast-e1")
    stages = [a.plan]
    return run_plan(a.cohort, a.out, stages, a.psi_map, a.qd_max, a.hold_s, a.workers,
                     a.max_attempts, stage_label=a.plan, fast_e1=a.fast_e1)


if __name__ == "__main__":
    raise SystemExit(main())
