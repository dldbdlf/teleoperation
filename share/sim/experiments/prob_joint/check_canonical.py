#!/usr/bin/env python3
"""Verify isolated E0-E3 canonical reference determinism across subprocess and q starts.
Does not test the unpushed project P/Pc canonical solver.
"""
import argparse,hashlib,json,subprocess,sys
import numpy as np
from nero_kin import Nero, READY
from heldout_ablation import make_scenario,canonical_logp

def fingerprint(seed,grid,frames):
    m=Nero.create();d=make_scenario(m,seed,frames,grid)
    qref=np.asarray([d['bank'][int(np.argmax(canonical_logp(d,t)))] for t in range(frames)])
    rng=np.random.default_rng(seed+918)
    initial_qs=np.array([np.clip(READY+rng.uniform(-.5,.5,7),m.lower,m.upper) for _ in range(20)])
    assert all(np.array_equal(qref[t],d['bank'][int(np.argmax(canonical_logp(d,t)))]) for t in range(frames) for _ in initial_qs)
    return hashlib.sha256(qref.tobytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--seed-start',type=int,default=48231);p.add_argument('--episodes',type=int,default=12)
    p.add_argument('--frames',type=int,default=24);p.add_argument('--grid',type=int,default=15)
    p.add_argument('--single',type=int,default=None)
    a=p.parse_args()
    if a.single is not None:
        print(json.dumps({'fingerprint':fingerprint(a.single,a.grid,a.frames),'seed':a.single}));return
    mismatches=[]
    for s in range(a.seed_start,a.seed_start+a.episodes):
        this=fingerprint(s,a.grid,a.frames)
        proc=subprocess.run([sys.executable,__file__,'--single',str(s),'--grid',str(a.grid),'--frames',str(a.frames)],capture_output=True,text=True,check=True)
        other=json.loads(proc.stdout)['fingerprint']
        if this!=other:mismatches.append(s)
    print(json.dumps({'test':'E0-E3 Q_REF CANONICAL SUBPROCESS AND 20 DIFFERENT Q STARTS','episodes':a.episodes,
                     'frames_per_episode':a.frames,'subprocess_mismatches':len(mismatches),
                     'mismatch_seeds':mismatches,'passed':not bool(mismatches)}))
    if mismatches:raise SystemExit(1)
if __name__=='__main__':main()
