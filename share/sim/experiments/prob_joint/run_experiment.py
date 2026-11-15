"""Exploratory, controlled NERO 7-DoF probabilistic-IK-branch experiment.

Not an EX04 894-trajectory replay. Not the unreleased Pc/FeasibleMAP implementation.
Compares a deterministic branch MAP, branch-aware tracking, a joint-space probability
one-frame confidence policy, and temporal joint-space posterior tracking on IDENTICAL
synthetic noisy elbow-belief observations and exact URDF-derived IK candidate banks.

Usage: OPENBLAS_NUM_THREADS=1 python run_experiment.py --output results.json
"""
from __future__ import annotations
import argparse
import json
import time
from pathlib import Path
import numpy as np
from scipy.special import logsumexp
from scipy.spatial.transform import Rotation
from nero_kin import Nero, READY, wrap

PI=np.pi

def adiff(a,b): return np.abs(wrap(a-b))
def jdist(a,b): return float(np.linalg.norm(wrap(a-b)))
def make_bank(m,T,step=15):
    bank=[]
    for psi in np.deg2rad(np.arange(-180,180,step)):
        for branch_index,q in enumerate(m.feasible(T,psi)):
            # Only geometrically-valid finite candidates are admitted.
            if np.all(np.isfinite(q)) and np.linalg.norm(m.fk(q)-T)<1.e-4:
                bank.append((q,psi,branch_index))
    if not bank: return np.empty((0,7)),np.array([]),np.array([])
    return np.array([a[0] for a in bank]),np.array([a[1] for a in bank]),np.array([a[2] for a in bank])

def pose_error(m,q,T):
    A=m.fk(q); R=Rotation.from_matrix(A[:3,:3]@T[:3,:3].T)
    return 1000*float(np.linalg.norm(A[:3,3]-T[:3,3])), float(np.rad2deg(R.magnitude()))

def generate(m,seed,frames,occl_start,occl_len,bank_step):
    rng=np.random.default_rng(seed)
    for attempt in range(60):
        q=READY+rng.normal(0,[.20,.10,.20,.12,.12,.06,.15])
        q=np.clip(q,m.lower+.08,m.upper-.08)
        T=m.fk(q)
        bank,psi,branch=make_bank(m,T,bank_step)
        if len(bank)>=10 and np.isfinite(m.psi(q)):break
    else:raise RuntimeError(f'seed {seed}: insufficient feasible IK candidates')
    true_psi=wrap(m.psi(q)+.25*np.sin(np.linspace(0,2.3*np.pi,frames)))
    valid=np.array([any(abs(float(wrap(p-s)))<np.deg2rad(25) for p in psi) for s in true_psi])
    # human posterior is observational, never conditioned on robot feasibility
    noise=rng.normal(0,np.deg2rad(11),frames)
    mu=wrap(true_psi+noise)
    sig=np.full(frames,np.deg2rad(11.0))
    # occlusion: measured elbow unobserved, belief broadens and becomes unreliable
    end=min(frames,occl_start+occl_len)
    if end>occl_start:
        sig[occl_start:end]=np.deg2rad(65)
        last=mu[occl_start-1]
        mu[occl_start:end]=wrap(last + np.cumsum(rng.normal(0,np.deg2rad(26),end-occl_start)))
    return dict(seed=seed,bank=bank,psi=psi,branch=branch,T=T,mu=mu,sigma=sig,
                true_psi=true_psi,occl_start=occl_start,occl_end=end,
                valid=valid,source_q=q)

def deterministic_posterior(data, t, home_weight=.055):
    # Fixed home prior; absolutely no q_prev, tracking state, or warm-start.
    bank=data['bank']; psi=data['psi']; sig=data['sigma'][t]
    dpsi=wrap(psi-data['mu'][t]);dh=wrap(bank-READY)
    logw= -.5*(dpsi/sig)**2 - home_weight*np.sum(dh*dh,axis=1)
    # stable deterministic ordering: bank enumeration fixed by psi grid and IK order
    logw-=logsumexp(logw)
    return logw

def run_one(m,data,method,rate=100,obs_hz=15,qd_max=np.pi):
    bank=data['bank']; n=len(data['mu']);q=READY.copy();qref_seq=[];tgt_seq=[];Q=[];ticks_target=[]
    # Candidate-mode tracking uses NO probabilistic time filter; only switching cost.
    # Probability methods maintain a discrete joint-space posterior over feasible IK solutions.
    prev_p=None;old_idx=None; latency=[];chosen=[];raw_modes=[]
    # Timing sample per frame; machine dependent; not a Jetson benchmark.
    tick_boundaries=np.rint(np.arange(n+1)*rate/obs_hz).astype(int); dt=1/rate
    for t in range(n):
        t0=time.perf_counter()
        logw=deterministic_posterior(data,t)
        i_ref=int(np.argmax(logw))
        q_ref=bank[i_ref]
        # E0: canonical only / no branch hysteresis
        if method=='E0_canonical':target_i=i_ref
        elif method=='E1_branch_tracker':
            jcost=-logw+0.85*np.sum(wrap(bank-q)**2,axis=1)
            # stable tie preference by candidate list order
            target_i=int(np.argmin(jcost))
        elif method=='E2_joint_prob_instant':
            # Single-frame categorical p(q,b|belief), entropy-based conservative actuation.
            # Candidate probability can be multi-modal; do not average joint configurations.
            p=np.exp(logw); entropy=float(-(p*logw).sum())
            # uncertainty gates requested transitions toward distant modes
            jcost=-logw+(0.45+0.23*entropy)*np.sum(wrap(bank-q)**2,axis=1)
            target_i=int(np.argmin(jcost))
        elif method=='E3_joint_prob_temporal':
            # Hidden-Markov inference on joint-space candidate support.
            # transition kernel favors smaller 7D joint distance, with sparse outlier floor.
            # q-ref stays the SAME deterministic function of current belief as E0.
            if prev_p is None: prev_p=np.exp(logw)
            else:
                d2=np.sum(wrap(bank[:,None,:]-bank[None,:,:])**2,axis=-1)
                transit=np.exp(-.5*d2/(.60**2))+0.0005
                transit/=transit.sum(axis=0,keepdims=True)
                pred=transit@prev_p
                lp=logw+np.log(np.maximum(pred,1e-300))
                prev_p=np.exp(lp-logsumexp(lp))
            p=np.maximum(prev_p,1.e-300)
            score= -np.log(p)+0.22*np.sum(wrap(bank-q)**2,axis=1)
            target_i=int(np.argmin(score))
        else:raise ValueError(method)
        latency.append(1000*(time.perf_counter()-t0))
        qref_seq.append(q_ref.copy());target=bank[target_i].copy();tgt_seq.append(target);chosen.append(target_i);raw_modes.append(i_ref)
        # Same physical command speed cap on every method at 100 Hz.
        for _ in range(tick_boundaries[t+1]-tick_boundaries[t]):
            dq=wrap(target-q);q=np.clip(q+np.clip(dq,-qd_max*dt,qd_max*dt),m.lower,m.upper)
            Q.append(q.copy());ticks_target.append(target.copy())
    Q=np.array(Q);T=data['T'];true=data['true_psi']
    pos=[];ori=[];perpsi=[]
    for j in range(n):
        idx=tick_boundaries[j+1]-1;p,o=pose_error(m,Q[idx],T);pos.append(p);ori.append(o)
        perpsi.append(float(np.rad2deg(adiff(m.psi(Q[idx]),true[j]))))
    Qref=np.array(qref_seq);targets=np.array(tgt_seq)
    qdot=np.diff(Q,axis=0)/dt;jerk=np.diff(Q,n=3,axis=0)/dt**3
    # Geodesic maximum-joint target jump in degrees at observation updates
    requested=np.rad2deg(np.max(adiff(targets[1:],targets[:-1]),axis=1))
    refjump=np.rad2deg(np.max(adiff(Qref[1:],Qref[:-1]),axis=1))
    a=data['occl_end'];b=min(n,a+obs_hz)
    recjump=requested[a-1:b-1] if a<n else np.array([])
    return {'method':method,'seed':data['seed'],'n_frames':n,'bank_count':len(bank),
      'wrist_mm_p50':float(np.median(pos)),'wrist_mm_p95':float(np.percentile(pos,95)),
      'wrist_deg_p50':float(np.median(ori)),
      'psi_deg_p90':float(np.percentile(np.asarray(perpsi)[data['valid']],90)),
      'jerk_rms':float(np.sqrt(np.mean(jerk**2))),
      'joint_vel_deg_s_p95':float(np.percentile(np.rad2deg(np.abs(qdot)),95)),
      'recovery_reqjump_deg':float(np.max(recjump)) if len(recjump) else None,
      'target_jump_over60_count':int(np.sum(requested>60)),
      'qref_jump_over60_count':int(np.sum(refjump>60)),
      'calc_ms_p95':float(np.percentile(latency,95)),
      'deadline15hz_miss_pct':float(100*np.mean(np.array(latency)>1000/obs_hz)),
      'min_limit_margin_deg':float(np.rad2deg(np.min(np.minimum(Q-m.lower,m.upper-Q)))),
      'qref':Qref,'Q':Q,'chosen':chosen}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--episodes',type=int,default=10);ap.add_argument('--frames',type=int,default=48)
    ap.add_argument('--bank-step',type=int,default=20);ap.add_argument('--output',default='results.json')
    args=ap.parse_args();m=Nero.create()
    # no mutation of shared observations; paired methods on identical inputs
    M=['E0_canonical','E1_branch_tracker','E2_joint_prob_instant','E3_joint_prob_temporal']
    rows=[];repeat=[];tstart=time.perf_counter()
    for i in range(args.episodes):
        data=generate(m,i+9131,args.frames,args.frames//3,args.frames//4,args.bank_step)
        qref=np.asarray([data['bank'][int(np.argmax(deterministic_posterior(data,t)))] for t in range(args.frames)])
        # Explicit path/call-order independence test on 100 equal input calls
        t=args.frames//2
        vectors=[data['bank'][int(np.argmax(deterministic_posterior(data,t)))] for _ in range(100)]
        mdiff=max(np.max(np.abs(x-vectors[0])) for x in vectors)
        repeat.append(mdiff)
        for method in M:
            r=run_one(m,data,method)
            if not np.array_equal(r.pop('qref'),qref):raise AssertionError('canonical changed across methods')
            r.pop('Q');r.pop('chosen')
            rows.append(r)
        print(f'completed {i+1}/{args.episodes} episodes; elapsed {time.perf_counter()-tstart:.1f}s',flush=True)
    summ={}
    keys=['wrist_mm_p50','wrist_mm_p95','psi_deg_p90','jerk_rms','recovery_reqjump_deg',
          'target_jump_over60_count','calc_ms_p95','deadline15hz_miss_pct']
    for method in M:
        sample=[r for r in rows if r['method']==method]
        summ[method]={k:round(float(np.mean([r[k] for r in sample if r[k] is not None])),4) for k in keys}
    result={'type':'EXPLORATORY_NERO_FIXED_WRIST_SYNTHETIC','not_reproduction_of_894':True,
            'notes':['Fixed tool pose per episode: tests branch handling, not full motion/vision','Analytic IK geometry from NERO URDF; finite psi grid specified via --bank-step','Probabilities are heuristic uncalibrated posterior over candidate IK states','Jetson latency cannot be inferred from this container CPU','E0-E3 are experimental controls, NOT the user local unreleased P/Pc versions'],
            'settings':{'episodes':args.episodes,'frames_per_episode':args.frames,'psi_bank_step_deg':args.bank_step,'camera_hz':15,'command_hz_nominal':100,'qdot_max_rad_s':float(PI),'noise_visible_psi_deg':11,'noise_occluded_psi_deg':65},
            'canonical_repeated_100x_max_abs_rad':float(max(repeat)),'summary':summ,'per_episode':rows,
            'wall_s':round(time.perf_counter()-tstart,2)}
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    Path(args.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'determinism_max_rad':result['canonical_repeated_100x_max_abs_rad'],'summary':summ,'wall_s':result['wall_s']},indent=2))
if __name__=='__main__':main()
