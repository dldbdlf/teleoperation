#!/usr/bin/env python3
"""Holdout, fully executed synthetic IK branch ablation with NERO 7DOF geometry.

E0: fixed-posterior canonical MAP reference (no tracker branch selection)
E1: canonical reference + fixed branch-aware continuity cost
E2: canonical reference + entropy adaptive categorical branch posterior decision
E3: canonical reference + temporal Markov posterior branch selector

Fixed wrist per episode, varying elbow psi; explicit simulator speed cap. Results
cannot be generalized to real moving-wrist or real D415 datasets. Jetson latency
is NOT measured here. Output deterministic seeds and run settings.
"""
from __future__ import annotations
import argparse,csv,json,time,hashlib,platform
from pathlib import Path
import numpy as np
from scipy.special import logsumexp
from scipy.spatial.transform import Rotation
from nero_kin import Nero, READY, wrap

METHODS=('E0','E1','E2','E3')

def make_bank(m,T,step):
    result=[]
    for psi in np.deg2rad(np.arange(-180,180,step)):
        for sol_i,q in enumerate(m.feasible(T,float(psi))):
            if np.all(np.isfinite(q)) and np.linalg.norm(m.fk(q)-T)<1e-4:
                result.append((q,float(psi),sol_i))
    if not result:
        return np.zeros((0,7)),np.zeros(0),np.zeros(0,dtype=int)
    return np.array([x[0] for x in result]),np.array([x[1] for x in result]),np.array([x[2] for x in result],int)

def make_scenario(m,seed,n,grid,occlusion_fraction=.25):
    rng=np.random.default_rng(seed)
    for _ in range(60):
        q=np.clip(READY+rng.normal(0,[.2,.1,.2,.12,.12,.06,.15]),m.lower+.08,m.upper-.08)
        T=m.fk(q)
        bank,psis,branches=make_bank(m,T,grid)
        if len(bank)>=10 and np.isfinite(m.psi(q)):
            break
    else: raise RuntimeError(f'no valid IK bank, seed={seed}')
    truth=wrap(m.psi(q)+.25*np.sin(np.linspace(0,2.3*np.pi,n)))
    mu=wrap(truth+rng.normal(0,np.deg2rad(11),n))
    sigma=np.full(n,np.deg2rad(11.))
    t0=n//3;t1=min(n,t0+round(n*occlusion_fraction))
    sigma[t0:t1]=np.deg2rad(65)
    if t1>t0:
        mu[t0:t1]=wrap(mu[t0-1]+np.cumsum(rng.normal(0,np.deg2rad(26),t1-t0)))
    valid=np.array([np.any(np.abs(wrap(psis-p))<np.deg2rad(25)) for p in truth])
    if not np.any(valid):raise RuntimeError('No feasible frames for elbow error')
    return {'T':T,'bank':bank,'psis':psis,'branches':branches,
            'truth':truth,'mu':mu,'sigma':sigma,'valid':valid,
            't0':t0,'t1':t1, 'seed':seed}

def canonical_logp(e,t):
    psi=e['psis']; bank=e['bank'];sig=e['sigma'][t]
    dpsi=wrap(psi-e['mu'][t]);dh=wrap(bank-READY)
    z=-(.5*(dpsi/sig)**2)-.055*np.sum(dh*dh,axis=1)
    return z-logsumexp(z)

def evaluate(m,e,method,rate=100,cam=15):
    bank=e['bank'];n=len(e['mu']);q=READY.copy();Q=[];refs=[];targets=[]
    selected=[]; lats=[]; oldp=None
    ticks=np.rint(np.arange(n+1)*rate/cam).astype(int)
    for i in range(n):
        stamp=time.perf_counter()
        logp=canonical_logp(e,i)
        ref_i=int(np.argmax(logp))
        refs.append(ref_i)
        dist2=np.sum(wrap(bank-q)**2,axis=1)
        if method=='E0': j=ref_i
        elif method=='E1': j=int(np.argmin(-logp+.85*dist2))
        elif method=='E2':
            p=np.exp(logp); ent=-(p*logp).sum()
            j=int(np.argmin(-logp+(.45+.23*ent)*dist2))
        elif method=='E3':
            if oldp is None: p=np.exp(logp)
            else:
                d2=np.sum(wrap(bank[:,None,:]-bank[None,:,:])**2,axis=-1)
                transition=np.exp(-.5*d2/(.60**2))+.0005
                transition/=transition.sum(axis=0,keepdims=True)
                predicted=transition@oldp
                lp=logp+np.log(np.maximum(predicted,1e-300))
                p=np.exp(lp-logsumexp(lp))
            oldp=p
            j=int(np.argmin(-np.log(np.maximum(p,1e-300))+.22*dist2))
        else:raise ValueError(method)
        lats.append(1000*(time.perf_counter()-stamp))
        target=bank[j].copy(); selected.append(j);targets.append(target)
        for _ in range(ticks[i+1]-ticks[i]):
            dq=wrap(target-q)
            q=np.clip(q+np.clip(dq,-np.pi/rate,np.pi/rate),m.lower,m.upper)
            Q.append(q.copy())
    Q=np.array(Q); targets=np.array(targets);refs=np.array(refs);selected=np.array(selected)
    assert np.all(Q>=m.lower-1e-8) and np.all(Q<=m.upper+1e-8)
    assert np.max(np.abs(np.diff(Q,axis=0)))<=np.pi/rate+1e-9
    pos=[];psi=[]
    for i in range(n):
        actual=Q[ticks[i+1]-1]
        pos.append(1000*np.linalg.norm(m.fk(actual)[:3,3]-e['T'][:3,3]))
        psi.append(np.rad2deg(abs(float(wrap(m.psi(actual)-e['truth'][i])))))
    jerk=np.diff(Q,n=3,axis=0)*(rate**3)
    req=np.rad2deg(np.max(np.abs(wrap(np.diff(targets,axis=0))),axis=1))
    recovery=req[max(0,e['t1']-1):min(n-1,e['t1']+cam-1)]
    return dict(seed=e['seed'],method=method,bank_count=len(bank),
        wrist_p95_mm=float(np.percentile(pos,95)),
        wrist_p50_mm=float(np.median(pos)),
        psi_p90_deg=float(np.percentile(np.array(psi)[e['valid']],90)),
        jerk_rms=float(np.sqrt(np.mean(jerk**2))),
        recovery_req_jump_deg=float(recovery.max()) if len(recovery) else None,
        jump_over60_count=int(np.sum(req>60)),
        target_mode_switches=int(np.sum(selected[1:]!=selected[:-1])),
        canonical_ref_switches=int(np.sum(refs[1:]!=refs[:-1])),
        actual_limit_min_margin_deg=float(np.rad2deg(np.min(np.minimum(Q-m.lower,m.upper-Q)))),
        selection_ms_p95=float(np.percentile(lats,95)))

def paired_ci(rows,methods,metric,bootstrap,seed=72):
    # stable paired frame-block resampling across episodes, not frames
    allpairs={r['seed']:r for r in rows}
    from collections import defaultdict
    by=defaultdict(dict)
    for r in rows:by[r['seed']][r['method']]=r
    rng=np.random.default_rng(seed)
    results={}
    for a,b in [('E1','E0'),('E2','E1'),('E3','E1')]:
        values=np.array([d[a][metric]-d[b][metric] for _,d in sorted(by.items()) if a in d and b in d])
        samples=rng.choice(values,(bootstrap,len(values)),replace=True).mean(axis=1)
        results[f'{a}-{b}']=dict(mean=float(values.mean()),CI95=[float(x) for x in np.percentile(samples,[2.5,97.5])])
    return results

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--episodes',type=int,default=30)
    parser.add_argument('--frames',type=int,default=72)
    parser.add_argument('--seed-start',type=int,default=19231)
    parser.add_argument('--grids',type=int,nargs='+',default=[10,15,20])
    parser.add_argument('--bootstrap',type=int,default=2000)
    parser.add_argument('--out',type=Path,default=Path('heldout_results'))
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    m=Nero.create();t0=time.perf_counter();out={}
    fields=['wrist_p95_mm','wrist_p50_mm','psi_p90_deg','jerk_rms','recovery_req_jump_deg','jump_over60_count','target_mode_switches','canonical_ref_switches','actual_limit_min_margin_deg','selection_ms_p95']
    for grid in args.grids:
        rows=[];equiv_err=0.
        for i in range(args.episodes):
            e=make_scenario(m,args.seed_start+i,args.frames,grid)
            canonical0=[np.argmax(canonical_logp(e,t)) for t in range(args.frames)]
            for _ in range(100):
                equiv_err=max(equiv_err,float(np.max(np.abs(np.array(canonical0)-np.array([np.argmax(canonical_logp(e,t)) for t in range(args.frames)])))))
            for method in METHODS:rows.append(evaluate(m,e,method))
            if (i+1)%10==0:print(f'{grid}°: {i+1}/{args.episodes}, elapsed {time.perf_counter()-t0:.1f} s',flush=True)
        with open(args.out/f'episodes_{grid}.csv','w',newline='') as fh:
            w=csv.DictWriter(fh,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        summary={}
        for method in METHODS:
            subset=[r for r in rows if r['method']==method]
            summary[method]={f:float(np.mean([r[f] for r in subset])) for f in fields}
        pairs={met:paired_ci(rows,METHODS,met,args.bootstrap) for met in fields if met != 'selection_ms_p95'}
        out[str(grid)]={'n_episodes':args.episodes,'canonical_repeat100x_max_idx_diff':equiv_err,
                        'summary':summary,'paired_ci':pairs}
        print(f'grid {grid}° summary E0/E1/E2/E3 wrist={[(x,round(summary[x]["wrist_p95_mm"],2)) for x in METHODS]}',flush=True)
    result=dict(metadata={'date':'2026-11-15','experiment':'NERO fixed-wrist held-out branch ablation','seed_start':args.seed_start,'episodes_per_grid':args.episodes,'frames_per_episode':args.frames,'grids':args.grids,'bootstrap':args.bootstrap,'python':platform.python_version(),'numpy':np.__version__, 'environment':platform.platform(),'jetson':False,'limitation':'fixed wrist, no real RGBD, heuristic belief, no real 894-run, latency excludes IK bank enumeration'},grids=out,wall_s=time.perf_counter()-t0)
    (args.out/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print('DONE',args.out,'wall',result['wall_s'])
if __name__=='__main__':main()
