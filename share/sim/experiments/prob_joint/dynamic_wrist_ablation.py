#!/usr/bin/env python3
"""Moving-wrist holdout: changing target SE(3), elbow uncertainty and wrist dropout.
Actual q constrained to pi rad/s; solver candidate IK banks recomputed every 15Hz
frame. This is a NEW small synthetic stress test, not EX04/Jetson performance.
"""
from __future__ import annotations
import argparse,csv,json,time,platform
from pathlib import Path
import numpy as np
from scipy.special import logsumexp
from scipy.spatial.transform import Rotation
from nero_kin import Nero,READY,wrap
from heldout_ablation import make_bank
METHODS=['E0','E1','E2','E3']

def scenario(m,seed,n,grid):
    rng=np.random.default_rng(seed)
    q0=np.clip(READY+rng.normal(0,.13,7),m.lower+.14,m.upper-.14)
    amp=rng.uniform(.04,.11,7)*rng.choice([-1,1],7)
    phase=rng.uniform(-1,1,7)
    ts=np.arange(n)/15
    truth_q=np.array([np.clip(q0+amp*np.sin(2*np.pi*(.3+phase*.05)*t+phase),m.lower+.05,m.upper-.05) for t in ts])
    Tg=[m.fk(q) for q in truth_q];psi=np.array([m.psi(q) for q in truth_q]);mu=wrap(psi+rng.normal(0,np.deg2rad(10),n))
    drop0=n//3;drop1=min(n,drop0+min(10,n//3)); sig=np.full(n,np.deg2rad(10.))
    if drop1>drop0:
      mu[drop0:drop1]=wrap(mu[drop0-1]+np.cumsum(rng.normal(0,np.deg2rad(25),drop1-drop0)))
      sig[drop0:drop1]=np.deg2rad(60)
    entries=[]
    for t in range(n):
       observed=Tg[min(t,drop0-1) if drop0<=t<drop1 else t]
       bank,psis,bids=make_bank(m,observed,grid)
       entries.append({'T':observed,'bank':bank,'psis':psis,'bids':bids,
                       'mu':mu[t],'sigma':sig[t]})
    return {'entries':entries,'truthT':Tg,'truepsi':psi,'drop0':drop0,'drop1':drop1,'seed':seed}

def logp(e):
    qs,psis=e['bank'],e['psis']
    z=-.5*(wrap(psis-e['mu'])/e['sigma'])**2-.055*np.sum(wrap(qs-READY)**2,axis=1)
    return z-logsumexp(z)

def evaluate(m,sc,method,cam=15,rate=100):
    q=READY.copy();Qs=[];ref=[];selected_q=[]; selected_ix=[];priorbank=None;priorp=None;invalid=0;lats=[];numcands=[]
    n=len(sc['entries']);ticks=np.rint(np.arange(n+1)*rate/cam).astype(int)
    for t,e in enumerate(sc['entries']):
      start=time.perf_counter()
      bank=e['bank'];numcands.append(len(bank))
      if not len(bank):
        invalid+=1;target=q.copy();ref.append(None);selected_ix.append(-1)
        priorp=None;priorbank=None
      else:
        lp=logp(e); ci=int(np.argmax(lp));ref.append(bank[ci].copy())
        d2=np.sum(wrap(bank-q)**2,axis=1)
        if method=='E0':i=ci
        elif method=='E1':i=int(np.argmin(-lp+.85*d2))
        elif method=='E2':
          prob=np.exp(lp); entropy=-float(np.sum(prob*lp))
          i=int(np.argmin(-lp+(.45+.23*entropy)*d2))
        elif method=='E3':
          if priorp is None:prob=np.exp(lp)
          else:
            r2=np.sum(wrap(bank[:,None,:]-priorbank[None,:,:])**2,axis=-1)
            trans=np.exp(-.5*r2/.60**2)+.0005;trans/=trans.sum(axis=0,keepdims=True)
            lpnew=lp+np.log(np.maximum(trans@priorp,1e-300))
            prob=np.exp(lpnew-logsumexp(lpnew))
          i=int(np.argmin(-np.log(np.maximum(prob,1e-300))+.22*d2))
          priorbank=bank.copy();priorp=prob.copy()
        else: raise ValueError(method)
        selected_ix.append(i);target=bank[i].copy()
      selected_q.append(target.copy())
      lats.append(1000*(time.perf_counter()-start))
      for _ in range(ticks[t+1]-ticks[t]):
        dq=wrap(target-q)
        q=np.clip(q+np.clip(dq,-np.pi/rate,np.pi/rate),m.lower,m.upper)
        Qs.append(q.copy())
    Qs=np.array(Qs);selected_q=np.array(selected_q)
    werr=[];oerr=[];pserr=[]
    for t in range(n):
        q=Qs[ticks[t+1]-1];T=m.fk(q);Tref=sc['truthT'][t]
        werr.append(1000*np.linalg.norm(T[:3,3]-Tref[:3,3]))
        oerr.append(np.rad2deg(Rotation.from_matrix(T[:3,:3]@Tref[:3,:3].T).magnitude()))
        pserr.append(np.rad2deg(abs(float(wrap(m.psi(q)-sc['truepsi'][t])))))
    jump=np.rad2deg(np.max(np.abs(wrap(np.diff(selected_q,axis=0))),axis=1))
    jerk=np.diff(Qs,n=3,axis=0)*rate**3
    k=sc['drop1']; rec=jump[k-1:min(n-1,k+cam-1)]
    return dict(seed=sc['seed'],method=method,candidates_mean=float(np.mean(numcands)),infeasible_frames=invalid,
      wrist_mm_p95=float(np.percentile(werr,95)),wrist_ori_deg_p95=float(np.percentile(oerr,95)),
      psi_deg_p90=float(np.percentile(pserr,90)),recovery_req_jump_deg=float(np.max(rec)) if len(rec) else np.nan,
      target_jump_over60_count=int(np.sum(jump>60)),jerk_rms=float(np.sqrt(np.mean(jerk**2))),
      selection_ms_p95=float(np.percentile(lats,95)),min_joint_margin_deg=float(np.rad2deg(np.min(np.minimum(Qs-m.lower,m.upper-Qs)))))

def main():
  p=argparse.ArgumentParser();p.add_argument('--episodes',type=int,default=8)
  p.add_argument('--frames',type=int,default=30);p.add_argument('--grid',type=int,default=20)
  p.add_argument('--seed-start',type=int,default=39231);p.add_argument('--out',type=Path,default=Path('dynamic_results'))
  a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
  m=Nero.create();rows=[];st=time.perf_counter()
  for i in range(a.episodes):
    sc=scenario(m,a.seed_start+i,a.frames,a.grid)
    for method in METHODS:rows.append(evaluate(m,sc,method))
    print(f'dynamic wrist episodes {i+1}/{a.episodes}, elapsed {time.perf_counter()-st:.1f}s',flush=True)
  with (a.out/'per_episode.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,list(rows[0]));w.writeheader();w.writerows(rows)
  mets=['wrist_mm_p95','wrist_ori_deg_p95','psi_deg_p90','recovery_req_jump_deg','target_jump_over60_count','jerk_rms','infeasible_frames']
  s={m:{k:float(np.mean([r[k] for r in rows if r['method']==m])) for k in mets} for m in METHODS}
  result={'status':'EXPLORATORY_SYNTHETIC_NOT_JETSON','notes':['Moving wrist ground-truth SE3; wrist observation held across dropout, psi drift','Not prior 894 trajectories','Selection timing excludes per-frame IK bank build time, do not claim real time','No physical collision/acceleration dynamics'],'seed_start':a.seed_start,'episodes':a.episodes,'frames':a.frames,'psi_grid_deg':a.grid,'summary':s,'wall_s':time.perf_counter()-st}
  (a.out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
  print(json.dumps(s,indent=2),flush=True)
if __name__=='__main__': main()
