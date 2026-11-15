"""Minimal NERO zero-pose URDF kinematics and original project analytic IK for controlled test.
Joint origin / limit constants transcribed from dldbdlf/teleoperation share/sim/assets/nero_description.urdf.
Original analytic SEW kinematics: share/sim/kinematics.py (repository main).
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation

JOINTS = [
 ((0,0,.138),(0,0,0),(-2.70526,2.70526)),
 ((0,0,0),(1.5707963,3.1415926,0),(-1.74,1.74)),
 ((0,-.31,0),(-1.5707963,0,3.1415926926),(-2.75,2.75)),
 ((0,0,0),(-1.5707963,0,3.1415926926),(-1.01,2.14)),
 ((0,-.27001,0),(1.5707963,-1.5707963,0),(-2.75,2.75)),
 ((0,0,0),(1.5707963,-1.5707963,0),(-.7330383,.8726646)),
 ((0,-.0235,0),(1.5707963,0,0),(-1.5707963,1.5707963)),
]
ET=np.array([0.,0.,-1.]); ER=np.array([-1.,0.,0.])
def wrap(x): return (np.asarray(x)+np.pi)%(2*np.pi)-np.pi
def rot(axis,t):
 k=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0.]])
 return np.eye(3)+np.sin(t)*k+(1-np.cos(t))*(k@k)
def screw(w,p,t):
 R=rot(w,t); T=np.eye(4); T[:3,:3]=R;T[:3,3]=p-R@p
 return T
def sp1(w,r,p,q):
 u=p-r;v=q-r;U=u-w*(w@u);V=v-w*(w@v)
 return np.arctan2(w@np.cross(U,V),U@V)
def sp2(w1,w2,r,p,q):
 u=p-r;v=q-r;c=w1@w2;den=c*c-1
 alpha=(c*(w2@u)-w1@v)/den; beta=(c*(w1@v)-w2@u)/den
 n=np.cross(w1,w2); g2=(u@u-alpha**2-beta**2-2*alpha*beta*c)/(n@n)
 if g2 < -1e-9:return []
 out=[]
 for g in ([0.] if g2<1e-12 else [np.sqrt(g2),-np.sqrt(g2)]):
  z=alpha*w1+beta*w2+g*n+r
  out.append((sp1(w1,r,z,q),sp1(w2,r,p,z)))
 return out
def sp3(w,r,p,q,delta):
 u=p-r;v=q-r;U=u-w*(w@u);V=v-w*(w@v)
 d2=delta**2-(w@(p-q))**2;nu=np.linalg.norm(U);nv=np.linalg.norm(V)
 if nu*nv<1e-12:return []
 c=(nu**2+nv**2-d2)/(2*nu*nv)
 if abs(c)>1+1e-9:return []
 t0=np.arctan2(w@np.cross(U,V),U@V);a=np.arccos(np.clip(c,-1,1))
 return [t0] if a<1e-12 else [t0+a,t0-a]
def sew_frame(esw):
 kr=np.cross(esw-ET,ER);kx=np.cross(kr,esw);n=np.linalg.norm(kx)
 if n<1e-9:return None
 ex=kx/n;return ex,np.cross(esw,ex)
def sew_angle(S,E,W):
 v=W-S;n=np.linalg.norm(v)
 if n<1e-9:return np.nan
 sw=v/n; frame=sew_frame(sw)
 if frame is None:return np.nan
 d=E-S;d=d-sw*(sw@d)
 return float(np.arctan2(frame[1]@d,frame[0]@d))
def elbow_from_psi(S,W,psi,L1,L2):
 D=np.linalg.norm(W-S)
 if D<1e-9 or D>L1+L2+1e-7 or D<abs(L1-L2)-1e-7:return None
 D=np.clip(D,abs(L1-L2),L1+L2); sw=(W-S)/np.linalg.norm(W-S)
 frame=sew_frame(sw)
 if frame is None:return None
 a=(L1**2-L2**2+D**2)/(2*D);r=np.sqrt(max(0,L1**2-a**2))
 return S+a*sw+r*(np.cos(psi)*frame[0]+np.sin(psi)*frame[1])
@dataclass
class Nero:
 axes: np.ndarray
 points: np.ndarray
 M: np.ndarray
 lower:np.ndarray
 upper:np.ndarray
 S:np.ndarray
 E0:np.ndarray
 W0:np.ndarray
 @staticmethod
 def create():
  T=np.eye(4);axes=[];pts=[];lower=[];upper=[]
  for pos,rpy,lims in JOINTS:
   J=np.eye(4); J[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();J[:3,3]=pos
   T=T@J; axes.append(T[:3,:3]@np.array([0.,0.,1.]));pts.append(T[:3,3].copy());lower.append(lims[0]);upper.append(lims[1])
  axes=np.asarray(axes);pts=np.asarray(pts)
  return Nero(axes,pts,T,np.array(lower),np.array(upper),pts[1],pts[3],pts[4])
 @property
 def L_se(self):return np.linalg.norm(self.E0-self.S)
 @property
 def L_ew(self):return np.linalg.norm(self.W0-self.E0)
 def fk(self,q):
  T=np.eye(4)
  for a,p,t in zip(self.axes,self.points,q):T=T@screw(a,p,t)
  return T@self.M
 def sew_points(self,q):
  T=np.eye(4)
  for a,p,t in zip(self.axes[:4],self.points[:4],q[:4]):T=T@screw(a,p,t)
  return self.S,(T@np.r_[self.E0,1])[:3],(T@np.r_[self.W0,1])[:3]
 def psi(self,q):return sew_angle(*self.sew_points(q))
 def ik(self,T,psi):
  a,P=self.axes,self.points;g=T@np.linalg.inv(self.M);W=(g@np.r_[self.W0,1])[:3]
  sols=[]
  for t4 in sp3(a[3],P[3],self.W0,self.S,np.linalg.norm(W-self.S)):
   W4=(screw(a[3],P[3],t4)@np.r_[self.W0,1])[:3]
   E=elbow_from_psi(self.S,W,psi,self.L_se,np.linalg.norm(W4-self.E0))
   if E is None:continue
   for t1,t2 in sp2(a[0],a[1],self.S,self.E0,E):
    T12=screw(a[0],P[0],t1)@screw(a[1],P[1],t2)
    t3=sp1(a[2],P[2],W4,(np.linalg.inv(T12)@np.r_[W,1])[:3])
    T14=T12@screw(a[2],P[2],t3)@screw(a[3],P[3],t4)
    h=np.linalg.inv(T14)@g;p7=self.W0+0.1*a[6]
    for t5,t6 in sp2(a[4],a[5],self.W0,p7,(h@np.r_[p7,1])[:3]):
     T56=screw(a[4],P[4],t5)@screw(a[5],P[5],t6)
     p=self.W0+0.1*np.cross(a[6],a[6]+[0.3,0.5,0.7])/np.linalg.norm(np.cross(a[6],a[6]+[0.3,0.5,0.7]))
     t7=sp1(a[6],P[6],p,(np.linalg.inv(T56)@h@np.r_[p,1])[:3])
     sols.append(wrap(np.array([t1,t2,t3,t4,t5,t6,t7])))
  return sols
 def feasible(self,T,psi):return [q for q in self.ik(T,psi) if np.all(q>=self.lower-1e-6) and np.all(q<=self.upper+1e-6)]
READY=np.array([0.,.1,0.,1.5,-1.5708,0.,0.])
if __name__=='__main__':
 m=Nero.create();rng=np.random.default_rng(0)
 allerrs=[]; cnt=0
 for i in range(30):
  q=rng.uniform(m.lower+.12,m.upper-.12); T=m.fk(q);ps=m.psi(q)
  solutions=m.feasible(T,ps)
  errs=[np.linalg.norm(m.fk(sol)-T) for sol in solutions]
  if errs:allerrs.extend(errs);cnt+=1
 print('NERO IK recovery',cnt,'/30','solutions',len(allerrs),'max_fk_error',max(allerrs) if allerrs else None)
