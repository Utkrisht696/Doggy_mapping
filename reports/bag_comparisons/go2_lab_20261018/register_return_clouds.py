"""Conditional static-scene return registration, independent of robot odometry."""
from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
r=Path(__file__).resolve().parent;a=np.load(r/'return_source_clouds.npz')
def voxel(p,size=.07):
 keys=np.floor(p/size).astype('int64');_,inv,count=np.unique(keys,axis=0,return_inverse=True,return_counts=True);out=np.zeros((len(count),3))
 for j in range(3):np.add.at(out[:,j],inv,p[:,j])
 return out/count[:,None]
def normals(p):
 tree=cKDTree(p);dist,ix=tree.query(p,k=16);v=p[ix]-p[ix].mean(axis=1)[:,None,:];cov=np.einsum('nki,nkj->nij',v,v)/16;lam,vec=np.linalg.eigh(cov);good=(dist[:,-1]<.5)&(lam[:,0]/np.maximum(lam.sum(axis=1),1e-10)<.05)&(lam[:,1]>.0001);return vec[:,:,0],good

def fit(target,source,guess=None):
 target=voxel(target);source=voxel(source);n,good=normals(target);target=target[good];n=n[good];tree=cKDTree(target);R=np.eye(3) if guess is None else guess[:3,:3].copy();T=np.zeros(3) if guess is None else guess[:3,3].copy();last={}
 for limit in [.4,.2,.12]:
  for iteration in range(35):
   p=source@R.T+T;d,ix=tree.query(p);mask=d<limit
   if mask.sum()<100:raise RuntimeError('Insufficient correspondences')
   p=p[mask];q=target[ix[mask]];nn=n[ix[mask]];res=np.sum(nn*(p-q),axis=1);trim=abs(res)<min(limit,.12);p=p[trim];q=q[trim];nn=nn[trim];res=res[trim];weight=np.minimum(1.,.025/np.maximum(abs(res),1e-8));A=np.c_[nn,np.cross(p,nn)];Aw=A*np.sqrt(weight)[:,None];b=-res*np.sqrt(weight);delta,_,rank,sing=np.linalg.lstsq(Aw,b,rcond=None)
   if rank<6:raise RuntimeError('Degenerate geometry')
   dr=Rotation.from_rotvec(delta[3:]).as_matrix();R=dr@R;T=dr@T+delta[:3];last={'correspondences':len(p),'source_voxels':len(source),'target_plane_voxels':len(target),'fraction_matched':float(len(p)/len(source)),'plane_abs_residual_median_m':float(np.median(abs(res))),'plane_abs_residual_p90_m':float(np.percentile(abs(res),90)),'normal_matrix_condition':float((sing[0]/sing[-1])**2),'translation_normal_eigenvalues':np.linalg.eigvalsh(nn.T@nn/len(nn)).tolist(),'iterations_last_stage':iteration+1}
   if np.linalg.norm(delta[:3])<1e-5 and np.linalg.norm(delta[3:])<1e-5:break
 matrix=np.eye(4);matrix[:3,:3]=R;matrix[:3,3]=T;last.update(transform_finish_imu_to_start_imu=matrix.tolist(),translation_m=T.tolist(),rotation_rpy_deg=Rotation.from_matrix(R).as_euler('xyz',degrees=True).tolist(),rotation_angle_deg=float(Rotation.from_matrix(R).magnitude()*180/np.pi));return matrix,last
start,finish=a['start_points'],a['finish_points'];trials=[];matrices=[]
for t,yaw in [(np.zeros(3),0),([.1,0,0],0),([-.1,0,0],0),([0,.1,0],2),([0,-.1,0],-2)]:
 guess=np.eye(4);guess[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix();guess[:3,3]=t
 try:
  m,report=fit(start,finish,guess);report['initial_translation_m']=np.asarray(t).tolist();report['initial_yaw_deg']=yaw;trials.append(report);matrices.append(m)
 except Exception as error:trials.append({'error':str(error)})
checks=[]
for si in range(2):
 for fi in range(2):
  sidx=np.arange(len(a['start_time']))[si::2];fidx=np.arange(len(a['finish_time']))[fi::2];sp=np.concatenate([start[a['start_offsets'][i]:a['start_offsets'][i+1]] for i in sidx]);fp=np.concatenate([finish[a['finish_offsets'][i]:a['finish_offsets'][i+1]] for i in fidx])
  try:m,rep=fit(sp,fp);rep['start_frame_parity']=si;rep['finish_frame_parity']=fi;checks.append(rep)
  except Exception as error:checks.append({'error':str(error)})
valid=[x for x in trials if 'translation_m' in x];splits=[x for x in checks if 'translation_m' in x];accepted=False
if len(valid)==5 and len(splits)==4:
 ts=np.array([x['translation_m'] for x in valid]);rots=Rotation.from_matrix(np.array([x[:3,:3] for x in matrices]));spread_t=float(np.max(np.linalg.norm(ts-ts[0],axis=1)));spread_r=float(np.max((rots[0].inv()*rots).magnitude())*180/np.pi);split_ts=np.array([x['translation_m'] for x in splits]);split_spread=float(np.max(np.linalg.norm(split_ts-ts[0],axis=1)));accepted=spread_t<.02 and spread_r<.3 and split_spread<.03 and min(x['fraction_matched'] for x in valid)>.35 and max(x['normal_matrix_condition'] for x in valid)<1e4 and max(x['plane_abs_residual_p90_m'] for x in valid)<.05
else:spread_t=spread_r=split_spread=None
report={'method':'Robust point-to-plane ICP on voxelized source raw clouds transformed by fixed approved sensor extrinsics. No vendor/Point-LIO poses used. Frames pooled only within stationary start/end holds; no trajectory alignment.','start_window_s':[2.6,3.6],'finish_window_s':[210,216],'trials':trials,'interleaved_frame_checks':checks,'translation_seed_spread_m':spread_t,'rotation_seed_spread_deg':spread_r,'split_frame_translation_spread_m':split_spread,'passes_consistency_gates':bool(accepted),'limitations':'Conditional static-scene geometric check, not surveyed ground truth. People, changed objects, sparse surfaces and robot body sway can affect it. Accepted only if seeds, split samples, overlap, residuals and geometry conditioning agree.'};(r/'return_registration.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
