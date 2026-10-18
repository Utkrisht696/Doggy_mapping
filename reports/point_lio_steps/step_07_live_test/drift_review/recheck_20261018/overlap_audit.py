"""Offline screen of shared wall geometry. Does not rerun/change Point-LIO."""
from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
b=Path('/home/jain/Doggy/reports/point_lio_steps/step_07_live_test');dest=Path(__file__).resolve().parent;a=np.load(b/'20261016T220926+1100/observed_outputs.npz');origin=int(a['odom_stamp_ns'][0]);t=(a['odom_stamp_ns']-origin)/1e9;p=a['odom_values'][:,:3];ct=(a['cloud_stamp_ns']-origin)/1e9;points=a['cloud_points'];off=a['cloud_offsets'];windows=[]
for tm in [10,35,45,55,65,75,85,95,105,115,125,132]:
 pos=np.array([np.interp(tm,t,p[:,j]) for j in range(3)]);ix=np.flatnonzero(abs(ct-tm)<=2.5);v=np.concatenate([points[off[k]:off[k+1]] for k in ix]);keep=(np.linalg.norm(v[:,:2]-pos[:2],axis=1)<9)&(v[:,2]>pos[2]+.15)&(v[:,2]<pos[2]+2);v=v[keep].astype(float)
 _,idx=np.unique(np.floor(v/.04).astype(np.int64),axis=0,return_index=True);v=v[idx];tree=cKDTree(v);dd,kn=tree.query(v,k=15);n=v[kn];d=n-n.mean(axis=1)[:,None];cov=np.einsum('nki,nkj->nij',d,d)/15;ev,evec=np.linalg.eigh(cov);norm=evec[:,:,0];mask=(ev[:,0]<.0036)&(ev[:,0]/np.maximum(ev.sum(axis=1),1e-12)<.06)&(ev[:,1]>.0004)&(abs(norm[:,2])<.5)&(dd[:,-1]<.8);v=v[mask];norm=norm[mask];windows.append({'time':tm,'pos':pos,'p':v,'norm':norm,'tree':cKDTree(v)});print(tm,len(v),'wall points',flush=True)
rows=[]
for i,e in enumerate(windows):
 for j,l in enumerate(windows[i+1:],i+1):
  if l['time']-e['time']<20:continue
  dd,nn=e['tree'].query(l['p']);mask=(dd<.4)&(abs(np.einsum('ij,ij->i',l['norm'],e['norm'][nn]))>.85);rd,rn=l['tree'].query(e['p']);rm=(rd<.4)&(abs(np.einsum('ij,ij->i',e['norm'],l['norm'][rn]))>.85)
  count=int(mask.sum());rc=int(rm.sum());frac=min(mask.mean(),rm.mean());normal=e['norm'][nn[mask],:2];evals=np.linalg.eigvalsh(normal.T@normal/max(count,1));rows.append({'early':e['time'],'late':l['time'],'source_matches':count,'target_matches':rc,'min_fraction':float(frac),'normal_diversity_min_eigenvalue':float(evals[0]),'raw_distance_median_m':float(np.median(dd[mask])) if count else None})
rows=sorted(rows,key=lambda x:x['min_fraction'],reverse=True);(dest/'wall_overlap_screen.json').write_text(json.dumps({'windows':[{'time_s':w['time'],'wall_points':len(w['p'])} for w in windows],'pairs':rows},indent=2)+'\n');np.savez_compressed(dest/'wall_windows.npz',**{f'{w["time"]}_{k}':w[k] for w in windows for k in ['p','norm','pos']});print('TOP',json.dumps(rows[:12],indent=2),flush=True)
