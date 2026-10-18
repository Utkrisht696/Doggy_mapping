"""Local horizontal surface-consistency diagnostic; not trajectory ground truth."""
from pathlib import Path
import numpy as np,json
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
base=Path(__file__).resolve().parent;a=np.load(base/'wall_windows.npz');screen=json.loads((base/'wall_overlap_screen.json').read_text());results=[]
def window(tm):
 p=a[f'{tm}_p'];n=a[f'{tm}_norm'];good=abs(n[:,2])<.25;return p[good],n[good]
def rotate(p,theta):
 c,s=np.cos(theta),np.sin(theta);R=np.array([[c,-s,0],[s,c,0],[0,0,1]]);return p@R.T
for early,late in [(10,35),(10,45),(10,55),(10,65),(65,115),(45,125),(55,132)]:
 ref,rn=window(early);src,sn=window(late);center=a[f'{early}_pos'];ref=ref-center;src=src-center;tree=cKDTree(ref)
 def match(theta,tr,source=src,norm=sn):
  x=rotate(source,theta)+tr;nn=rotate(norm,theta);dist,idx=tree.query(x);keep=(dist<.4)&(abs(np.einsum('ij,ij->i',nn,rn[idx]))>.9);res=np.einsum('ij,ij->i',rn[idx[keep]],x[keep]-ref[idx[keep]]);return x,idx,keep,res
 def stats(theta,tr):
  x,idx,keep,res=match(theta,tr)
  return {'matches':int(keep.sum()),'source_fraction':float(keep.mean()),'median_abs_plane_residual_m':float(np.median(abs(res))) if len(res) else None,'p90_abs_plane_residual_m':float(np.percentile(abs(res),90)) if len(res) else None}
 before=stats(0,np.zeros(3));fits=[]
 # Fits use different starts. Half the source points are held out for validation.
 train=np.arange(len(src))%2==0
 for initial_deg in [-2,0,2]:
  theta=np.deg2rad(initial_deg);tr=np.zeros(3);converged=False;cond=None;rank=0
  for it in range(60):
   x,idx,keep,res=match(theta,tr,src[train],sn[train]);x=x[keep];n=rn[idx[keep]]
   if len(res)<80:break
   jac=np.column_stack([-x[:,1]*n[:,0]+x[:,0]*n[:,1],n[:,:2]]);weights=np.minimum(1,.025/np.maximum(abs(res),1e-10));jw=jac*np.sqrt(weights[:,None]);step,_,rank,sv=np.linalg.lstsq(jw,-res*np.sqrt(weights),rcond=None);cond=float(sv[0]/sv[-1]) if sv[-1]>0 else None
   if rank<3:break
   dt=step[0];theta+=dt;tr=rotate(tr[None],dt)[0]+[step[1],step[2],0]
   if abs(dt)<1e-5 and np.linalg.norm(step[1:])<1e-5:converged=True;break
  _,_,keep,res=match(theta,tr,src[~train],sn[~train]);fits.append({'initial_yaw_deg':initial_deg,'converged':converged,'iterations':it+1,'fit_translation_xy_m':tr[:2].tolist(),'fit_yaw_deg':float(np.rad2deg(theta)),'jacobian_condition_units_mixed':cond,'rank':int(rank),'heldout_matches':int(keep.sum()),'heldout_median_abs_plane_residual_m':float(np.median(abs(res))) if len(res) else None})
 spread=float(np.ptp(np.array([f['fit_translation_xy_m'] for f in fits]),axis=0).max());yawspread=float(np.ptp([f['fit_yaw_deg'] for f in fits]));screenrow=next(r for r in screen['pairs'] if r['early']==early and r['late']==late)
 accepted=all(f['converged'] and f['rank']==3 and f['heldout_matches']>=80 for f in fits) and screenrow['min_fraction']>=.3 and screenrow['normal_diversity_min_eigenvalue']>=.02 and spread<.02 and yawspread<.3
 fit=fits[1];r={'early_s':early,'late_s':late,'raw_wall_overlap_min_fraction':screenrow['min_fraction'],'before_fit':before,'fits':fits,'initialization_translation_spread_m':spread,'initialization_yaw_spread_deg':yawspread,'accepted_as_local_surface_consistency':bool(accepted),'accepted_as_global_drift':False,'local_correction_norm_m':float(np.linalg.norm(fit['fit_translation_xy_m'])),'local_correction_yaw_deg':fit['fit_yaw_deg'],'note':'Rigid adjustment of shared wall returns in the already registered map. Robust nearest-neighbour fit may be biased by limited visibility; not independent position truth or end-of-route drift.'};results.append(r)
report={'method':'Wall-normal matching, horizontal translation/yaw point-to-plane fit, 3 initial yaw values, alternate source-point holdout. No estimator, TF or sensor configuration changes.','results':results,'global_horizontal_drift_measured':False};(base/'shared_wall_fits.json').write_text(json.dumps(report,indent=2)+'\n')
fig,axs=plt.subplots(1,2,figsize=(12,5),layout='constrained')
for tm,color in [(10,'#1f77b4'),(45,'#f28e2b')]:
 v,_=window(tm);axs[0].scatter(v[:,0],v[:,1],c=color,s=3,alpha=.5,label=f'{tm}s')
axs[0].axis('equal');axs[0].set(xlabel='map X (m)',ylabel='map Y (m)',title='Shared walls, as recorded');axs[0].legend()
labels=[f"{r['early_s']} → {r['late_s']}s" for r in results];values=[r['before_fit']['median_abs_plane_residual_m']*100 if r['before_fit']['median_abs_plane_residual_m'] is not None else 0 for r in results];cols=['#2ca02c' if r['accepted_as_local_surface_consistency'] else '#999999' for r in results];axs[1].bar(range(len(values)),values,color=cols);axs[1].set_xticks(range(len(values)),labels,rotation=35,ha='right');axs[1].set(ylabel='Median absolute wall-plane residual (cm)',title='Local surface mismatch; grey = insufficient evidence')
for ax in axs:ax.grid(alpha=.2)
fig.savefig(base/'shared_wall_check.png',dpi=160);plt.close(fig)
print(json.dumps([{'early_s':r['early_s'],'late_s':r['late_s'],'plane_error':r['before_fit'],'correction_m':r['local_correction_norm_m'],'yaw_deg':r['local_correction_yaw_deg'],'accepted_local':r['accepted_as_local_surface_consistency'],'converged':[x['converged'] for x in r['fits']],'spread_m':r['initialization_translation_spread_m']} for r in results],indent=2))
