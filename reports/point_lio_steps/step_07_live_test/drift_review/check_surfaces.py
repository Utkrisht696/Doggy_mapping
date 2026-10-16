"""Local overlap consistency, not independent trajectory ground truth."""
from pathlib import Path
import numpy as np,json
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
base=Path(__file__).resolve().parent;dest=base/'drive';a=np.load(dest/'comparison.npz');t=a['time'];p=a['lio_position'];source=base.parent/'20261016T220926+1100';cl=np.load(source/'observed_outputs.npz');cs=(cl['cloud_stamp_ns']-cl['odom_stamp_ns'][0])/1e9;points=cl['cloud_points'];offsets=cl['cloud_offsets']
def down(x,v=.05):
 if not len(x):return x
 _,ix=np.unique(np.floor(x/v).astype(np.int64),axis=0,return_index=True);return x[ix]
def window(center,ref,span=1.5):
 ix=np.flatnonzero(abs(cs-center)<=span);x=np.concatenate([points[offsets[k]:offsets[k+1]] for k in ix]);x=x[np.linalg.norm(x-ref,axis=1)<6];return down(x)
candidates=[]
for late in np.arange(45,t[-1]-2,3):
 j=np.argmin(abs(t-late));prior=(t>=3)&(t<late-20);ind=np.flatnonzero(prior);i=ind[np.argmin(np.linalg.norm(p[ind]-p[j],axis=1))];dist=np.linalg.norm(p[i]-p[j])
 if dist<.85:candidates.append((dist,float(t[i]),float(t[j])))
selected=[]
for candidate in sorted(candidates):
 if all(abs(candidate[2]-s[2])>9 for s in selected):selected.append(candidate)
 if len(selected)>=4:break
results=[];fig,axs=plt.subplots(max(1,len(selected)),2,figsize=(12,max(4,len(selected)*3.4)),squeeze=False,layout='constrained')
for row,(separation,early,late) in enumerate(selected):
 pe=np.array([np.interp(early,t,p[:,j]) for j in range(3)]);pl=np.array([np.interp(late,t,p[:,j]) for j in range(3)]);center=(pe+pl)/2
 ref=window(early,center).astype(float)-center;src=window(late,center).astype(float)-center
 tree=cKDTree(ref);_,kn=tree.query(ref,k=15,workers=1);neighbors=ref[kn];dev=neighbors-neighbors.mean(axis=1)[:,None];cov=np.einsum('nki,nkj->nij',dev,dev)/15;ev,evec=np.linalg.eigh(cov);normals=evec[:,:,0];planar=(ev[:,0]<.0016)&(ev[:,0]/np.maximum(ev.sum(axis=1),1e-12)<.08)&(ev[:,1]>.0001)
 def residuals(x):
  dist,ind=tree.query(x,workers=1);keep=(dist<.35)&planar[ind];signed=np.einsum('ij,ij->i',normals[ind[keep]],x[keep]-ref[ind[keep]]);return dist,ind,keep,signed
 def residualstats(x):
  dd,ii,keep,signed=residuals(x)
  return {'source_points':len(x),'overlap_fraction_within_0_35m':float(keep.mean()),'planar_correspondences':int(keep.sum()),'absolute_point_to_plane_residual_m':dict(zip(['median','p90','p95'],map(float,np.percentile(abs(signed),[50,90,95]))))}
 before=residualstats(src);rot=Rotation.identity();trans=np.zeros(3);condition=None
 for iteration in range(25):
  x=rot.apply(src)+trans;dd,ind,keep,signed=residuals(x);xk=x[keep];n=normals[ind[keep]];jac=np.column_stack([np.cross(xk,n),n]);w=np.minimum(1,.03/np.maximum(abs(signed),1e-8));jw=jac*np.sqrt(w[:,None]);delta,_,rank,sv=np.linalg.lstsq(jw,-signed*np.sqrt(w),rcond=None);condition=float(sv[0]/sv[-1]) if sv[-1]>0 else None
  if rank<6:break
  dr=Rotation.from_rotvec(delta[:3]);rot=dr*rot;trans=dr.apply(trans)+delta[3:]
  if np.linalg.norm(delta[:3])<1e-5 and np.linalg.norm(delta[3:])<1e-5:break
 after=residualstats(rot.apply(src)+trans)
 r={'early_time_s':early,'late_time_s':late,'estimated_imu_position_separation_m':float(separation),'early_submap_points':len(ref),'late_submap_points':len(src),'window_half_width_s':1.5,'before_fit':before,'after_local_fit':after,'local_fit_rotation_deg':float(np.rad2deg(rot.magnitude())),'local_fit_translation_at_overlap_center_m':float(np.linalg.norm(trans)),'local_fit_translation_xyz_m':trans.tolist(),'fit_jacobian_condition_number_units_mixed':condition,'iterations':iteration+1,'interpretation':'Small correction means local registered surfaces agree at this overlap. ICP is not independent truth; moving objects, viewpoint, sparse returns and underconstrained planes affect the fit. No trajectory or application changed.'};results.append(r)
 ax=axs[row,0]
 for cloud,color,label in [(ref,'#1f77b4',f'Early {early:.1f}s'),(src,'#f28e2b',f'Late {late:.1f}s')]:
  # Walls carry more horizontal alignment information than the floor.
  z=cloud[:,2]+center[2];keep=(z>-.15)&(z<1.7);v=cloud[keep];ax.scatter(v[:,0]+center[0],v[:,1]+center[1],s=1,c=color,alpha=.5,label=label)
 ax.set(xlabel='map X (m)',ylabel='map Y (m)',title='Recorded wall-height points, before local fit');ax.axis('equal');ax.legend(fontsize=8);ax.grid(alpha=.2)
 ax=axs[row,1];d,ii,k,s=residuals(src);ax.hist(abs(s),bins=np.linspace(0,.25,60),histtype='step',label='As recorded',density=True);d,ii,k,s=residuals(rot.apply(src)+trans);ax.hist(abs(s),bins=np.linspace(0,.25,60),histtype='step',label='After diagnostic local fit',density=True);ax.set(xlabel='Absolute local surface residual (m)',ylabel='Density',title=f'Correction: {np.linalg.norm(trans)*100:.1f} cm / {np.rad2deg(rot.magnitude()):.2f}°');ax.legend(fontsize=8);ax.grid(alpha=.2)
if not selected:axs[0,0].text(.1,.5,'No overlapping revisit candidates within screening thresholds');axs[0,1].axis('off')
fig.suptitle('Local revisits: recorded surface consistency (no mapper rerun)',fontsize=14);fig.savefig(dest/'surface_consistency.png',dpi=160);plt.close(fig)
report={'method':'Point-to-plane ICP of 3-second map-frame scan windows, 5 cm voxels, 6 m local radius, <35 cm correspondence gate; Huber weighting 3 cm. Local diagnostic fit only.','revisit_candidates':len(candidates),'selected_revisits':results,'no_external_ground_truth':True};(dest/'surface_consistency.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
