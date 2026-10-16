"""Use the user-identified low drain at the first right turn as a terrain constraint."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
base=Path(__file__).resolve().parent;out=base/'drive';a=np.load(out/'comparison.npz');t=a['time'];p=a['lio_position'];cl=np.load(base.parent/'20261016T220926+1100/observed_outputs.npz');ct=(cl['cloud_stamp_ns']-cl['odom_stamp_ns'][0])/1e9;pts=cl['cloud_points'];offs=cl['cloud_offsets'];rng=np.random.default_rng(13)
user={'statement':'check where i am saying robot turned 90 degree that spot is lower than everywhere else so you can use that as a ref','constraint':'The first roughly 90-degree right-turn/drain area is lower than the other sections of this route. Qualitative terrain ordering, not a measured surveyed height.','source':'User observation','turn_region_source_time_s':[65,75]};(out/'drain_reference_user.json').write_text(json.dumps(user,indent=2)+'\n')
def cloudwin(tm,half=2):
 ix=np.flatnonzero(abs(ct-tm)<=half);v=np.concatenate([pts[offs[i]:offs[i+1]] for i in ix]);_,ind=np.unique(np.floor(v/.03).astype(np.int64),axis=0,return_index=True);return v[ind].astype(float)
def floor(tm,radius):
 cp=np.array([np.interp(tm,t,p[:,j]) for j in range(3)]);v=cloudwin(tm);v=v[(np.linalg.norm(v[:,:2]-cp[:2],axis=1)<radius)&(v[:,2]<cp[2]-.15)&(v[:,2]>cp[2]-.9)]
 if len(v)<50:return None
 xy=v[:,:2]-cp[:2];A=np.column_stack([xy,np.ones(len(v))]);best=None;bestn=0
 for _ in range(250):
  ix=rng.choice(len(v),3,replace=False)
  try:b=np.linalg.solve(A[ix],v[ix,2])
  except np.linalg.LinAlgError:continue
  if np.linalg.norm(b[:2])>np.tan(np.deg2rad(15)):continue
  keep=abs(A@b-v[:,2])<.02
  if keep.sum()>bestn:best=keep;bestn=int(keep.sum())
 if best is None or bestn<.3*len(v):return None
 b=np.linalg.lstsq(A[best],v[best,2],rcond=None)[0];return {'time_s':tm,'radius_m':radius,'floor_z_at_robot_xy_m':float(b[2]),'tilt_deg':float(np.rad2deg(np.arctan(np.linalg.norm(b[:2])))),'inliers':bestn,'inlier_fraction':float(best.mean()),'rms_m':float(np.sqrt(np.mean((A[best]@b-v[best,2])**2)))}
fits=[]
for radius in [1.0,1.5,2.0,3.0]:
 for tm in [10,25,65,68,70,73,75,114,115,116,125,130,133]:
  f=floor(tm,radius)
  if f:fits.append(f)
# Same XY floor-cell comparison on outward and return passes at the drain.
turn=(t>=68)&(t<=74);drain=np.median(p[turn,:2],axis=0)
def cells(start,end):
 ix=np.flatnonzero((ct>=start)&(ct<=end));v=np.concatenate([pts[offs[i]:offs[i+1]] for i in ix]);v=v[(np.linalg.norm(v[:,:2]-drain,axis=1)<2)&(v[:,2]>-.85)&(v[:,2]<-.20)];keys=np.floor(v[:,:2]/.15).astype(np.int64);ids,inv,cnt=np.unique(keys,axis=0,return_inverse=True,return_counts=True);s=np.argsort(inv);order=v[s,2];ends=np.r_[0,np.cumsum(cnt)];return {tuple(k):float(np.median(order[ends[i]:ends[i+1]])) for i,k in enumerate(ids) if cnt[i]>=5}
early=cells(61,75);late=cells(113,119);common=sorted(early.keys()&late.keys());delta=np.array([late[k]-early[k] for k in common]);revisit={'outward_window_s':[61,75],'return_window_s':[113,119],'landmark_map_xy_m':drain.tolist(),'radius_m':2,'common_15cm_cells':len(common),'median_late_minus_early_floor_z_m':float(np.median(delta)) if len(delta) else None,'median_absolute_floor_height_difference_m':float(np.median(abs(delta))) if len(delta) else None,'p90_absolute_floor_height_difference_m':float(np.percentile(abs(delta),90)) if len(delta) else None,'limitation':'Same map-XY cells assume adequate horizontal registration; mixture of points/terrain and sparse samples can affect height.'}
comparisons=[]
for radius in [1.,1.5,2.,3.]:
 drainfits=[f['floor_z_at_robot_xy_m'] for f in fits if f['radius_m']==radius and 65<=f['time_s']<=75];endfits=[f['floor_z_at_robot_xy_m'] for f in fits if f['radius_m']==radius and 125<=f['time_s']<=133];startfits=[f['floor_z_at_robot_xy_m'] for f in fits if f['radius_m']==radius and f['time_s']<=25]
 if drainfits and endfits:comparisons.append({'radius_m':radius,'median_start_floor_z_m':float(np.median(startfits)) if startfits else None,'median_drain_floor_z_m':float(np.median(drainfits)),'median_final_section_floor_z_m':float(np.median(endfits)),'final_section_lower_than_drain_m':float(np.median(drainfits)-np.median(endfits))})
q=Rotation.from_quat(a['lio_quaternion']);yaw=np.unwrap(q.as_euler('xyz')[:,2]);idx=[np.argmin(abs(t-s)) for s in [65,71]];turnangle=float(np.rad2deg(yaw[idx[1]]-yaw[idx[0]]))
report={'user_terrain_reference':user,'detected_first_right_turn_yaw_change_deg_65_to_71_s':turnangle,'local_floor_plane_radius_sensitivity':comparisons,'floor_plane_fits':fits,'drain_revisit_same_xy':revisit,'interpretation':'If the user-identified drain really is the lowest section, a reconstructed later section below that drain violates the observed terrain ordering. This is evidence of residual vertical map inconsistency, but the physical height difference is not surveyed, so the true drift cannot be assigned an exact value.','no_application_or_estimator_changes':True};(out/'drain_reference_check.json').write_text(json.dumps(report,indent=2)+'\n')
fig,axs=plt.subplots(1,3,figsize=(17,5.2),layout='constrained');seg=np.stack([p[:-1,:2],p[1:,:2]],axis=1);lc=LineCollection(seg,cmap='viridis',norm=plt.Normalize(t[0],t[-1]),linewidth=2);lc.set_array(t[:-1]);axs[0].add_collection(lc);axs[0].autoscale();axs[0].scatter(*drain,s=130,facecolors='none',edgecolors='red',linewidths=2,label='First right turn / drain');axs[0].scatter(*p[-1,:2],marker='X',c='black',s=55,label='End');axs[0].set_aspect('equal',adjustable='datalim');axs[0].set(xlabel='map X (m)',ylabel='map Y (m)',title='Terrain reference identified by user');axs[0].legend(fontsize=8);fig.colorbar(lc,ax=axs[0],label='Seconds',shrink=.7)
local=json.loads((out/'additional_checks.json').read_text())['local_floor_planes'];ft=np.array([x['time_s'] for x in local]);fz=np.array([x['floor_z_at_robot_xy_m'] for x in local]);dr=np.median([x['floor_z_at_robot_xy_m'] for x in fits if x['radius_m']==1.5 and 65<=x['time_s']<=75]);axs[1].plot(ft,fz,'o-',label='Local floor Z (3 m fits)');axs[1].axvspan(65,75,color='red',alpha=.12,label='First right turn');axs[1].axhline(dr,color='red',ls='--',label='Drain estimate (1.5 m fits)');axs[1].set(xlabel='Seconds after first pose',ylabel='map floor Z (m)',title='Final section maps below the known low area');axs[1].legend(fontsize=8)
if common:
 xy=(np.array(common)+.5)*.15;sc=axs[2].scatter(xy[:,0],xy[:,1],c=delta*100,cmap='coolwarm',vmin=-8,vmax=8,s=18);axs[2].set_aspect('equal');axs[2].set(xlabel='map X (m)',ylabel='map Y (m)',title=f'Drain repeat: {len(common)} shared floor cells');fig.colorbar(sc,ax=axs[2],label='Return minus outward height (cm)',shrink=.7)
for ax in axs:ax.grid(alpha=.2)
fig.savefig(out/'drain_reference.png',dpi=160);plt.close(fig)
print(json.dumps({'turn_deg':turnangle,'height_comparisons':comparisons,'drain_revisit':revisit},indent=2))
