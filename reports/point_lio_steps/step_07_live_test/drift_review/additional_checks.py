"""Stationary repeatability, smoothed travel and surface-fit limitations."""
from pathlib import Path
import json,numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
base=Path(__file__).resolve().parent;out=base/'drive';a=np.load(out/'comparison.npz');t=a['time'];p=a['lio_position'];ref=a['reference_position'];cl=np.load(base.parent/'20261016T220926+1100/observed_outputs.npz');ct=(cl['cloud_stamp_ns']-cl['odom_stamp_ns'][0])/1e9;points=cl['cloud_points'];offsets=cl['cloud_offsets'];rng=np.random.default_rng(42)
def cloudwin(tm,half=1):
 ix=np.flatnonzero(abs(ct-tm)<=half);v=np.concatenate([points[offsets[k]:offsets[k+1]] for k in ix]);_,ind=np.unique(np.floor(v/.05).astype(np.int64),axis=0,return_index=True);return v[ind].astype(float)
# Same quiet start, independent disjoint scan windows; fixed map coordinates.
early=cloudwin(3);late=cloudwin(24);tree=cKDTree(early);dist,ind=tree.query(late);d2,ind2=cKDTree(late).query(early)
repeat={'early_window_s':[2,4],'late_window_s':[23,25],'nearest_neighbor_median_m':float(np.median(dist)),'nearest_neighbor_p90_m':float(np.percentile(dist,90)),'fraction_within_0_10m':float((dist<.1).mean()),'reverse_nearest_neighbor_median_m':float(np.median(d2)),'interpretation':'As-recorded clouds without additional registration; includes sparse sampling and visibility differences.'}
# Path length is smoothed to prevent centimetre stationary jitter inflating travel.
grid=np.arange(0,t[-1],.05);lp=np.column_stack([np.interp(grid,t,p[:,j]) for j in range(3)]);rp=np.column_stack([np.interp(grid,t,ref[:,j]) for j in range(3)])
travel=[]
for smooth in [.15,.3,.5]:
 l=gaussian_filter1d(lp,smooth/.05,axis=0);r=gaussian_filter1d(rp,smooth/.05,axis=0);ll=np.linalg.norm(np.diff(l,axis=0),axis=1).sum();rl=np.linalg.norm(np.diff(r,axis=0),axis=1).sum();travel.append({'gaussian_sigma_s':smooth,'lio_smoothed_path_length_m':float(ll),'vendor_smoothed_path_length_m':float(rl),'ratio':float(ll/rl)})
# Local floor planes: global height consistency is conditional on the floor being flat.
floors=[]
for center in np.arange(5,t[-1]-2,5):
 cp=np.array([np.interp(center,t,p[:,j]) for j in range(3)]);v=cloudwin(center,2);v=v[(np.linalg.norm(v[:,:2]-cp[:2],axis=1)<3)&(v[:,2]<cp[2]-.15)&(v[:,2]>cp[2]-.9)]
 if len(v)<150:continue
 v=v[::max(1,len(v)//5000)];xy=v[:,:2]-cp[:2];A=np.column_stack([xy,np.ones(len(v))]);best=None;bestn=0
 for _ in range(150):
  inds=rng.choice(len(v),3,replace=False)
  try:beta=np.linalg.solve(A[inds],v[inds,2])
  except np.linalg.LinAlgError:continue
  if np.linalg.norm(beta[:2])>np.tan(np.deg2rad(15)):continue
  keep=abs(A@beta-v[:,2])<.025
  if keep.sum()>bestn:best=keep;bestn=keep.sum()
 if best is None or bestn<.4*len(v):continue
 beta=np.linalg.lstsq(A[best],v[best,2],rcond=None)[0];res=A[best]@beta-v[best,2];floors.append({'time_s':float(center),'floor_z_at_robot_xy_m':float(beta[2]),'plane_slope_xy':beta[:2].tolist(),'tilt_deg':float(np.rad2deg(np.arctan(np.linalg.norm(beta[:2])))),'inlier_fraction':float(best.mean()),'inliers':int(bestn),'residual_rms_m':float(np.sqrt(np.mean(res**2))),'lio_imu_z_m':float(cp[2])})
surface=json.loads((out/'surface_consistency.json').read_text())
for item in surface['selected_revisits']:
 item['accepted_as_drift_measurement']=False;item['rejection_reason']='Opposite viewing directions expose different walls; only partial surfaces overlap. ICP exhausted 25 iterations, and translation/yaw corrections cannot be treated as measured drift.'
(out/'surface_consistency.json').write_text(json.dumps(surface,indent=2)+'\n')
report={'stationary_surface_repeatability':repeat,'smoothed_travel_comparison':travel,'local_floor_planes':floors,'floor_interpretation':'Floor map height change is a potential vertical map inconsistency only if the real floor remains the same level. Different floor sections and pose sway are not independently measured.','revisit_fit_accepted':False,'single_initial_vs_quiet_window_alignment':{'quiet_window_s':[3,25]}}
lr=Rotation.from_quat(a['lio_quaternion']);rr=Rotation.from_quat(a['reference_quaternion']);m=(t>3)&(t<25);rot=(lr[m]*rr[m].inv()).mean();tr=np.median(p[m]-rot.apply(ref[m]),axis=0);diff=p-(rot.apply(ref)+tr);report['single_initial_vs_quiet_window_alignment'].update(quiet_window_endpoint_difference_m=float(np.linalg.norm(diff[-1])),quiet_window_endpoint_xyz_difference_m=diff[-1].tolist())
for label in ['drive','restart']:
 d=np.load(base/label/'decoded.npz');it=d['imu_stamp_ns'];dt=np.diff(it)/1e9;vals=d['imu_values'];s=d['sport_stamp_ns'];sd=np.diff(s)/1e9
 report[label+'_imu_gaps']={'over_10ms':int((dt>.010).sum()),'over_20ms':int((dt>.020).sum()),'max_ms':float(dt.max()*1000),'unique_sport_gap_max_ms':float(np.diff(np.unique(s)).max()/1e6),'same_gap_in_sport_stamps':True,'gap_intervals_s':[((it[i]-d['lio_stamp_ns'][0])/1e9,float(dt[i])) for i in np.argsort(dt)[-10:][::-1]],'note':'No adapter stamp losses relative to recorded unique SportModeState stamps. Gaps already occur in recorded source times; recording alone cannot identify firmware vs upstream DDS loss.'}
(out/'additional_checks.json').write_text(json.dumps(report,indent=2)+'\n')
fig,axs=plt.subplots(1,2,figsize=(13,5),layout='constrained')
for v,col,label in [(early,'#1f77b4','2–4s'),(late,'#f28e2b','23–25s')]:
 wall=(v[:,2]>-.15)&(v[:,2]<1.6);axs[0].scatter(v[wall,0],v[wall,1],s=1,c=col,alpha=.45,label=label)
axs[0].axis('equal');axs[0].set(xlabel='map X (m)',ylabel='map Y (m)',title='Initial quiet period: wall scans overlay');axs[0].legend()
if floors:
 ft=[x['time_s'] for x in floors];fz=[x['floor_z_at_robot_xy_m'] for x in floors];axs[1].plot(ft,fz,'o-',label='Local fitted floor Z');axs[1].plot(t,p[:,2],label='Point-LIO IMU Z');axs[1].set(xlabel='Seconds after first pose',ylabel='map Z (m)',title='Floor consistency: conditional on a level real floor');axs[1].legend()
for ax in axs:ax.grid(alpha=.25)
fig.savefig(out/'stationary_and_floor.png',dpi=160);plt.close(fig)
print(json.dumps({'stationary':repeat,'travel':travel,'floor_samples':len(floors),'floor_first_last':floors[0::max(1,len(floors)-1)] if floors else [],'imu_gaps':report['drive_imu_gaps']},indent=2))
