"""Lab loop return checks; no surveyed pose or route length is assumed."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
trial=Path(__file__).resolve().parent;r=trial/'run_01';baseline=Path('/home/jain/Doggy/reports/bag_comparisons/go2_lab_20261018');out=r/'analysis';out.mkdir(exist_ok=True);s=np.load(baseline/'source_decoded.npz');a=np.load(r/'outputs.npz');origin=int(s['raw_stamp_ns'][0]);t=(a['odom_stamp_ns']-origin)/1e9;p=a['odom_values'][:,:3];lr=Rotation.from_quat(a['odom_values'][:,3:7]);vt=(s['vendor_stamp_ns']-origin)/1e9;v=s['vendor_values'];vr=Slerp(vt,Rotation.from_quat(v[:,3:7]))(t);vp=np.column_stack([np.interp(t,vt,v[:,j]) for j in range(3)])+vr.apply([-.02557,0,.04232]);windows={'start':[2.6,3.6],'finish':[210.,216.]};holds={}
for name,(start,end) in windows.items():
 mask=(t>=start)&(t<=end);assert mask.sum()>=10;hold={'window_source_s':[start,end],'samples':int(mask.sum())}
 for label,points,rot in [('point_lio',p,lr),('vendor',vp,vr)]:
  pts=points[mask];med=np.median(pts,axis=0);hold[label]={'median_xyz_m':med.tolist(),'mean_quaternion_xyzw':rot[mask].mean().as_quat().tolist(),'position_p95_radius_m':float(np.percentile(np.linalg.norm(pts-med,axis=1),95)),'median_first_last_8_samples_shift_m':float(np.linalg.norm(np.median(pts[-8:],axis=0)-np.median(pts[:8],axis=0)))}
 holds[name]=hold
returns={}
for label in ['point_lio','vendor']:
 begin=holds['start'][label];end=holds['finish'][label];delta=np.array(end['median_xyz_m'])-begin['median_xyz_m'];rb=Rotation.from_quat(begin['mean_quaternion_xyzw']);re=Rotation.from_quat(end['mean_quaternion_xyzw']);rel=rb.inv()*re;y0=rb.as_euler('xyz')[2];y1=re.as_euler('xyz')[2];heading=(y1-y0+np.pi)%(2*np.pi)-np.pi
 returns[label]={'horizontal_return_offset_m':float(np.linalg.norm(delta[:2])),'height_return_change_m':float(delta[2]),'return_3d_offset_m':float(np.linalg.norm(delta)),'heading_return_change_deg':float(np.rad2deg(heading)),'full_orientation_change_deg':float(np.rad2deg(rel.magnitude())),'relative_pose_translation_start_imu_axes_m':rb.inv().apply(delta).tolist(),'relative_pose_quaternion_xyzw':rel.as_quat().tolist(),'delta_map_axes_xyz_m':delta.tolist()}
ly=Rotation.from_quat(holds['start']['point_lio']['mean_quaternion_xyzw']).as_euler('xyz')[2];vy=Rotation.from_quat(holds['start']['vendor']['mean_quaternion_xyzw']).as_euler('xyz')[2];lp=Rotation.from_euler('z',-ly).apply(p-holds['start']['point_lio']['median_xyz_m']);rp=Rotation.from_euler('z',-vy).apply(vp-holds['start']['vendor']['median_xyz_m']);dt=np.diff(t);step=np.linalg.norm(np.diff(p,axis=0),axis=1);angle=np.rad2deg((lr[:-1].inv()*lr[1:]).magnitude());contig=dt<.15
# A path estimate with explicit treatment of recording gaps; never a measured distance.
def path_length(points):
 lengths=[];groups=np.split(np.arange(len(t)),np.flatnonzero(dt>.15)+1)
 for group in groups:
  tt=t[group];pp=points[group]
  if len(tt)<3:continue
  grid=np.arange(tt[0],tt[-1]+1e-8,.1)
  if len(grid)<2:continue
  pg=np.column_stack([np.interp(grid,tt,pp[:,j]) for j in range(3)]);pg=gaussian_filter1d(pg,2.,axis=0,mode='nearest');lengths.append(float(np.linalg.norm(np.diff(pg,axis=0),axis=1).sum()))
 return {'smoothed_observed_path_excluding_gaps_m':float(sum(lengths)),'straight_chord_lengths_across_gaps_m':[float(x) for x in np.linalg.norm(np.diff(points,axis=0)[~contig],axis=1)],'method':'Separate contiguous segments, 0.1 s resampling, Gaussian sigma 0.2 s. Gaps >0.15 s are excluded. This is estimator-derived travel, not surveyed distance.'}
def stats(x):return dict(zip(['median','p95','p99','max'],map(float,np.percentile(x,[50,95,99,100]))))
gap_events=[]
for ix in np.flatnonzero(~contig):
 gap_events.append({'before_source_s':float(t[ix]),'after_source_s':float(t[ix+1]),'dt_s':float(dt[ix]),'lio_pose_change_m':float(step[ix]),'lio_rotation_change_deg':float(angle[ix])})
health={'pose_count':len(t),'first_pose_s':float(t[0]),'last_pose_s':float(t[-1]),'nonfinite_pose_values':int((~np.isfinite(a['odom_values'])).sum()),'nonincreasing_pose_stamps':int((dt<=0).sum()),'average_rate_including_gaps_hz':float((len(t)-1)/(t[-1]-t[0])),'median_contiguous_rate_hz':float(1/np.median(dt[contig])),'contiguous_dt_s':stats(dt[contig]),'contiguous_translation_steps_m':stats(step[contig]),'contiguous_rotation_steps_deg':stats(angle[contig]),'contiguous_steps_over_15cm_count':int((step[contig]>.15).sum()),'contiguous_steps_over_5deg_count':int((angle[contig]>5).sum()),'recording_gap_pose_changes':gap_events,'tf_full_pose_stamp_match':bool(np.array_equal(a['odom_stamp_ns'],a['map_odom_stamp_ns'])),'tf_missing_pose_count':int(len(np.setdiff1d(a['odom_stamp_ns'],a['map_odom_stamp_ns']))),'tf_extra_stamp_count':int(len(np.setdiff1d(a['map_odom_stamp_ns'],a['odom_stamp_ns'])))}
# Same floor-candidate cells in unadjusted map coordinates, no registration/levelling.
ct=(a['cloud_stamp_ns']-origin)/1e9;points=a['cloud_points'];offset=a['cloud_offsets'];initial=np.array(holds['start']['point_lio']['median_xyz_m'])
def floor_cells(name):
 start,end=windows[name];ix=np.flatnonzero((ct>=start)&(ct<=end));pp=np.concatenate([points[offset[k]:offset[k+1]] for k in ix]).astype(float);pp=pp[(np.linalg.norm(pp[:,:2]-initial[:2],axis=1)<3)&(pp[:,2]>initial[2]-.9)&(pp[:,2]<initial[2]-.15)];ids,inv,cnt=np.unique(np.floor(pp[:,:2]/.15).astype('int64'),axis=0,return_inverse=True,return_counts=True);ii=np.argsort(inv);z=pp[ii,2];endpoints=np.r_[0,np.cumsum(cnt)];return {tuple(key):float(np.median(z[endpoints[i]:endpoints[i+1]])) for i,key in enumerate(ids) if cnt[i]>=5}
f0,f1=floor_cells('start'),floor_cells('finish');common=sorted(f0.keys()&f1.keys());dz=np.array([f1[k]-f0[k] for k in common]);floor={'common_15cm_cells':len(common),'method':'Unadjusted map XY cells, >=5 points each, within 3 m and .15-.9 m below original IMU. Assumes sufficient horizontal correspondence; not absolute height accuracy.'}
if len(dz):floor.update(median_signed_difference_m=float(np.median(dz)),median_absolute_difference_m=float(np.median(abs(dz))),p90_absolute_difference_m=float(np.percentile(abs(dz),90)))
registration=json.loads((baseline/'return_registration.json').read_text());geometric={}
if registration['passes_consistency_gates']:
 fit=registration['trials'][0];truth=np.array(fit['transform_finish_imu_to_start_imu']);rt=Rotation.from_matrix(truth[:3,:3])
 for label,ret in returns.items():
  delta=np.array(ret['relative_pose_translation_start_imu_axes_m'])-truth[:3,3];rr=Rotation.from_quat(ret['relative_pose_quaternion_xyzw']);geometric[label]={'relative_translation_disagreement_xyz_m':delta.tolist(),'relative_translation_disagreement_3d_m':float(np.linalg.norm(delta)),'relative_translation_disagreement_xy_m':float(np.linalg.norm(delta[:2])),'relative_rotation_disagreement_deg':float(np.rad2deg((rt.inv()*rr).magnitude()))}
report={'source_bag':'/home/jain/go2_bags/go2_20261018_173219','user_route':'Moved around the lab, returning roughly to the same starting position and heading. No surveyed route length or endpoint error supplied.','source_seconds_reference':'First original raw-cloud header timestamp','holds':holds,'return_checks':returns,'estimated_travel':{'point_lio':path_length(p),'vendor':path_length(vp)},'health':health,'revisited_floor':floor,'independent_source_cloud_registration_consistent':registration['passes_consistency_gates'],'relative_pose_disagreement_vs_static_scene_registration':geometric,'source_gaps':json.loads((baseline/'large_source_gaps.json').read_text()),'stationary_start':json.loads((baseline/'stationary_start.json').read_text()),'replay_result':json.loads((r/'result.json').read_text()),'limitations':['Physical return position/heading only approximate, so median return offset is not pure estimator drift.','Sensor recording contains an approximately four-second gap during motion and a shorter gap. Both are retained; no samples are fabricated.','Path lengths come from the estimates, not a tape measure; unobserved motion during gaps is unknown.','This trial uses vendor relative pose to reseed at acquisition gaps; no global loop closure. Small endpoint error does not prove the trajectory is accurate everywhere.','Vendor odometry is a comparison estimate sharing body IMU information, not ground truth.','Zero acquisition-lag correction remains uncalibrated; replay success is not proof of exact LiDAR/IMU acquisition synchronization.','Raw-cloud registration is a conditional static-scene geometric check, not independent surveyed truth.']};(out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(out/'comparison.npz',source_time=t,lio_initial_heading_position=lp,vendor_initial_heading_position=rp)
fig,axs=plt.subplots(2,2,figsize=(13.5,10),layout='constrained');fig.suptitle('Go2 lab circuit: gap recovery trial vs vendor odometry',fontsize=15)
# Break path lines at missing acquisition intervals rather than drawing unobserved tracks.
def break_gaps(points):
 xx=points.copy();xx[np.r_[False,~contig]]=np.nan;return xx
lplot,rplot=break_gaps(lp),break_gaps(rp);axs[0,0].plot(lplot[:,0],lplot[:,1],label='Point-LIO');axs[0,0].plot(rplot[:,0],rplot[:,1],'--',label='Vendor');axs[0,0].scatter(0,0,c='green',label='Start');axs[0,0].scatter(*lp[-1,:2],c='red',label='Point-LIO end');axs[0,0].set(xlabel='Initial heading X (m)',ylabel='Y (m)',title='Full route; lines broken at recording gaps');axs[0,0].set_aspect('equal',adjustable='datalim');axs[0,0].legend(fontsize=9)
near=np.linalg.norm(lp[:,:2],axis=1)<2;initial_mask=(t>=2.6)&(t<=3.6);end_mask=(t>=210)&(t<=216);axs[0,1].scatter(lp[initial_mask,0]*100,lp[initial_mask,1]*100,s=9,label='Point-LIO start');axs[0,1].scatter(lp[end_mask,0]*100,lp[end_mask,1]*100,s=9,label='Point-LIO finish');axs[0,1].scatter(rp[end_mask,0]*100,rp[end_mask,1]*100,s=9,label='Vendor finish');axs[0,1].scatter(0,0,c='black',marker='+');axs[0,1].set(xlabel='X (cm)',ylabel='Y (cm)',title='Quiet start/end holds after one initial alignment');axs[0,1].set_aspect('equal',adjustable='datalim');axs[0,1].legend(fontsize=9)
axs[1,0].plot(t,lplot[:,2]*100,label='Point-LIO');axs[1,0].plot(t,rplot[:,2]*100,'--',label='Vendor');axs[1,0].set(xlabel='Source time (s)',ylabel='Height relative to start (cm)',title='Height estimate, no extra levelling');axs[1,0].legend(fontsize=9)
lyaw=np.rad2deg(np.unwrap(lr.as_euler('xyz')[:,2])-ly);vyaw=np.rad2deg(np.unwrap(vr.as_euler('xyz')[:,2])-vy);lyaw-=360*round(np.median(lyaw[initial_mask])/360);vyaw-=360*round(np.median(vyaw[initial_mask])/360);lyaw[np.r_[False,~contig]]=np.nan;vyaw[np.r_[False,~contig]]=np.nan;axs[1,1].plot(t,lyaw,label='Point-LIO');axs[1,1].plot(t,vyaw,'--',label='Vendor');axs[1,1].set(xlabel='Source time (s)',ylabel='Unwrapped heading change (degrees)',title='Turns around the lab');axs[1,1].legend(fontsize=9)
for ax in axs.ravel():ax.grid(alpha=.25)
for ax in axs[1]:
 for gap in report['source_gaps']['raw']:ax.axvspan(gap['before_s'],gap['after_s'],color='red',alpha=.15)
fig.savefig(out/'lab_comparison.png',dpi=165);fig.savefig(out/'lab_comparison.pdf');plt.close(fig);print(json.dumps({'returns':returns,'travel':report['estimated_travel'],'health':health,'floor':floor,'registration_disagreement':geometric},indent=2))
