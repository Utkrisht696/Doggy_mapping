"""Hold-to-hold comparison: nominal route and vendor are not surveyed ground truth."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
run=Path(__file__).resolve().parent;source=np.load(run/'source_decoded.npz');a=np.load(run/'outputs.npz');out=run/'analysis';out.mkdir(exist_ok=True)
origin=int(source['raw_stamp_ns'][0]);t=(a['odom_stamp_ns']-origin)/1e9;p=a['odom_values'][:,:3];lr=Rotation.from_quat(a['odom_values'][:,3:7]);vt=(source['vendor_stamp_ns']-origin)/1e9;v=source['vendor_values'];vr=Slerp(vt,Rotation.from_quat(v[:,3:7]))(t);vp=np.column_stack([np.interp(t,vt,v[:,j]) for j in range(3)])+vr.apply([-.02557,0,.04232])
windows={'start':[3.,7.],'far_3m':[27.5,30.5],'return_3m':[46.5,50.],'far_5m':[73.,76.],'return_5m':[106.,109.5]}
holds={}
for name,(start,end) in windows.items():
 mask=(t>=start)&(t<=end);assert mask.sum()>10
 it=(source['imu_stamp_ns']-origin)/1e9;im=(it>=start)&(it<=end)
 holds[name]={'time_window_s':[start,end],'point_lio_samples':int(mask.sum()),'lio_median_xyz_m':np.median(p[mask],axis=0).tolist(),'vendor_median_xyz_m':np.median(vp[mask],axis=0).tolist(),'lio_mean_quaternion_xyzw':lr[mask].mean().as_quat().tolist(),'vendor_mean_quaternion_xyzw':vr[mask].mean().as_quat().tolist(),'body_gyro_rms_rad_s':float(np.sqrt(np.mean(np.sum(source['imu_values'][im,3:6]**2,axis=1))))}
 for label,points in [('lio',p),('vendor',vp)]:
  pts=points[mask];med=np.median(pts,axis=0)
  holds[name][label+'_position_p95_distance_to_median_m']=float(np.percentile(np.linalg.norm(pts-med,axis=1),95));holds[name][label+'_hold_start_end_translation_m']=float(np.linalg.norm(np.median(pts[-8:],axis=0)-np.median(pts[:8],axis=0)))
checks=[]
for target,start,far,end in [(3.,'start','far_3m','return_3m'),(5.,'return_3m','far_5m','return_5m')]:
 row={'nominal_distance_m':target,'start_hold':start,'far_hold':far,'return_hold':end}
 for label,key in [('point_lio','lio'),('vendor','vendor')]:
  ps=np.array(holds[start][key+'_median_xyz_m']);pf=np.array(holds[far][key+'_median_xyz_m']);pe=np.array(holds[end][key+'_median_xyz_m']);delta=pf-ps;closed=pe-ps;rs=Rotation.from_quat(holds[start][key+'_mean_quaternion_xyzw']);re=Rotation.from_quat(holds[end][key+'_mean_quaternion_xyzw']);distance=np.linalg.norm(delta[:2]);axis=rs.apply([1.,0,0]);axis[2]=0;axis/=np.linalg.norm(axis)
  row[label]={'outbound_horizontal_endpoint_distance_m':float(distance),'inbound_horizontal_endpoint_distance_m':float(np.linalg.norm((pe-pf)[:2])),'outbound_projected_initial_heading_m':float(delta@axis),'outbound_vertical_change_m':float(delta[2]),'nominal_distance_difference_m':float(distance-target),'nominal_distance_difference_percent':float(100*(distance-target)/target),'return_horizontal_offset_m':float(np.linalg.norm(closed[:2])),'return_vertical_offset_m':float(closed[2]),'return_3d_offset_m':float(np.linalg.norm(closed)),'return_orientation_change_deg':float(np.rad2deg((rs.inv()*re).magnitude())),'outbound_displacement_xyz_m':delta.tolist(),'return_displacement_xyz_m':closed.tolist()}
 checks.append(row)
overall={}
for label,key in [('point_lio','lio'),('vendor','vendor')]:
 delta=np.array(holds['return_5m'][key+'_median_xyz_m'])-holds['start'][key+'_median_xyz_m'];rot0=Rotation.from_quat(holds['start'][key+'_mean_quaternion_xyzw']);rot1=Rotation.from_quat(holds['return_5m'][key+'_mean_quaternion_xyzw']);overall[label]={'final_horizontal_offset_from_original_start_m':float(np.linalg.norm(delta[:2])),'final_vertical_offset_from_original_start_m':float(delta[2]),'final_3d_offset_from_original_start_m':float(np.linalg.norm(delta)),'final_orientation_change_deg':float(np.rad2deg((rot0.inv()*rot1).magnitude()))}
# Compare paths using one initial yaw/translation; preserve gravity axes, no scale fitting.
yaw0=lr[(t>=3)&(t<=7)].mean().as_euler('xyz')[2];vyaw0=vr[(t>=3)&(t<=7)].mean().as_euler('xyz')[2];lp=Rotation.from_euler('z',-yaw0).apply(p-np.array(holds['start']['lio_median_xyz_m']));rp=Rotation.from_euler('z',-vyaw0).apply(vp-np.array(holds['start']['vendor_median_xyz_m']))
# Floor repeatability uses map XY cells directly, no registration or levelling.
ct=(a['cloud_stamp_ns']-origin)/1e9;points=a['cloud_points'];offsets=a['cloud_offsets'];initial=np.array(holds['start']['lio_median_xyz_m'])
def cells(name):
 start,end=windows[name];ix=np.flatnonzero((ct>=start)&(ct<=end));pts=np.concatenate([points[offsets[k]:offsets[k+1]] for k in ix]).astype(float);pts=pts[(np.linalg.norm(pts[:,:2]-initial[:2],axis=1)<3)&(pts[:,2]>initial[2]-.9)&(pts[:,2]<initial[2]-.15)];keys=np.floor(pts[:,:2]/.15).astype('int64');ids,inv,cnt=np.unique(keys,axis=0,return_inverse=True,return_counts=True);order=np.argsort(inv);z=pts[order,2];ends=np.r_[0,np.cumsum(cnt)];return {tuple(k):float(np.median(z[ends[i]:ends[i+1]])) for i,k in enumerate(ids) if cnt[i]>=5}
floor0=cells('start');floorchecks=[]
for name in ['return_3m','return_5m']:
 f=cells(name);common=sorted(f.keys()&floor0.keys());diff=np.array([f[k]-floor0[k] for k in common]);row={'hold':name,'common_15cm_xy_cells':len(common)}
 if len(diff):row.update(median_signed_floor_height_difference_m=float(np.median(diff)),median_absolute_floor_height_difference_m=float(np.median(abs(diff))),p90_absolute_floor_height_difference_m=float(np.percentile(abs(diff),90)))
 floorchecks.append(row)
def stats(vals):return dict(zip(['median','p95','p99','max'],map(float,np.percentile(vals,[50,95,99,100]))))
dt=np.diff(t);steps=np.linalg.norm(np.diff(p,axis=0),axis=1);angles=np.rad2deg((lr[:-1].inv()*lr[1:]).magnitude());yaw=np.rad2deg(np.unwrap(lr.as_euler('xyz')[:,2])-yaw0);imu_st=source['imu_stamp_ns'];largest_gap=np.argsort(np.diff(imu_st))[-10:][::-1]
gaps=[]
for ix in largest_gap:
 moment=(int(imu_st[ix])+int(imu_st[ix+1])-2*origin)/2e9;near=np.abs(t[1:]-moment)<.15;gaps.append({'time_s':float(moment),'imu_gap_s':float((imu_st[ix+1]-imu_st[ix])/1e9),'nearby_max_lio_step_m':float(steps[near].max()) if near.any() else None})
health={'lio_poses':len(t),'first_pose_s':float(t[0]),'last_pose_s':float(t[-1]),'nonfinite_pose_values':int((~np.isfinite(a['odom_values'])).sum()),'nonincreasing_pose_stamps':int((np.diff(a['odom_stamp_ns'])<=0).sum()),'lio_dt_s':stats(dt),'average_pose_frequency_hz':float((len(t)-1)/(t[-1]-t[0])),'lio_step_translation_m':stats(steps),'lio_step_rotation_deg':stats(angles),'steps_over_0_15_m':int((steps>.15).sum()),'rotation_steps_over_5_deg':int((angles>5).sum()),'tf_pose_stamps_exact':bool(np.array_equal(a['odom_stamp_ns'],a['map_odom_stamp_ns'])),'tf_missing_pose_stamp_count':len(np.setdiff1d(a['odom_stamp_ns'],a['map_odom_stamp_ns'])),'tf_extra_stamp_count':len(np.setdiff1d(a['map_odom_stamp_ns'],a['odom_stamp_ns'])),'lio_yaw_range_relative_to_start_deg':[float(yaw.min()),float(yaw.max())],'largest_imu_gaps':gaps}
report={'source_bag':'/home/jain/go2_bags/classic','seconds_reference':'First raw cloud source header timestamp','user_route':'3 m forward, wait, 3 m back, wait, 5 m forward, wait, 5 m back, wait','measurement_limitations':['Reported 3 m and 5 m are nominal references; physical measurement method and uncertainty were not supplied.','Curved walking paths: reported hold-to-hold horizontal displacement, not integrated travelled path length.','No independent endpoint survey. Return offsets include real placement differences and estimator error; they are not pure drift measurements.','Vendor odometry shares body IMU information and is not ground truth. Its poses include the developer body-to-IMU lever arm.','Repeated floor-cell agreement measures internal consistency, not surveyed absolute vertical accuracy.','Zero LiDAR/IMU acquisition-lag correction remains uncalibrated. Replay success does not prove exact sensor synchronization.'],'holds':holds,'distance_and_return_checks':checks,'overall_return_check':overall,'revisited_start_floor_checks':floorchecks,'floor_method':'Map XY grid 15 cm, >=5 points per cell, within 3 m of initial IMU and .15-.9 m below it; no ICP/relevelling. Same XY assumes adequate horizontal registration.','health':health,'replay_result':json.loads((run/'result.json').read_text()),'application_settings_changed':False,'diagnostic_startup_override':{'init_map_size':4000}}
(out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(out/'comparison.npz',time=t,lio_position_initial_heading=lp,vendor_position_initial_heading=rp,lio_yaw_deg=yaw)
fig,axs=plt.subplots(2,2,figsize=(13.5,9),layout='constrained');fig.suptitle('Classic bag: 3 m and 5 m out-and-back — Point-LIO vs vendor',fontsize=14)
axs[0,0].plot(t,lp[:,0],label='Point-LIO');axs[0,0].plot(t,rp[:,0],'--',label='Vendor');axs[0,0].axhline(3,color='grey',ls=':',label='Nominal 3 m');axs[0,0].axhline(5,color='black',ls=':',label='Nominal 5 m');axs[0,0].set(xlabel='Source time (s)',ylabel='Forward displacement (m)',title='Distances from the original start');axs[0,0].legend(fontsize=9)
axs[0,1].plot(lp[:,0],lp[:,1],label='Point-LIO');axs[0,1].plot(rp[:,0],rp[:,1],'--',label='Vendor');axs[0,1].scatter(*lp[0,:2],c='green',label='Start');axs[0,1].scatter(*lp[-1,:2],c='red',label='Point-LIO end');axs[0,1].set(xlabel='Initial heading X (m)',ylabel='Lateral Y (m)',title='Each estimate centred at start; no scale fit');axs[0,1].set_aspect('equal',adjustable='datalim');axs[0,1].legend(fontsize=9)
axs[1,0].plot(t,lp[:,2]*100,label='Point-LIO');axs[1,0].plot(t,rp[:,2]*100,'--',label='Vendor');axs[1,0].set(xlabel='Source time (s)',ylabel='Height change (cm)',title='IMU height relative to start');axs[1,0].legend(fontsize=9)
labels=['3 m return','5 m return','Final vs original'];lvals=[checks[0]['point_lio']['return_horizontal_offset_m'],checks[1]['point_lio']['return_horizontal_offset_m'],overall['point_lio']['final_horizontal_offset_from_original_start_m']];vvals=[checks[0]['vendor']['return_horizontal_offset_m'],checks[1]['vendor']['return_horizontal_offset_m'],overall['vendor']['final_horizontal_offset_from_original_start_m']];x=np.arange(3);axs[1,1].bar(x-.18,np.array(lvals)*100,.35,label='Point-LIO');axs[1,1].bar(x+.18,np.array(vvals)*100,.35,label='Vendor');axs[1,1].set_xticks(x,labels);axs[1,1].set(ylabel='Horizontal return offset (cm)',title='Includes physical endpoint placement differences');axs[1,1].legend(fontsize=9)
for ax in axs.ravel():ax.grid(alpha=.25)
for ax in [axs[0,0],axs[1,0]]:
 for name,(start,end) in windows.items():ax.axvspan(start,end,color='green',alpha=.08)
fig.savefig(out/'comparison.png',dpi=170);fig.savefig(out/'comparison.pdf');plt.close(fig)
print(json.dumps({'checks':checks,'overall':overall,'floor':floorchecks,'health':health},indent=2))
