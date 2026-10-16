"""Read-only drift screening; vendor estimates are a reference, never truth."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scipy.ndimage import gaussian_filter1d,binary_erosion
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
base=Path(__file__).resolve().parent
mount=np.array([-.02557,0,.04232])
def finite_number(x):return float(x)
def groups(mask):
 ind=np.flatnonzero(mask)
 return [] if not len(ind) else np.split(ind,np.flatnonzero(np.diff(ind)>1)+1)
def quat_angle(r):return np.rad2deg(r.magnitude())
for label in ['analysis']:
 dest=base/label;a=np.load(dest/'decoded.npz');audit=json.loads((dest/'bag_audit.json').read_text());origin=int(a['lio_stamp_ns'][0]);t=(a['lio_stamp_ns']-origin)/1e9;pose=a['lio_values'];p=pose[:,:3];lr=Rotation.from_quat(pose[:,3:7])
 def unique_stream(k):
  st,ix=np.unique(a[k+'_stamp_ns'],return_index=True);return (st-origin)/1e9,a[k+'_values'][ix]
 vt,v=unique_stream('vendor');it,imu=unique_stream('imu');st,sport=unique_stream('sport')
 vr=Slerp(vt,Rotation.from_quat(v[:,3:7]))(t);vp=np.column_stack([np.interp(t,vt,v[:,j]) for j in range(3)])+vr.apply(mount)
 align=lr[0]*vr[0].inv();offset=p[0]-align.apply(vp[0]);ref=align.apply(vp)+offset;rr=align*vr
 diff=p-ref;ang=quat_angle(rr.inv()*lr);speed_lio=np.linalg.norm(np.diff(p,axis=0),axis=1)/np.diff(t);angvel=quat_angle(lr[:-1].inv()*lr[1:])/np.diff(t)
 grid=np.arange(max(0,vt[0],it[0],st[0]),min(t[-1],vt[-1],it[-1],st[-1]),.02)
 vg=np.column_stack([np.interp(grid,vt,v[:,j]) for j in range(3)]);vgs=gaussian_filter1d(vg,12.5,axis=0);vendor_speed=np.linalg.norm(np.gradient(vgs,.02,axis=0),axis=1)
 sport_speed=np.linalg.norm(np.column_stack([np.interp(grid,st,sport[:,j]) for j in [3,4,5]]),axis=1)
 gyro_norm=np.linalg.norm(imu[:,3:6],axis=1);gyro_rms=np.sqrt(gaussian_filter1d(np.interp(grid,it,gyro_norm**2),12.5))
 quiet=(vendor_speed<.02)&(sport_speed<.03)&(gyro_rms<.04);quiet=binary_erosion(quiet,iterations=25,border_value=0)
 segments=[]
 for g in groups(quiet):
  start,end=grid[g[0]],grid[g[-1]]
  if end-start<3:continue
  m=(t>=start)&(t<=end);ix=np.flatnonzero(m)
  if len(ix)<3:continue
  ps=p[m];r=lr[m];rf=ref[m];res=diff[m]-diff[m][0];refdeg=quat_angle(rr[m][0].inv()*rr[m]);rdeg=quat_angle(r[0].inv()*r)
  segments.append({'start_s':float(start),'end_s':float(end),'duration_s':float(end-start),'lio_net_translation_m':float(np.linalg.norm(ps[-1]-ps[0])),'lio_max_excursion_m':float(np.linalg.norm(ps-ps[0],axis=1).max()),'lio_max_rotation_deg':float(rdeg.max()),'reference_max_translation_m':float(np.linalg.norm(rf-rf[0],axis=1).max()),'reference_max_rotation_deg':float(refdeg.max()),'relative_translation_error_change_max_m':float(np.linalg.norm(res,axis=1).max()),'body_gyro_rms_max_rad_s':float(gyro_rms[g].max()),'classification':'Candidate stationary from vendor odometry, SportModeState velocity and calibrated body gyro; not externally verified'})
 def stats(v):return dict(zip(['median','p95','p99','max'],map(float,np.percentile(v,[50,95,99,100]))))
 metrics={'source_run':audit['source_run'],'seconds_reference':'Seconds after first recorded Point-LIO pose','duration_s':float(t[-1]),'lio_poses':len(t),'lio_nonfinite_values':int((~np.isfinite(pose)).sum()),'lio_dt_s':stats(np.diff(t)),'lio_step_translation_m':stats(np.linalg.norm(np.diff(p,axis=0),axis=1)),'lio_step_angle_deg':stats(quat_angle(lr[:-1].inv()*lr[1:])),'lio_speed_m_s':stats(speed_lio),'lio_angular_speed_deg_s':stats(angvel),'lio_raw_path_length_m':float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),'lio_height_change_end_m':float(p[-1,2]-p[0,2]),'lio_height_extent_m':float(np.ptp(p[:,2])),'reference_height_change_end_m':float(ref[-1,2]-ref[0,2]),'single_initial_alignment_reference_comparison':{'reference':'Vendor /utlidar/robot_odom with developer body->IMU lever arm; shares body IMU and is not ground truth','position_difference_m':stats(np.linalg.norm(diff,axis=1)),'orientation_difference_deg':stats(ang),'endpoint_position_difference_m':float(np.linalg.norm(diff[-1])),'endpoint_difference_xyz_m':diff[-1].tolist(),'endpoint_orientation_difference_deg':float(ang[-1])},'candidate_stationary_segments':segments,'position_step_over_0_15_m_count':int((np.linalg.norm(np.diff(p,axis=0),axis=1)>.15).sum()),'rotation_step_over_5_deg_count':int((quat_angle(lr[:-1].inv()*lr[1:])>5).sum()),'tf_pose_stamp_sequence_exact_match':bool(np.array_equal(a['lio_stamp_ns'],a['tfmap_stamp_ns'])),'tf_conflicting_parents':{k:v for k,v in audit['tf_parents'].items() if len(v)>1}}
 # Repeated timestamps are inspected separately from gaps; no invented source rates.
 for k in ['lio','vendor','imu','sport','raw']:
  s=a[k+'_stamp_ns'];dt=np.diff(s)/1e9;positive=dt[dt>0];metrics[k+'_stamps']={'samples':len(s),'duplicates':int((dt==0).sum()),'backward':int((dt<0).sum()),'positive_dt_s':stats(positive),'gap_over_3_median_dt_count':int((dt>3*np.median(positive)).sum())}
 body=a['imu_stamp_ns'];source=np.unique(a['sport_stamp_ns']);source=source[(source>=body[0])&(source<=body[-1])];metrics['body_imu_missing_unique_sport_stamp_count']=int(len(np.setdiff1d(source,body)))
 raw=a['raw_values'];rs=a['raw_stamp_ns'];begin=rs+np.rint(raw[:,2]*1e9).astype(np.int64);end=rs+np.rint(raw[:,3]*1e9).astype(np.int64);inside=(begin>=body[0])&(end<=body[-1]);metrics['raw_scan_imu_outer_bracketing']={'scans_within_imu_recording':int(inside.sum()),'scans_outside_imu_recording':int((~inside).sum()),'point_time_span_s':stats(raw[:,3]-raw[:,2])}
 # Relative pose disagreements at fixed source-time intervals, no refitting.
 metrics['relative_pose_reference_comparison']={}
 for sec in [1,5]:
  good=t+sec<=t[-1];ts=t[good];pl=np.column_stack([np.interp(ts+sec,t,p[:,j]) for j in range(3)]);pr=np.column_stack([np.interp(ts+sec,t,ref[:,j]) for j in range(3)]);lrf=Slerp(t,lr)(ts+sec);rrf=Slerp(t,rr)(ts+sec)
  dl=lr[good].inv().apply(pl-p[good]);dr=rr[good].inv().apply(pr-ref[good]);de=np.linalg.norm(dl-dr,axis=1);are=quat_angle((rr[good].inv()*rrf).inv()*(lr[good].inv()*lrf))
  metrics['relative_pose_reference_comparison'][str(sec)+'s']={'translation_difference_m':stats(de),'rotation_difference_deg':stats(are),'largest_translation_difference_time_s':float(ts[np.argmax(de)])}
 bridge=audit['bridge_status_samples'];metrics['bridge_last_status']=bridge[-1] if bridge else None
 # Largest motion step and relative discrepancy times can be inspected in the replay.
 steps=np.linalg.norm(np.diff(p,axis=0),axis=1);ix=np.argsort(steps)[-8:][::-1];metrics['largest_pose_steps']=[{'time_s':float(t[i+1]),'distance_m':float(steps[i]),'dt_s':float(t[i+1]-t[i]),'angle_deg':float(quat_angle(lr[i].inv()*lr[i+1]))} for i in ix]
 np.savez_compressed(dest/'comparison.npz',time=t,lio_position=p,reference_position=ref,lio_quaternion=lr.as_quat(),reference_quaternion=rr.as_quat(),position_difference=diff,orientation_difference_deg=ang,quiet_grid=grid,quiet_mask=quiet,vendor_speed=vendor_speed,gyro_rms=gyro_rms)
 (dest/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
 fig,axs=plt.subplots(3,2,figsize=(14,12),layout='constrained');fig.suptitle('3 m and 5 m out-and-back test'+' — recorded drift and continuity checks',fontsize=15)
 axs[0,0].plot(p[:,0],p[:,1],label='Point-LIO');axs[0,0].plot(ref[:,0],ref[:,1],'--',label='Vendor estimate, aligned only at start');axs[0,0].scatter(*p[0,:2],c='green',label='Start');axs[0,0].scatter(*p[-1,:2],c='red',label='End');axs[0,0].axis('equal');axs[0,0].set(xlabel='map X (m)',ylabel='map Y (m)',title='Estimated paths; neither is ground truth');axs[0,0].legend(fontsize=8)
 axs[0,1].plot(grid,vendor_speed,label='Vendor position speed');axs[0,1].plot(grid,sport_speed,label='Sport state speed',alpha=.6);axs[0,1].set(ylabel='Speed (m/s)',title='Motion context');axs[0,1].legend(fontsize=8)
 axs[1,0].plot(t,np.linalg.norm(diff,axis=1),label='3D position disagreement');axs[1,0].set(ylabel='Difference (m)',title='Difference after ONE initial rigid alignment')
 axs[1,1].plot(t,ang,label='Orientation disagreement');axs[1,1].set(ylabel='Difference (degrees)',title='Point-LIO vs vendor orientation')
 axs[2,0].plot(t,p[:,2]-p[0,2],label='Point-LIO');axs[2,0].plot(t,ref[:,2]-ref[0,2],'--',label='Vendor');axs[2,0].set(ylabel='Height change (m)',title='Vertical estimate');axs[2,0].legend(fontsize=8)
 axs[2,1].plot(t[1:],steps,label='Translation per output');axs[2,1].set(ylabel='Step (m)',title='Output continuity (~65 ms per pose)')
 for ax in axs.ravel():ax.grid(alpha=.25)
 for ax in [axs[0,1],*axs[1],*axs[2]]:
  ax.set_xlabel('Seconds after first Point-LIO pose')
  for s in segments:ax.axvspan(s['start_s'],s['end_s'],color='green',alpha=.10)
 fig.savefig(dest/'drift_overview.png',dpi=160);fig.savefig(dest/'drift_overview.pdf');plt.close(fig)
 print(label,json.dumps({'reference_comparison':metrics['single_initial_alignment_reference_comparison'],'stationary':segments,'largest_steps':metrics['largest_pose_steps'][:2],'imu_missing':metrics['body_imu_missing_unique_sport_stamp_count']}),flush=True)
