"""Offline direct cloud-floor/body-gyro timing diagnostic; no map/replay."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3];folder=ROOT/'reports/point_lio_steps/step_04_timing';info=json.loads((folder/'current_posture_recording.json').read_text());DEST=ROOT/info['directory'];d=np.load(DEST/'decoded_samples.npz');floor=np.load(DEST/'floor_normals.npz');details=json.loads((DEST/'floor_fits.json').read_text())['fits']
ref=int(d['body_stamp_ns'][0]);bt=(d['body_stamp_ns']-ref)/1e9;gyro=d['body_values'][:,3:6];qbody=d['body_values'][:,[7,8,9,6]]
# Use reviewed calibrated body gyro and actual dt. No pose estimate or map is
# published; this short-lived relative integration is only a timing reference.
quats=np.empty((len(bt),4));rot=Rotation.from_quat(qbody[0]);quats[0]=rot.as_quat()
for i in range(1,len(bt)):
 rot=rot*Rotation.from_rotvec((gyro[i-1]+gyro[i])*.5*(bt[i]-bt[i-1]));quats[i]=rot.as_quat()
integrated=Slerp(bt,Rotation.from_quat(quats));fused=Slerp(bt,Rotation.from_quat(qbody))
ft=floor['times'];normals=floor['normals'];lags=np.arange(-.15,.15001,.001)
# Reject only objectively noisy/poorly spread floor fits, without using lag.
quality=np.array([v['inliers']>=40 and v['plane_rms_m']<.012 and v['second_spatial_std_m']>.08 for v in details]);results=[];curves={}
phases=[('pitch',90.,126.),('roll',129.,165.)]
def cost(rotator,t,n):
 world=rotator(t).apply(n);center=world.mean(axis=0);center/=np.linalg.norm(center);angles=np.arccos(np.clip(world@center,-1,1));return float(np.mean(np.minimum(angles,np.deg2rad(1.5))**2)),angles
for reference,rotator in [('integrated_calibrated_body_gyro',integrated),('fused_body_orientation_diagnostic',fused)]:
 for phase,left,right in phases:
  mask=(ft>=left)&(ft<=right)&quality;t=ft[mask];n=normals[mask]
  if len(t)<40:continue
  losses=np.array([cost(rotator,t+lag,n)[0] for lag in lags]);k=int(np.argmin(losses));best=float(lags[k]);z=int(np.argmin(abs(lags)));near=lags[losses<=losses[k]*1.05];curves[(reference,phase)]=losses
  results.append({'reference':reference,'phase':phase,'floor_scans':len(t),'body_stamp_minus_cloud_point_stamp_s':best,'rms_world_floor_angle_at_zero_deg':float(np.rad2deg(np.sqrt(losses[z]))),'rms_world_floor_angle_at_best_deg':float(np.rad2deg(np.sqrt(losses[k]))),'within_5_percent_minimum_mse_lag_range_s':[float(near.min()),float(near.max())],'boundary_hit':k in (0,len(lags)-1)})
window_results=[]
for phase,left,right in phases:
 for start in np.arange(left,right-9,5):
  mask=(ft>=start)&(ft<=start+10)&quality;t=ft[mask];n=normals[mask]
  if len(t)<40:continue
  losses=np.array([cost(integrated,t+lag,n)[0] for lag in lags]);k=int(np.argmin(losses));window_results.append({'phase':phase,'start_s':float(start),'end_s':float(start+10),'scans':len(t),'lag_s':float(lags[k]),'rms_best_deg':float(np.rad2deg(np.sqrt(losses[k]))),'boundary_hit':k in (0,len(lags)-1)})
result={'method':'Fit an unconstrained floor plane to each raw scan in body coordinates; assign its mean inlier point time. Integrate calibrated body gyro using real source intervals. Search delta so integrated_R(t_point+delta)*normal_cloud is most constant within each phase, allowing an unknown fixed world-floor normal. Minimize squared angular residual capped at 1.5 degrees for robustness. No orientation/rate gain or sensor extrinsic was fitted. Fused body orientation is a separate diagnostic only.','sign':'Positive delta means subtract delta from body IMU header stamps for Point-LIO.','fits':results,'short_window_fits':window_results,'quality_floor_scans':int(quality.sum()),'all_floor_scans':len(ft),'limits':['Planar stationary floor, rigid mounting and the approved topic-derived cloud/body conversion are assumed.','Each scan plane represents an average inlier acquisition time; rapid motion causes intra-scan distortion.','Residual gyro bias/drift, finite cloud density and floor-fit noise limit millisecond precision.','Objective-width intervals are sensitivity ranges, not confidence intervals.'],'configuration_changed':False}
(DEST/'cloud_lag_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
fig,axs=plt.subplots(2,1,figsize=(10,7),constrained_layout=True)
for reference in ('integrated_calibrated_body_gyro','fused_body_orientation_diagnostic'):
 for phase,_,_ in phases:
  if (reference,phase) in curves:axs[0].plot(lags*1000,np.rad2deg(np.sqrt(curves[(reference,phase)])),label=phase+' '+reference.replace('_',' '))
axs[0].set_xlabel('body timestamp minus cloud point timestamp (ms)');axs[0].set_ylabel('world floor angle RMS (degrees)');axs[0].legend(fontsize=7)
normal_angles=np.rad2deg(np.column_stack([np.arctan2(normals[:,1],normals[:,2]),np.arctan2(-normals[:,0],np.hypot(normals[:,1],normals[:,2]))]));axs[1].plot(ft,normal_angles[:,0],'.',ms=2,label='floor normal roll');axs[1].plot(ft,normal_angles[:,1],'.',ms=2,label='floor normal pitch');axs[1].set_ylabel('degrees');axs[1].set_xlabel('source seconds since first recorded body IMU');axs[1].legend();fig.savefig(DEST/'cloud_timing_check.png',dpi=160);fig.savefig(DEST/'cloud_timing_check.pdf');plt.close(fig)
print(json.dumps(result,indent=2))
