"""Offline source-time gyro response comparison with fixed reviewed axes."""
from pathlib import Path
import json
import numpy as np
from scipy.signal import butter,sosfiltfilt
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/home/jain/Doggy');DEST=Path('/home/jain/Doggy/reports/point_lio_steps/step_04_timing/final_timing_check/motion_20261016T203925+1100');d=np.load(DEST/'decoded_samples.npz')
ref=int(d['body_stamp_ns'][0]);bt=(d['body_stamp_ns']-ref)/1e9;lt=(d['lidar_imu_stamp_ns']-ref)/1e9
B=d['body_values'][:,3:6];R=np.array(json.loads((ROOT/'config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics']['mapping.extrinsic_R']).reshape(3,3);L=d['lidar_imu_values'][:,3:6]@R.T
angles=Rotation.from_quat(d['body_values'][:,[7,8,9,6]]).as_euler('xyz',degrees=True)
# All absolute source-time differences are computed as integers before conversion.
step=.002;grid=np.arange(max(bt[0],lt[0])+.5,min(bt[-1],lt[-1])-.5,step)
b=np.column_stack([np.interp(grid,bt,B[:,j]) for j in range(3)]);l=np.column_stack([np.interp(grid,lt,L[:,j]) for j in range(3)])
lag_grid=np.arange(-.2,.20001,.0005)
phases=[['pitch_1', 40.632000000000005, 69.904, 1], ['roll_3', 80.616, 103.964, 0], ['pitch_2', 114.56400000000001, 148.71200000000002, 1], ['roll_5', 155.268, 182.60000000000002, 0]]
results=[];curves={};filtered={}
for fc in (3.,5.,10.,20.):
 sos=butter(3,fc,fs=1/step,output='sos');bf=sosfiltfilt(sos,b,axis=0);lf=sosfiltfilt(sos,l,axis=0);filtered[fc]=(bf,lf)
 for name,start,end,axis in phases:
  mask=(grid>=start)&(grid<=end);times=grid[mask];a=lf[mask,axis];a=a-a.mean();mse=[]
  for lag in lag_grid:
   v=np.interp(times+lag,grid,bf[:,axis]);v-=v.mean();mse.append(np.mean((v-a)**2))
  mse=np.array(mse);k=int(np.argmin(mse));lag=float(lag_grid[k]);v=np.interp(times+lag,grid,bf[:,axis]);v-=v.mean();offset=float(np.mean(np.interp(times+lag,grid,bf[:,axis])-lf[mask,axis]));corr=float(np.corrcoef(v,a)[0,1]);gain=float(np.dot(v,a)/np.dot(a,a));zero=int(np.argmin(abs(lag_grid)));near=lag_grid[mse<=mse[k]*1.05]
  result={'phase':name,'axis':axis,'window_source_s':[start,end],'zero_phase_lowpass_hz':fc,'body_stamp_minus_lidar_stamp_best_s':lag,'rmse_zero_rad_s':float(np.sqrt(mse[zero])),'rmse_best_rad_s':float(np.sqrt(mse[k])),'correlation_best':corr,'body_vs_rotated_lidar_gain_diagnostic':gain,'constant_body_minus_lidar_offset_rad_s':offset,'within_5_percent_minimum_mse_lag_range_s':[float(near.min()),float(near.max())],'lag_search_boundary_hit':k in (0,len(lag_grid)-1)}
  results.append(result);curves[(fc,name)]=mse
# Independent short windows, choose the axis with known phase excitation.
windows=[];bf,lf=filtered[5.]
for name,start,end,axis in phases:
 for left in np.arange(start,end-5,3):
  mask=(grid>=left)&(grid<=left+6);times=grid[mask];a=lf[mask,axis];a=a-a.mean()
  if a.std()<.025:continue
  mse=[]
  for lag in lag_grid:
   v=np.interp(times+lag,grid,bf[:,axis]);v-=v.mean();mse.append(np.mean((v-a)**2))
  k=int(np.argmin(mse));best=float(lag_grid[k]);v=np.interp(times+best,grid,bf[:,axis]);windows.append({'phase':name,'start_s':float(left),'end_s':float(left+6),'lag_s':best,'corr':float(np.corrcoef(v,a)[0,1]),'rmse_rad_s':float(np.sqrt(mse[k]))})
quality={}
for name in ('body','sport','lidar_imu','cloud','odom','clock'):
 t=d[name+'_stamp_ns'];dt=np.diff(t)/1e6;quality[name]={'messages':len(t),'backsteps':int(np.sum(dt<0)),'duplicates':int(np.sum(dt==0)),'interval_ms_quantiles':np.percentile(dt,[0,50,95,99,100]).tolist()}
output={'lag_sign_convention':'delta = body event timestamp minus LiDAR-IMU event timestamp; compare body(t+delta) with R*lidar_gyro(t). Positive delta would be subtracted from body IMU stamps by Point-LIO time_lag_imu_to_lidar.','comparison_type':'Sensor gyro response delay; internal LiDAR gyro->cloud hardware synchronization is not proven by this test. Fixed reviewed raw-cloud rotation used for gyro axes, as supported by prior bag.', 'body_rpy_range_degrees':np.column_stack([angles.min(axis=0),angles.max(axis=0)]).tolist(),'phase_fits':results,'short_window_fits':windows,'quality':quality,'no_configuration_changes':True,'no_mapper_or_bag_replay':True}
(DEST/'gyro_lag_analysis.json').write_text(json.dumps(output,indent=2)+'\n')
fig,axs=plt.subplots(4,1,figsize=(11,11),constrained_layout=True)
axs[0].plot(bt,angles[:,0],label='body roll');axs[0].plot(bt,angles[:,1],label='body pitch');axs[0].set_ylabel('degrees');axs[0].legend();axs[0].set_xlim(35.632000000000005,187.60000000000002)
for ax,(name,start,end,axis) in zip(axs[1:3],phases):
 f=next(r for r in results if r['phase']==name and r['zero_phase_lowpass_hz']==5.);mask=(grid>=start)&(grid<=end);times=grid[mask];a=lf[mask,axis];a-=a.mean();v=np.interp(times,grid,bf[:,axis]);v-=v.mean();vshift=np.interp(times+f['body_stamp_minus_lidar_stamp_best_s'],grid,bf[:,axis]);vshift-=vshift.mean();ax.plot(times,a,label='rotated LiDAR gyro');ax.plot(times,v,label='body gyro at unchanged timestamps',alpha=.7);ax.plot(times,vshift,'--',label=f"body aligned by {f['body_stamp_minus_lidar_stamp_best_s']*1000:.1f} ms",alpha=.8);ax.set_ylabel(name+' rad/s');ax.legend(fontsize=8)
for name,_,_,_ in phases:
 mse=curves[(5.,name)];axs[3].plot(lag_grid*1000,np.sqrt(mse),label=name)
axs[3].set_xlabel('body event timestamp minus LiDAR-IMU event timestamp (ms)');axs[3].set_ylabel('RMSE rad/s');axs[3].legend();fig.savefig(DEST/'gyro_timing_comparison.png',dpi=160);fig.savefig(DEST/'gyro_timing_comparison.pdf');plt.close(fig)
print(json.dumps(output,indent=2))
