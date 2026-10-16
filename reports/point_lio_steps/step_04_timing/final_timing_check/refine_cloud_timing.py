"""Offline per-point rotational floor diagnostic; no map, ROS or parameter edits."""
from pathlib import Path
import argparse,json,time
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();ROOT=Path('/home/jain/Doggy');DEST=Path(a.directory).resolve();d=np.load(DEST/'decoded_samples.npz');floor=np.load(DEST/'floor_normals.npz');fits=json.loads((DEST/'floor_fits.json').read_text())['fits'];phases=json.loads((DEST/'motion_windows.json').read_text())['phases'];ref=int(d['body_stamp_ns'][0]);bt=(d['body_stamp_ns']-ref)/1e9;ct=(d['cloud_stamp_ns']-ref)/1e9;gyro=d['body_values'][:,3:6];q=d['body_values'][:,[7,8,9,6]];rot=Rotation.from_quat(q[0]);quats=np.empty_like(q);quats[0]=rot.as_quat()
for i in range(1,len(bt)):
 rot=rot*Rotation.from_rotvec((gyro[i-1]+gyro[i])*.5*(bt[i]-bt[i-1]));quats[i]=rot.as_quat()
rotators={'integrated_calibrated_gyro':Slerp(bt,Rotation.from_quat(quats)),'fused_orientation_diagnostic':Slerp(bt,Rotation.from_quat(q))}
geo=json.loads((ROOT/'config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics'];R=np.array(geo['mapping.extrinsic_R']).reshape(3,3);t=np.array(geo['mapping.extrinsic_T']);body_mount=np.array([-.02557,0,.04232]);points=d['cloud_points'];pt=d['cloud_point_times'];offsets=d['cloud_offsets'];rng=np.random.default_rng(2718);datasets=[]
for name,left,right,axis in phases:
 indexes=[j for j,f in enumerate(fits) if left<=f['effective_source_s']<=right and f['inliers']>=100 and f['plane_rms_m']<.012 and f['second_spatial_std_m']>.08][::3];xx=[];tt=[];ids=[]
 for j in indexes:
  f=fits[j];i=f['cloud_index'];li,ri=offsets[i:i+2];x=points[li:ri].astype(float)@R.T+t;body=x+body_mount;n=floor['normals'][j];keep=np.flatnonzero(np.abs(body@n+f['plane_offset_m'])<.025)
  if len(keep)<50:continue
  keep=np.sort(rng.choice(keep,min(120,len(keep)),replace=False));xx.append(x[keep]);tt.append(ct[i]+pt[li:ri][keep]);ids.append(i)
 if len(xx)>=12:datasets.append({'name':name,'left':left,'right':right,'axis':axis,'x':np.concatenate(xx),'time':np.concatenate(tt),'offsets':np.cumsum([0]+[len(x) for x in xx]),'scans':len(xx),'ids':ids})
assert datasets,'No suitable floor scans'
def evaluate(rotator,ds,lag):
 x=rotator(ds['time']+lag).apply(ds['x']);normals=[];rms=[]
 for lo,hi in zip(ds['offsets'][:-1],ds['offsets'][1:]):
  c=x[lo:hi]-x[lo:hi].mean(axis=0);ev,evec=np.linalg.eigh(c.T@c/len(c));n=evec[:,0];n=n if n[2]>=0 else -n;normals.append(n);rms.append(np.sqrt(max(0,ev[0])))
 normals=np.array(normals);center=normals.mean(axis=0);center/=np.linalg.norm(center);angles=np.arccos(np.clip(normals@center,-1,1));h=np.deg2rad(.5);loss=np.where(angles<=h,angles**2,2*h*angles-h*h)
 return float(loss.mean()),float(np.rad2deg(np.sqrt(np.mean(angles**2)))),float(np.median(rms))
results=[];curves=[];start=time.monotonic()
for reference,rotator in rotators.items():
 for ds in datasets:
  cache={}
  def cost(lag):
   lag=round(float(lag),7)
   if lag not in cache:cache[lag]=evaluate(rotator,ds,lag)
   return cache[lag][0]
  coarse=np.arange(-.1,.10001,.01);k=int(np.argmin([cost(v) for v in coarse]));best0=coarse[k];fine=np.arange(max(-.1,best0-.015),min(.1,best0+.015)+.0001,.001);[cost(v) for v in fine];cost(0);lags=np.array(sorted(cache));loss=np.array([cache[v][0] for v in lags]);ix=int(np.argmin(loss));best=lags[ix];near=lags[loss<=loss[ix]*1.05];r={'reference':reference,'phase':ds['name'],'source_window_s':[ds['left'],ds['right']],'floor_scans':ds['scans'],'selected_points':len(ds['x']),'best_body_stamp_minus_cloud_point_stamp_s':float(best),'best_loss':float(loss[ix]),'zero_loss':float(cache[0][0]),'zero_to_best_loss_improvement_percent':float(100*(cache[0][0]-loss[ix])/cache[0][0]),'normal_angle_rms_zero_deg':cache[0][1],'normal_angle_rms_best_deg':cache[best][1],'median_intrascan_plane_rms_zero_m':cache[0][2],'median_intrascan_plane_rms_best_m':cache[best][2],'within_5_percent_minimum_loss_range_s':[float(near.min()),float(near.max())],'boundary_hit':bool(abs(best)>=.0999)};results.append(r);curves.append((reference,ds['name'],lags,loss));print(json.dumps(r),flush=True)
result={'method':'Select fixed floor inlier candidates from unconstrained per-scan planes. Apply the reviewed LiDAR-to-body-IMU extrinsics and integrate calibrated body gyro at actual source dt. For each candidate lag, rotate each individual point at its own acquisition time plus lag; refit world normals separately per scan, with free plane offset, then minimize between-scan normal dispersion using a non-saturating Huber loss. Fused attitude is an independent diagnostic only. No translation/rotation/bias/offset is optimized or published.','lag_sign':'Positive delta would be subtracted from body IMU stamps by Point-LIO only if independently validated.','results':results,'elapsed_s':time.monotonic()-start,'application_changed':False,'mapper_or_replay_started':False,'limits':['This is a floor diagnostic, not a general temporal calibration package.','Within-scan body translation is not compensated; posture motion and pivot/lever arm can bias results.','Only rotational motion about two axes is excited; raw LiDAR internal IMU still has invalid acceleration and is not an absolute reference.','Sparse/noisy points, imperfect physical extrinsics, gyro drift and fixed inlier selection can confound lag.','Loss-width ranges are sensitivity measures, not confidence intervals.','Fresh repeat blocks are considered independently; disagreement prevents applying a correction.']};(DEST/'per_point_cloud_timing.json').write_text(json.dumps(result,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(10,5.5),layout='constrained')
for reference,name,lags,loss in curves:ax.plot(lags*1000,np.rad2deg(np.sqrt(loss)),label=name+' '+reference)
ax.axvline(0,color='black',ls=':',lw=1);ax.set_xlabel('Body timestamp minus cloud point timestamp (ms)');ax.set_ylabel('Robust floor-normal dispersion score (degrees)');ax.set_title('Per-point rotational timing diagnostic; lower is better');ax.grid(alpha=.2);ax.legend(fontsize=7);fig.savefig(DEST/'per_point_cloud_timing.png',dpi=160);fig.savefig(DEST/'per_point_cloud_timing.pdf')
