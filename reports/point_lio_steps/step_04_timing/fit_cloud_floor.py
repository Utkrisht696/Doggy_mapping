"""Offline stationary-floor fits for a direct cloud/body-gyro timing diagnostic."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3];folder=ROOT/'reports/point_lio_steps/step_04_timing';info=json.loads((folder/'current_posture_recording.json').read_text());DEST=ROOT/info['directory'];d=np.load(DEST/'decoded_samples.npz');ref=int(d['body_stamp_ns'][0]);bt=(d['body_stamp_ns']-ref)/1e9;ct=(d['cloud_stamp_ns']-ref)/1e9
geo=json.loads((ROOT/'config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics'];R=np.array(geo['mapping.extrinsic_R']).reshape(3,3);t=np.array(geo['mapping.extrinsic_T'])+np.array([-.02557,0,.04232]);bodyq=d['body_values'][:,[7,8,9,6]]
points=d['cloud_points'];times=d['cloud_point_times'];offset=d['cloud_offsets'];rng=np.random.default_rng(1842);results=[];normals=[];qbody=Rotation.from_quat(bodyq)
for i in np.flatnonzero((ct>78)&(ct<172)):
 x=points[offset[i]:offset[i+1]].astype(float)@R.T+t;pt=times[offset[i]:offset[i+1]]
 j=np.clip(np.searchsorted(bt,ct[i]+float(np.median(pt))),0,len(bt)-1);prior=qbody[j].inv().apply([0,0,1]);height=x@prior
 candidate=(height>-.7)&(height<-.12)&(np.linalg.norm(x,axis=1)<8);a=x[candidate];ta=pt[candidate]
 if len(a)<30:continue
 best=None;best_count=0
 for _ in range(70):
  sample=a[rng.choice(len(a),3,replace=False)];n=np.cross(sample[1]-sample[0],sample[2]-sample[0]);norm=np.linalg.norm(n)
  if norm<1e-7:continue
  n/=norm
  if n@prior<0:n=-n
  if n@prior<.94:continue
  bias=-sample[0]@n;mask=np.abs(a@n+bias)<.015;count=int(mask.sum())
  if count>best_count:best=mask;best_count=count
 if best_count<30:continue
 for _ in range(3):
  inliers=a[best];center=inliers.mean(axis=0);_,sing,V=np.linalg.svd(inliers-center,full_matrices=False);n=V[-1];n=n if n@prior>0 else -n;bias=-center@n;best=np.abs(a@n+bias)<.015
 inliers=a[best];center=inliers.mean(axis=0);_,sing,V=np.linalg.svd(inliers-center,full_matrices=False);n=V[-1];n=n if n@prior>0 else -n;bias=-center@n
 spread=float(sing[1]/np.sqrt(len(inliers)));rms=float(np.sqrt(np.mean((inliers@n+bias)**2)))
 if len(inliers)<30 or spread<.05:continue
 normals.append(n);results.append({'cloud_index':int(i),'effective_source_s':float(ct[i]+ta[best].mean()),'mean_point_offset_s':float(ta[best].mean()),'point_time_std_s':float(ta[best].std()),'inliers':len(inliers),'plane_rms_m':rms,'second_spatial_std_m':spread,'plane_offset_m':float(bias),'prior_difference_deg':float(np.rad2deg(np.arccos(np.clip(n@prior,-1,1))))})
np.savez_compressed(DEST/'floor_normals.npz',times=np.array([r['effective_source_s'] for r in results]),normals=np.array(normals))
(DEST/'floor_fits.json').write_text(json.dumps({'fits':results,'method':'Unconstrained RANSAC+SVD plane fit, with only broad floor candidate selection from body attitude/height. Plane normal not forced to IMU direction; effective time is mean inlier point timestamp. Used solely for offline timing diagnosis, no map or pose estimator launched.'},indent=2)+'\n')
print('Floor fits:',len(results),'median inliers',np.median([r['inliers'] for r in results]),'median rms m',np.median([r['plane_rms_m'] for r in results]),'spread prior difference deg',np.percentile([r['prior_difference_deg'] for r in results],[0,50,95,100]))
