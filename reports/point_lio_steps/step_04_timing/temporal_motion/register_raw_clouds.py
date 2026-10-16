"""Offline independent scan-pair registration, no ROS or IMU/odometry guidance."""
from pathlib import Path
import argparse,json,time
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
P=argparse.ArgumentParser();P.add_argument('--method',choices=['plane','point'],default='plane');P.add_argument('--stride',type=int,default=12);P.add_argument('--label',default='pilot');P.add_argument('--voxel',type=float,default=.06);P.add_argument('--point-window',type=float,default=1.0);a=P.parse_args()
ROOT=Path('/home/jain/Doggy');BASE=Path(__file__).resolve().parent;info=json.loads((BASE/'current_recording.json').read_text());DEST=ROOT/info['directory'];assert info['status']=='stopped_saved';d=np.load(DEST/'decoded_samples.npz');ref=int(d['body_stamp_ns'][0]);ct=(d['cloud_stamp_ns']-ref)/1e9;points=d['cloud_points'];times=d['cloud_point_times'];offset=d['cloud_offsets'];geo=json.loads((ROOT/'config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics'];R=np.array(geo['mapping.extrinsic_R']).reshape(3,3);trans=np.array(geo['mapping.extrinsic_T']);cache={};reg=o3d.pipelines.registration;rows=[];rotations=[];translations=[];started=time.monotonic()
def cloud(i):
 if i in cache:return cache[i]
 l,r=offset[i:i+2];x=points[l:r].astype(float)@R.T+trans;t=times[l:r].astype(float);keep=np.isfinite(x).all(axis=1)&np.isfinite(t)&(np.linalg.norm(x,axis=1)<8)
 if a.point_window<1:
  mid=float(np.median(t[keep]));keep&=np.abs(t-mid)<=a.point_window/2
 x=x[keep];t=t[keep];keys=np.floor(x/a.voxel).astype(np.int64);_,ix,inv=np.unique(keys,axis=0,return_index=True,return_inverse=True);count=np.bincount(inv);x=np.column_stack([np.bincount(inv,weights=x[:,j])/count for j in range(3)]);t=np.bincount(inv,weights=t)/count
 pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(x));pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.28,max_nn=25));cache[i]=(pc,t)
 return cache[i]
indexes=np.flatnonzero((ct>62)&(ct<216))[::a.stride]
for k,i in enumerate(indexes):
 if i<1:continue
 prev,tp=cloud(i-1);cur,tc=cloud(i)
 if len(prev.points)<100 or len(cur.points)<100:continue
 def align(source,target):
  estimator=reg.TransformationEstimationPointToPlane(reg.HuberLoss(.025)) if a.method=='plane' else reg.TransformationEstimationPointToPoint()
  initial=np.eye(4)
  for radius in ([.3,.12] if a.method=='plane' else [.25,.12,.06]):
   result=reg.registration_icp(source,target,radius,initial,estimator,reg.ICPConvergenceCriteria(max_iteration=40));initial=result.transformation
  return result
 fit=align(cur,prev);T=np.array(fit.transformation,copy=True);rev=align(prev,cur);cycle=T@rev.transformation;cycle_angle=float(np.rad2deg(Rotation.from_matrix(cycle[:3,:3]).magnitude()));cycle_translation=float(np.linalg.norm(cycle[:3,3]));pairs=np.asarray(fit.correspondence_set)
 if len(pairs)<20:continue
 x=np.asarray(cur.points)[pairs[:,0]]@T[:3,:3].T+T[:3,3];normals=np.asarray(prev.normals)[pairs[:,1]];target=np.asarray(prev.points)[pairs[:,1]];residual=np.sum(normals*(x-target),axis=1);w=np.minimum(1,.025/np.maximum(np.abs(residual),1e-12));J=np.column_stack([np.cross(x,normals),normals]);H=(J*w[:,None]).T@J/max(1,len(J));Hr=H[:3,:3]-H[:3,3:]@np.linalg.pinv(H[3:,3:],rcond=1e-8)@H[3:,:3];er=np.linalg.eigvalsh(Hr);et=np.linalg.eigvalsh(H[3:,3:]);rr=float(max(0,er[0])/max(er[-1],1e-12));tr=float(max(0,et[0])/max(et[-1],1e-12));rv=Rotation.from_matrix(T[:3,:3]).as_rotvec();dt=(ct[i]+float(tc.mean()))-(ct[i-1]+float(tp.mean()));reason=[]
 for failed,name in [(len(pairs)<100,'too_few_correspondences'),(fit.fitness<.65,'low_overlap'),(fit.inlier_rmse>.06,'large_distance_residual'),(cycle_angle>.3,'rotation_cycle_error'),(cycle_translation>.02,'translation_cycle_error'),(rr<.003,'rotation_degeneracy'),(tr<.003,'translation_degeneracy'),(np.linalg.norm(rv)/dt>3,'implausible_pair_rotation_rate')]:
  if failed:reason.append(name)
 # Timestamp alternatives expose sensitivity to the effective scan acquisition time.
 wp=np.linalg.norm(J[:,:3],axis=1)**2;wp/=max(wp.sum(),1e-12);ta=float(ct[i-1]+np.mean(tp[pairs[:,1]]));tb=float(ct[i]+np.mean(tc[pairs[:,0]]));taw=float(ct[i-1]+wp@tp[pairs[:,1]]);tbw=float(ct[i]+wp@tc[pairs[:,0]])
 rows.append({'cloud_previous':int(i-1),'cloud_current':int(i),'t_previous_s':ta,'t_current_s':tb,'t_previous_weighted_s':taw,'t_current_weighted_s':tbw,'t_previous_allpoints_s':float(ct[i-1]+tp.mean()),'t_current_allpoints_s':float(ct[i]+tc.mean()),'current_voxel_points':len(cur.points),'previous_voxel_points':len(prev.points),'correspondences':len(pairs),'fitness':float(fit.fitness),'inlier_rmse_m':float(fit.inlier_rmse),'plane_residual_rms_m':float(np.sqrt(np.mean(residual**2))),'rotation_information_eigenvalues':er.tolist(),'rotation_information_ratio':rr,'translation_normal_eigenvalues':et.tolist(),'translation_information_ratio':tr,'cycle_angle_deg':cycle_angle,'cycle_translation_m':cycle_translation,'rotation_speed_rad_s':float(np.linalg.norm(rv)/dt),'accepted':not reason,'rejection_reasons':reason});rotations.append(T[:3,:3]);translations.append(T[:3,3])
 if k%25==0:print(json.dumps({'pair':k,'total':len(indexes),'accepted':sum(r['accepted'] for r in rows),'elapsed_s':round(time.monotonic()-started,1)}),flush=True)
 if len(cache)>10:cache.clear()
np.savez_compressed(DEST/(a.label+'_registration.npz'),rotation=np.array(rotations),translation=np.array(translations));reasons={reason:sum(reason in r['rejection_reasons'] for r in rows) for reason in sorted({s for r in rows for s in r['rejection_reasons']})};summary={'label':a.label,'pairs':len(rows),'accepted_pairs':sum(r['accepted'] for r in rows),'rejection_counts':reasons,'stride':a.stride,'registration_method':a.method,'voxel_m':a.voxel,'point_time_window_s':a.point_window,'elapsed_s':time.monotonic()-started,'method':'Consecutive raw scans converted with fixed approved extrinsics, voxelised with per-voxel mean point time. Multiscale ICP from identity (point/plane per registration_method), equal independent reverse procedure, geometric Schur rotation/translation conditioning. No IMU, body attitude, vendor odometry or map used to initialize registration.','rows':rows,'limits':['Full scans can be motion-distorted; scan-average effective time is uncertain.','Good overlap/conditioning/cycle consistency do not prove correct correspondences or true registration.','Quality thresholds are diagnostic choices, not a calibration accuracy specification.'],'mapper_or_replay_started':False,'application_changed':False};(DEST/(a.label+'_registration.json')).write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='rows'},indent=2))
