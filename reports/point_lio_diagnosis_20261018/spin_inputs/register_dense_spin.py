"""Offline LiDAR-only registration to a confirmed stationary reference; no ROS."""
from pathlib import Path
import argparse,json,time
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
P=argparse.ArgumentParser();P.add_argument('--input',required=True);P.add_argument('--end',type=float,default=63);P.add_argument('--label',default='dense');P.add_argument('--point-window',type=float,default=1.0);a=P.parse_args();DEST=Path(a.input).resolve();d=np.load(DEST/'raw_points.npz');ref=int(d['stamp_ns'][0]);ct=(d['stamp_ns']-ref)/1e9;points=d['points'];times=d['point_times'];offset=d['offsets'];geo=json.loads(Path('/home/jain/Doggy/config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics'];R=np.array(geo['mapping.extrinsic_R']).reshape(3,3);trans=np.array(geo['mapping.extrinsic_T']);reg=o3d.pipelines.registration;rows=[];poses=[];guess=np.eye(4);start=time.monotonic()
def raw(i):
 lo,hi=offset[i:i+2];x=points[lo:hi].astype(float)@R.T+trans;t=times[lo:hi].astype(float);valid=(np.linalg.norm(x,axis=1)<8)&np.isfinite(x).all(axis=1)&np.isfinite(t);return x[valid],t[valid]
def voxel(x,t,size):
 _,iv=np.unique(np.floor(x/size).astype(np.int64),axis=0,return_inverse=True);c=np.bincount(iv);return np.column_stack([np.bincount(iv,weights=x[:,j])/c for j in range(3)]),np.bincount(iv,weights=t)/c
ref_indexes=np.flatnonzero((ct>=.5)&(ct<=4.5));rx=np.concatenate([raw(i)[0] for i in ref_indexes]);rx,_=voxel(rx,np.zeros(len(rx)),.035);target=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(rx));search=o3d.geometry.KDTreeSearchParamHybrid(radius=.18,max_nn=35);target.estimate_normals(search);target.estimate_covariances(search);ev=np.linalg.eigvalsh(np.asarray(target.covariances));valid=(ev[:,0]/np.maximum(ev.sum(axis=1),1e-12)<.035)&(ev[:,1]/np.maximum(ev[:,2],1e-12)>.04);target=target.select_by_index(np.flatnonzero(valid));print(json.dumps({'reference_scans':len(ref_indexes),'reference_points':len(target.points)}),flush=True)
def align(source,initial):
 result=None;T=initial
 for distance,it in [(.5,30),(.20,25),(.10,25)]:
  result=reg.registration_icp(source,target,distance,T,reg.TransformationEstimationPointToPlane(reg.HuberLoss(.02)),reg.ICPConvergenceCriteria(max_iteration=it));T=result.transformation
 return result
indexes=np.flatnonzero((ct>=5)&(ct<=a.end))
for k,i in enumerate(indexes):
 x,t=raw(i)
 if a.point_window<1:
  keep=np.abs(t-np.median(t))<=a.point_window/2;x,t=x[keep],t[keep]
 x,t=voxel(x,t,.045)
 if len(x)<80:continue
 source=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(x));fit=align(source,guess);T=np.array(fit.transformation,copy=True);pairs=np.asarray(fit.correspondence_set)
 if len(pairs)<30:continue
 transformed=x[pairs[:,0]]@T[:3,:3].T+T[:3,3];normals=np.asarray(target.normals)[pairs[:,1]];y=np.asarray(target.points)[pairs[:,1]];residual=np.sum(normals*(transformed-y),axis=1);w=np.minimum(1,.02/np.maximum(np.abs(residual),1e-12));J=np.column_stack([np.cross(transformed,normals),normals]);H=(J*w[:,None]).T@J/len(J);er=np.linalg.eigvalsh(H[:3,:3]-H[:3,3:]@np.linalg.pinv(H[3:,3:],rcond=1e-8)@H[3:,:3]);et=np.linalg.eigvalsh(H[3:,3:]);rr=float(max(0,er[0])/max(er[-1],1e-12));tr=float(max(0,et[0])/max(et[-1],1e-12));reason=[]
 if fit.fitness<.65:reason.append('low_overlap')
 if fit.inlier_rmse>.05:reason.append('large_correspondence_residual')
 if rr<.003:reason.append('rotation_degeneracy')
 if tr<.003:reason.append('translation_degeneracy')
 if np.sqrt(np.mean(residual**2))>.025:reason.append('large_plane_residual')
 perturb=np.eye(4);perturb[:3,:3]=Rotation.from_rotvec([.01,-.01,.025]).as_matrix();perturb[:3,3]=[.02,-.02,.01];second=align(source,guess@perturb);T2=np.array(second.transformation,copy=True);angle=float(np.rad2deg(Rotation.from_matrix(T[:3,:3].T@T2[:3,:3]).magnitude()));position=float(np.linalg.norm(T[:3,3]-T2[:3,3]))
 if angle>.2:reason.append('initialization_rotation_inconsistent')
 if position>.015:reason.append('initialization_translation_inconsistent')
 accepted=not reason
 if accepted:guess=T
 weight=w*np.linalg.norm(J[:,:3],axis=1)**2;weight/=max(weight.sum(),1e-12)
 rows.append({'cloud_index':int(i),'time_allpoints_s':float(ct[i]+t.mean()),'time_matched_s':float(ct[i]+np.mean(t[pairs[:,0]])),'time_weighted_s':float(ct[i]+weight@t[pairs[:,0]]),'scan_point_time_std_s':float(t.std()),'source_points':len(x),'correspondences':len(pairs),'fitness':float(fit.fitness),'inlier_rmse_m':float(fit.inlier_rmse),'plane_residual_rms_m':float(np.sqrt(np.mean(residual**2))),'rotation_info_ratio':rr,'translation_info_ratio':tr,'initialization_angle_difference_deg':angle,'initialization_position_difference_m':position,'accepted':accepted,'rejection_reasons':reason});poses.append(T)
 if k%100==0:print(json.dumps({'cloud':k,'total':len(indexes),'accepted':sum(r['accepted'] for r in rows),'elapsed_s':round(time.monotonic()-start,1)}),flush=True)
np.savez_compressed(DEST/(a.label+'_poses.npz'),pose=np.array(poses));result={'label':a.label,'reference_source_interval_s':[.5,4.5],'reference_scans':len(ref_indexes),'reference_points':len(target.points),'evaluated_clouds':len(rows),'accepted_clouds':sum(r['accepted'] for r in rows),'point_time_window_s':a.point_window,'elapsed_s':time.monotonic()-start,'rejection_counts':{s:sum(s in r['rejection_reasons'] for r in rows) for s in sorted({v for r in rows for v in r['rejection_reasons']})},'rows':rows,'method':'Dense stationary raw-LiDAR reference (user-confirmed hold), 3.5 cm voxels and planar-neighbour normal filtering. Source scans 4.5 cm voxels with centroid point times; robust point-to-plane multiscale alignment. Initial guesses use only last accepted LiDAR pose; second perturbed initialization checks local consistency. No IMU or vendor pose/velocity guides registration.','limits':['Reference stationarity assumed from user-confirmed motion sequence; small standing sway can broaden the reference.','Fixed initial-view reference may not cover scenes observed after large turns.','Full-scan intra-scan distortion and effective point-time uncertainty remain.','Quality gates reduce but cannot eliminate wrong matches.'],'mapping_or_replay_started':False,'ros_publications':False,'active_configuration_changed':False};(DEST/(a.label+'_poses.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['rows','method','limits']},indent=2))
