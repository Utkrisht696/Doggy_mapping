from pathlib import Path
import argparse,json,numpy as np
from scipy.spatial.transform import Rotation
import open3d as o3d
p=argparse.ArgumentParser();p.add_argument('input');p.add_argument('output');a=p.parse_args();r=Path(a.input);out=Path(a.output);out.mkdir(parents=True,exist_ok=True);d=np.load(r/'raw_points.npz');origin=int(d['stamp_ns'][0]);t=(d['stamp_ns']-origin)/1e9;h=json.loads((r/'candidate_holds.json').read_text())['holds'];geo=json.loads(Path('/home/jain/Doggy/config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics'];R=np.array(geo['mapping.extrinsic_R']).reshape(3,3);T=np.array(geo['mapping.extrinsic_T']);reg=o3d.pipelines.registration
windows=[h[0]['source_window_s'],h[-1]['source_window_s']];windows[0]=[max(windows[0][0],1.),min(windows[0][1],5.5)]
def cloud(window):
 ids=np.flatnonzero((t>=window[0])&(t<=window[1]));x=np.concatenate([d['points'][d['offsets'][i]:d['offsets'][i+1]] for i in ids]).astype(float)@R.T+T;pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(x)).voxel_down_sample(.04);pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.2,max_nn=40));return pc,len(ids)
start,ns=cloud(windows[0]);end,ne=cloud(windows[1]);assert ns>=8 and ne>=8

def align(source,target,initial):
 tr=initial
 for rad,n in [(.6,40),(.25,40),(.10,40)]:
  fit=reg.registration_icp(source,target,rad,tr,reg.TransformationEstimationPointToPlane(reg.HuberLoss(.025)),reg.ICPConvergenceCriteria(max_iteration=n));tr=np.array(fit.transformation,copy=True)
 pairs=np.asarray(fit.correspondence_set);x=np.asarray(source.points)[pairs[:,0]]@tr[:3,:3].T+tr[:3,3];normal=np.asarray(target.normals)[pairs[:,1]];y=np.asarray(target.points)[pairs[:,1]];res=np.sum(normal*(x-y),axis=1);w=np.minimum(1,.025/np.maximum(abs(res),1e-12));J=np.column_stack([np.cross(x,normal),normal]);H=(J*w[:,None]).T@J/max(1,len(J));Hr=H[:3,:3]-H[:3,3:]@np.linalg.pinv(H[3:,3:],rcond=1e-8)@H[3:,:3];er=np.linalg.eigvalsh(Hr);et=np.linalg.eigvalsh(H[3:,3:]);return {'transform':tr.tolist(),'fitness':float(fit.fitness),'inlier_rmse_m':float(fit.inlier_rmse),'plane_rms_m':float(np.sqrt(np.mean(res**2))),'rotation_information_ratio':float(er[0]/max(er[-1],1e-12)),'translation_information_ratio':float(et[0]/max(et[-1],1e-12)),'yaw_deg':float(Rotation.from_matrix(tr[:3,:3]).as_euler('xyz',degrees=True)[2])}
results={}
for name,source,target in [('forward',end,start),('reverse',start,end)]:
 rows=[]
 for yaw in [0,-10,10,-30,30,-60,60]:
  tr=np.eye(4);tr[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix();v=align(source,target,tr);v['initial_yaw_deg']=yaw;rows.append(v)
 rows.sort(key=lambda v:(-v['fitness'],v['inlier_rmse_m']));results[name]={'best':rows[0],'starts':rows}
b=results['forward']['best'];rev=results['reverse']['best'];tr=np.array(b['transform']);cycle=tr@np.array(rev['transform']);consistent=[v for v in results['forward']['starts'] if np.linalg.norm(np.array(v['transform'])[:3,3]-tr[:3,3])<.03 and np.rad2deg(Rotation.from_matrix(tr[:3,:3].T@np.array(v['transform'])[:3,:3]).magnitude())<.3];angle=float(np.rad2deg(Rotation.from_matrix(cycle[:3,:3]).magnitude()));position=float(np.linalg.norm(cycle[:3,3]));ok=b['fitness']>.7 and b['inlier_rmse_m']<.06 and b['plane_rms_m']<.035 and b['rotation_information_ratio']>.003 and b['translation_information_ratio']>.003 and angle<.3 and position<.03 and len(consistent)>=3
report={'source':str(r),'source_windows_s':windows,'scans':[ns,ne],'voxel_points':[len(start.points),len(end.points)],'quality_passed':bool(ok),'consistent_forward_initializations':len(consistent),'reverse_cycle_angle_deg':angle,'reverse_cycle_translation_m':position,'end_imu_pose_in_start_imu_frame':{'xyz_m':tr[:3,3].tolist(),'rotation_xyzw':Rotation.from_matrix(tr[:3,:3]).as_quat().tolist(),'rpy_deg':Rotation.from_matrix(tr[:3,:3]).as_euler('xyz',degrees=True).tolist()},'registrations':results,'limits':['Independent LiDAR-only static start/end registration; no IMU, vendor pose or Point-LIO pose initializes ICP.','Multi-start/cycle/geometry checks do not guarantee correct correspondence in repeated scenes.','Endpoint check only; no trajectory/timing calibration or map correction is applied.','Clouds are accumulated only during detected quiet holds; sway and sensor noise remain.']};(out/'endpoint_registration.json').write_text(json.dumps(report,indent=2)+'\n');o3d.io.write_point_cloud(str(out/'start_cloud.ply'),start);end.transform(tr);o3d.io.write_point_cloud(str(out/'end_aligned.ply'),end);print(json.dumps({k:v for k,v in report.items() if k not in ['registrations','limits']},indent=2))
