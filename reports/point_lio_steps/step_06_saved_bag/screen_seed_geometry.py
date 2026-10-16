"""Read-only zero-motion plane-match proxy, not an actual estimator trial."""
from pathlib import Path
import numpy as np,json
from scipy.spatial import cKDTree
p=Path('reports/point_lio_steps/step_04_timing/temporal_motion/20261016T205503+1100');d=np.load(p/'decoded_samples.npz');off=d['cloud_offsets'];pts=d['cloud_points'];g=json.loads(Path('config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics'];R=np.array(g['mapping.extrinsic_R']).reshape(3,3);t=np.array(g['mapping.extrinsic_T']);b=d['body_values'];bm=(d['body_stamp_ns']>=d['cloud_stamp_ns'][0])&(d['body_stamp_ns']<d['cloud_stamp_ns'][0]+10**9);vertical=np.mean(b[bm,:3],axis=0);vertical/=np.linalg.norm(vertical)
def voxel(points):
 _,inv=np.unique(np.floor(points/.1).astype(np.int64),axis=0,return_inverse=True);counts=np.bincount(inv);return np.column_stack([np.bincount(inv,weights=points[:,j])/counts for j in range(3)])
def cloud(i):return voxel(pts[off[i]:off[i+1]])@R.T+t
res=[];query_index=22;query=cloud(query_index)
for size in [1,10,20]:
 target=np.vstack([cloud(i) for i in range(2,2+size)]);tree=cKDTree(target);dist,index=tree.query(query,k=5);normals=[];acceptedpoints=[]
 for point,ds,ids in zip(query,dist,index):
  if ds[-1]>np.sqrt(5):continue
  neighbours=target[ids];n=np.linalg.lstsq(neighbours,-np.ones(5),rcond=None)[0];length=np.linalg.norm(n)
  if length<1e-12:continue
  n/=length;offset=1/length
  if np.max(abs(neighbours@n+offset))>.1:continue
  pd=point@n+offset
  if np.linalg.norm(point)<=81*pd**2:continue
  normals.append(n);acceptedpoints.append(point)
 normals=np.array(normals);acceptedpoints=np.array(acceptedpoints)
 row={'seed_scans':size,'seed_points_before_map_voxel_merging':len(target),'query_points':len(query),'accepted_plane_matches':len(normals),'accepted_fraction':len(normals)/len(query)}
 if len(normals):
  row['near_horizontal_plane_fraction']=float(np.mean(abs(normals@vertical)>.95));row['normal_information_eigenvalues']=np.linalg.eigvalsh(normals.T@normals).tolist()
  H=np.c_[normals,np.cross(acceptedpoints,normals)];J=H[:,3:];S=J.T@J-J.T@normals@np.linalg.pinv(normals.T@normals)@normals.T@J;row['rotation_schur_eigenvalues']=np.linalg.eigvalsh(S).tolist()
 res.append(row)
run=Path(json.loads(Path('reports/point_lio_steps/step_06_saved_bag/current_run.json').read_text())['directory']);result={'method':'Offline NumPy nearest-five plane-test approximation at zero motion, using scan 2 as a seed hypothesis; not a measurement from the running filter. The same held-out query scan is used for all seed counts. PCL voxel/numerical/map-filter differences are possible. Accepted matches do not prove correspondence correctness or estimator stability.','query_scan_index':query_index,'query_source_elapsed_s':float((d['cloud_stamp_ns'][query_index]-d['cloud_stamp_ns'][0])/1e9),'results':res};(run/'seed_geometry_screening.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
