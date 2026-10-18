from pathlib import Path
import sys,json,numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation,Slerp
r=Path(sys.argv[1]);s=np.load(r/'source_decoded.npz');o=int(s['raw_stamp_ns'][0]);vt=(s['vendor_stamp_ns']-o)/1e9;it=(s['imu_stamp_ns']-o)/1e9;v=s['vendor_values'];im=s['imu_values'];grid=np.arange(max(vt[0],it[0]),min(vt[-1],it[-1]),.1);p=np.column_stack([np.interp(grid,vt,v[:,j]) for j in range(3)]);p=gaussian_filter1d(p,1.,axis=0);speed=np.linalg.norm(np.gradient(p,.1,axis=0),axis=1);gyro=np.sqrt(np.interp(grid,it,gaussian_filter1d(np.sum(im[:,3:6]**2,axis=1),25)));q=Slerp(vt,Rotation.from_quat(v[:,3:7]))(grid);yaw=np.unwrap(q.as_euler('xyz')[:,2]);yawrate=np.abs(np.gradient(gaussian_filter1d(yaw,1.),.1));dist_v=np.abs(grid[:,None]-vt[np.clip(np.searchsorted(vt,grid)[:,None]+np.array([-1,0]),0,len(vt)-1)]).min(axis=1);dist_i=np.abs(grid[:,None]-it[np.clip(np.searchsorted(it,grid)[:,None]+np.array([-1,0]),0,len(it)-1)]).min(axis=1);valid=(dist_v<.05)&(dist_i<.04);still=(speed<.05)&(gyro<.08)&(yawrate<np.deg2rad(5))&valid
ix=np.flatnonzero(still);groups=np.split(ix,np.flatnonzero(np.diff(ix)>1)+1);holds=[]
for g in groups:
 if len(g)<18:continue
 start=float(grid[g[0]]+.3);end=float(grid[g[-1]]-.3)
 mask=(vt>=start)&(vt<=end);med=np.median(v[mask,:3],axis=0);rot=Rotation.from_quat(v[mask,3:7]).mean();holds.append({'index':len(holds),'source_window_s':[start,end],'vendor_body_median_xyz_m':med.tolist(),'vendor_body_yaw_deg':float(rot.as_euler('xyz',degrees=True)[2]),'vendor_position_p95_radius_m':float(np.percentile(np.linalg.norm(v[mask,:3]-med,axis=1),95))})
d={'source_origin_ns':o,'duration_source_s':float(grid[-1]),'method':'Candidate stationary windows: 0.1s grid, smoothed vendor position speed <0.05m/s, body gyro RMS <0.08rad/s, yaw rate <5deg/s; >=1.8s, trimmed 0.3s. Actual acquisition holes excluded. These identify pauses, not surveyed poses.','holds':holds};(r/'candidate_holds.json').write_text(json.dumps(d,indent=2)+'\n');print(r.name)
for h in holds:print(h['index'],np.round(h['source_window_s'],1),np.round(h['vendor_body_median_xyz_m'],3),round(h['vendor_body_yaw_deg'],1))
