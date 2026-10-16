"""Read-only comparison of saved failed mapper output against the closed bag."""
from pathlib import Path
from collections import Counter
import sqlite3,json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
sys.path.insert(0,str(Path('scripts').resolve()))
from lidar_cloud_utils import cloud_arrays
base=Path('reports/point_lio_steps/step_06_saved_bag');p=Path(json.loads((base/'current_run.json').read_text())['directory']);bag=Path('reports/point_lio_steps/step_04_timing/temporal_motion/20261016T205503+1100');src=np.load(bag/'decoded_samples.npz');out=np.load(p/'outputs.npz');origin=int(src['cloud_stamp_ns'][0]);end=int(out['input_cloud_stamp_ns'][-1]);mask=(src['odom_stamp_ns']>=origin)&(src['odom_stamp_ns']<=end);v=src['odom_values'][mask];bm=(src['body_stamp_ns']>=origin)&(src['body_stamp_ns']<=end);body=src['body_values'][bm]
def angles(q):
 q=q/np.linalg.norm(q,axis=1)[:,None];return 2*np.arccos(np.clip(abs(q @ q[0]),0,1))*180/np.pi
r={'observed_source_interval_s':(end-origin)/1e9,'vendor_body_position_max_displacement_m':float(np.linalg.norm(v[:,:3]-v[0,:3],axis=1).max()),'body_fused_orientation_max_change_deg':float(angles(body[:,6:10]).max()),'point_lio_orientation_max_change_from_first_deg':float(angles(out['odom_values'][:,3:7]).max()),'point_lio_position_max_displacement_from_first_m':float(np.linalg.norm(out['odom_values'][:,:3]-out['odom_values'][0,:3],axis=1).max()),'estimator_tf_stamps_exact_match':bool(np.array_equal(out['odom_stamp_ns'],out['map_odom_stamp_ns']))}
for label,fullkey,outkey in [('imu','body_stamp_ns','input_imu_stamp_ns'),('cloud','cloud_stamp_ns','input_cloud_stamp_ns')]:
 received=out[outkey];expected=src[fullkey];expected=expected[(expected>=received[0])&(expected<=received[-1])];r[label+'_missing_in_observed_stamp_interval']=int(len(np.setdiff1d(expected,received)))
profile=json.loads(Path('config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics'];R=np.array(profile['mapping.extrinsic_R']).reshape(3,3);t=np.array(profile['mapping.extrinsic_T'])+[-.02557,0,.04232]
def keyed(m):
 xyz,records,valid=cloud_arrays(m);f={v.name:v for v in m.fields};endian='>' if m.is_bigendian else '<';dtype=np.dtype({'names':['ring','time','intensity'],'formats':[endian+'u2',endian+'u4',endian+'u4'],'offsets':[f[v].offset for v in ('ring','time','intensity')],'itemsize':m.point_step});keys=np.ascontiguousarray(records).view(dtype).reshape(-1);tuples=[tuple(map(int,(k['ring'],k['time'],k['intensity']))) for k in keys];counts=Counter(k for k,v in zip(tuples,valid) if v);return {k:xyz[i].astype(float) for i,k in enumerate(tuples) if valid[i] and counts[k]==1}
con=sqlite3.connect('file:'+str((bag/'bag/bag_0.db3').resolve())+'?mode=ro',uri=True);ids=dict(con.execute('select name,id from topics'));names={ids[k]:k for k in ['/utlidar/cloud','/utlidar/cloud_base']};pending={};residual=[];pairs=[]
for topic,data in con.execute('select topic_id,data from messages where topic_id in (?,?) order by timestamp limit 500',(ids['/utlidar/cloud'],ids['/utlidar/cloud_base'])):
 m=deserialize_message(data,PointCloud2);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 if stamp>end:continue
 pending.setdefault(stamp,{})[names[topic]]=m
 if len(pending[stamp])==2:
  pair=pending.pop(stamp);ka=keyed(pair['/utlidar/cloud']);kb=keyed(pair['/utlidar/cloud_base']);keys=ka.keys() & kb.keys()
  if keys:
   x=np.array([ka[k] for k in keys]);y=np.array([kb[k] for k in keys]);err=np.linalg.norm(x @ R.T+t-y,axis=1);residual.extend(err.tolist());pairs.append({'source_elapsed_s':(stamp-origin)/1e9,'matches':len(err),'max_residual_m':float(err.max())})
con.close();r['raw_to_vendor_body_geometry_pairs']=len(pairs);r['raw_to_vendor_body_matched_points']=len(residual)
if residual:r['raw_to_vendor_body_geometry_residual_m_quantiles']=dict(zip(['median','p95','max'],map(float,np.percentile(residual,[50,95,100]))))
r['raw_to_body_comparison_note']='Uniquely matched ring/time/intensity signatures, not array indices; vendor body clouds contain a filtered subset.'
(p/'failure_crosschecks.json').write_text(json.dumps(r,indent=2)+'\n');(p/'geometry_pair_checks.json').write_text(json.dumps(pairs,indent=2)+'\n')
t=(out['odom_stamp_ns']-origin)/1e9;t_body=(src['body_stamp_ns'][bm]-origin)/1e9;t_vendor=(src['odom_stamp_ns'][mask]-origin)/1e9
fig,axs=plt.subplots(2,2,figsize=(12,8));fig.suptitle('First Point-LIO test: stationary recording, stopped after 13.7 s',fontsize=14)
ax=axs[0,0];ax.plot(t,np.linalg.norm(out['odom_values'][:,:3]-out['odom_values'][0,:3],axis=1),label='Point-LIO');ax.plot(t_vendor,np.linalg.norm(v[:,:3]-v[0,:3],axis=1),label='Vendor odometry reference');ax.set(ylabel='Displacement from initial pose (m)',xlabel='Recording time (s)');ax.legend();ax.grid(alpha=.3)
ax=axs[0,1];ax.plot(t,angles(out['odom_values'][:,3:7]),label='Point-LIO');ax.plot(t_body,angles(body[:,6:10]),label='Body fused orientation reference');ax.set(ylabel='Rotation from initial orientation (degrees)',xlabel='Recording time (s)');ax.legend();ax.grid(alpha=.3)
points=out['cloud_points'];offset=out['cloud_offsets'];n=len(out['cloud_stamp_ns']);colors=['#3b82f6','#fb923c']
for ax,coord,label in [(axs[1,0],1,'Y'),(axs[1,1],2,'Z')]:
 for idx,color,name in [(0,colors[0],'First registered scan'),(n-1,colors[1],'Last registered scan')]:
  cloud=points[offset[idx]:offset[idx+1]];ax.scatter(cloud[:,0],cloud[:,coord],s=3,alpha=.5,c=color,label=name)
 ax.set(xlabel='Map X (m)',ylabel='Map '+label+' (m)');ax.axis('equal');ax.legend(markerscale=2);ax.grid(alpha=.3)
fig.tight_layout();fig.savefig(p/'stationary_failure.png',dpi=160);fig.savefig(p/'stationary_failure.pdf');plt.close(fig)
print(json.dumps(r,indent=2))
