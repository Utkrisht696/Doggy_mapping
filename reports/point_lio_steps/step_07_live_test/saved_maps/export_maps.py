"""Offline export of recorded, map-frame registered scans; never runs an estimator."""
from pathlib import Path
import json,sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import plotly.graph_objects as go

base=Path(__file__).resolve().parent.parent
for label,run_id in [('drive','20261016T220926+1100'),('restart','20261016T221648+1100')]:
 run=base/run_id; out=base/'saved_maps'/label; out.mkdir(exist_ok=True)
 db=next((run/'bag').glob('*.db3')); c=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
 topic=c.execute("select id from topics where name='/point_lio/cloud_registered'").fetchone()[0]
 chunks=[]; count=0
 for row in c.execute('select data from messages where topic_id=? order by timestamp,id',(topic,)):
  m=deserialize_message(row[0],PointCloud2)
  if m.header.frame_id!='map':raise ValueError('Unexpected cloud frame '+m.header.frame_id)
  offsets={f.name:f.offset for f in m.fields}; names=['x','y','z','intensity']; endian='>' if m.is_bigendian else '<'
  dtype=np.dtype({'names':names,'formats':[endian+'f4']*4,'offsets':[offsets[n] for n in names],'itemsize':m.point_step})
  a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)
  p=np.column_stack([a[n] for n in names]);chunks.append(p[np.isfinite(p[:,:3]).all(axis=1)]);count+=1
 c.close(); points=np.concatenate(chunks).astype('<f4'); del chunks
 n=len(points)
 def pcd(path,p):
  head=f'# Recorded Point-LIO registered scans; frame=map\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(p)}\nDATA binary\n'
  with path.open('wb') as f:f.write(head.encode());f.write(p.astype('<f4',copy=False).tobytes())
 pcd(out/'registered_scans_full.pcd',points)
 _,idx=np.unique(np.floor(points[:,:3]/.05).astype(np.int64),axis=0,return_index=True)
 p=points[np.sort(idx)];pcd(out/'map_5cm.pcd',p)
 a=np.load(run/'observed_outputs.npz'); trajectory=a['odom_values'][:,:3]; stamps=a['odom_stamp_ns'];t=(stamps-stamps[0])/1e9
 np.savetxt(out/'trajectory.csv',np.column_stack([t,trajectory]),delimiter=',',header='seconds_from_first_pose,x_m,y_m,z_m',comments='')
 # Full export is unchanged; visualization excludes outer 0.5% XYZ tails for readable limits.
 lo,hi=np.percentile(p[:,:3],[.5,99.5],axis=0);margin=np.maximum((hi-lo)*.05,.2); inside=((p[:,:3]>=lo-margin)&(p[:,:3]<=hi+margin)).all(axis=1);v=p[inside];v=v[::max(1,int(np.ceil(len(v)/200000)))];zlo,zhi=np.percentile(v[:,2],[2,98])
 title='Live drive — saved Point-LIO scans' if label=='drive' else 'Short restart — saved Point-LIO scans'
 plt.style.use('dark_background'); fig=plt.figure(figsize=(15,7)); ax=fig.add_subplot(121)
 sc=ax.scatter(v[:,0],v[:,1],c=v[:,2],cmap='turbo',s=.6,vmin=zlo,vmax=zhi,rasterized=True);ax.plot(trajectory[:,0],trajectory[:,1],c='white',lw=1,label='Estimated path');ax.scatter(*trajectory[0,:2],c='lime',s=40,label='Start');ax.scatter(*trajectory[-1,:2],c='red',s=40,label='End');ax.set(xlabel='map X (m)',ylabel='map Y (m)',title='Top view');ax.set_aspect('equal');ax.legend(loc='upper right',fontsize=8)
 ax=fig.add_subplot(122,projection='3d');vs=v[::max(1,int(np.ceil(len(v)/90000)))];ax.scatter(vs[:,0],vs[:,1],vs[:,2],c=vs[:,2],cmap='turbo',s=.3,vmin=zlo,vmax=zhi,depthshade=False);ax.plot(*trajectory.T,c='white',lw=1);ax.view_init(25,-60);ax.set(xlabel='map X (m)',ylabel='map Y (m)',zlabel='map Z (m)',title='Oblique view');ax.set_box_aspect(np.maximum(hi-lo,.1));fig.suptitle(title+'\nAccumulated registered scans, 5 cm voxels; height colouring',fontsize=13);fig.colorbar(sc,ax=fig.axes[:2],label='map Z (m)',shrink=.5,pad=.07);fig.savefig(out/'map_preview.png',dpi=160,bbox_inches='tight');plt.close(fig)
 traces=[go.Scatter3d(x=v[:,0],y=v[:,1],z=v[:,2],mode='markers',marker=dict(size=1.5,color=v[:,2],colorscale='Turbo',cmin=float(zlo),cmax=float(zhi),colorbar=dict(title='Z (m)')),name='Registered scans',hoverinfo='skip'),go.Scatter3d(x=trajectory[:,0],y=trajectory[:,1],z=trajectory[:,2],mode='lines',line=dict(color='white',width=5),name='Estimated path'),go.Scatter3d(x=trajectory[[0,-1],0],y=trajectory[[0,-1],1],z=trajectory[[0,-1],2],mode='markers+text',marker=dict(color=['lime','red'],size=6),text=['Start','End'],name='Endpoints')]
 fig=go.Figure(traces);fig.update_layout(title=title+' — drag to rotate, scroll to zoom',template='plotly_dark',scene=dict(aspectmode='data',xaxis_title='map X (m)',yaxis_title='map Y (m)',zaxis_title='map Z (m)'),margin=dict(l=0,r=0,b=0,t=55));fig.write_html(out/'map_3d.html',include_plotlyjs=True,auto_open=False)
 result={'source_run':str(run.relative_to(base.parent.parent.parent)),'source_bag':str(db),'frame':'map','registered_scan_messages':count,'full_finite_points':n,'voxel_size_m':.05,'voxel_points':len(p),'preview_points':len(v),'output_duration_s':float(t[-1]),'pose_extent_xyz_m':np.ptp(trajectory,axis=0).tolist(),'map_type':'Accumulated recorded Point-LIO registered scans; not an internal estimator map dump or 2D occupancy grid','visualization_only_tail_crop_percent':.5,'full_pcd_cropped':False,'estimator_rerun':False}
 (out/'export.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'label':label,**result}),flush=True)
