"""Measured-motion overview for user confirmation; no calibration or mapper."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
ROOT=Path('/home/jain/Doggy');BASE=Path(__file__).resolve().parent;info=json.loads((BASE/'current_recording.json').read_text());DEST=ROOT/info['directory'];assert info['status']=='stopped_saved';d=np.load(DEST/'decoded_samples.npz');ref=int(d['body_stamp_ns'][0]);bt=(d['body_stamp_ns']-ref)/1e9;ot=(d['odom_stamp_ns']-ref)/1e9
# Cue annotation only: estimate its source-time location from nearby receipt/source
# differences. Original timestamps are never modified in any saved sensor stream.
near=np.abs(d['body_receipt_ns']-int(info['cue_host_ns']))<2*10**9;offset=int(np.median((d['body_receipt_ns']-d['body_stamp_ns'])[near]));cue=(int(info['cue_host_ns'])-offset-ref)/1e9;baseline=(bt>=cue)&(bt<=cue+8);obase=(ot>=cue)&(ot<=cue+8)
angles=Rotation.from_quat(d['body_values'][:,[7,8,9,6]]).as_euler('xyz',degrees=True);oq=d['odom_values'][:,[4,5,6,3]];oyaw=Rotation.from_quat(oq).as_euler('xyz')[:,2];yaw0=float(np.median(np.unwrap(oyaw)[obase]));p0=np.median(d['odom_values'][obase,:3],axis=0);c,s=np.cos(yaw0),np.sin(yaw0);rot=np.array([[c,-s],[s,c]]);xy=(d['odom_values'][:,:2]-p0[:2])@rot;z=d['odom_values'][:,2]-p0[2];bodyyaw=np.rad2deg(np.unwrap(np.deg2rad(angles[:,2])));bodyyaw-=np.median(bodyyaw[baseline]);vendor_yaw=np.rad2deg(np.unwrap(oyaw)-yaw0)
# Movement intervals are a rough index for the user, not assumed ground truth.
grid=np.arange(max(bt[0],ot[0]),min(bt[-1],ot[-1]),.05);gxy=np.column_stack([np.interp(grid,ot,xy[:,j]) for j in range(2)]);smooth=gaussian_filter1d(gxy,3,axis=0);vel=np.gradient(smooth,.05,axis=0);speed=np.linalg.norm(vel,axis=1);gyaw=gaussian_filter1d(np.interp(grid,bt,bodyyaw),3);yawrate=np.abs(np.gradient(gyaw,.05));tilt=np.column_stack([np.interp(grid,bt,angles[:,j]) for j in range(2)]);tiltrate=np.linalg.norm(np.gradient(gaussian_filter1d(tilt,3,axis=0),.05,axis=0),axis=1)
def segments(mask):
 ix=np.flatnonzero(mask);groups=np.split(ix,np.flatnonzero(np.diff(ix)>.8/.05)+1);out=[]
 for g in groups:
  if len(g)<8 or grid[g[-1]]-grid[g[0]]<.5:continue
  out.append([round(float(grid[g[0]]),1),round(float(grid[g[-1]]),1)])
 return out
intervals={'translation_from_vendor_odom_speed_over_0_04_m_s':segments(speed>.04),'turning_from_body_yaw_rate_over_5_deg_s':segments(yawrate>5),'tilting_from_roll_pitch_rate_over_5_deg_s':segments(tiltrate>5)}
summary={'recording_duration_s':(info['stored_receipt_end_ns']-info['stored_receipt_start_ns'])/1e9,'cue_source_elapsed_s_approx':cue,'position_reference':'Vendor /utlidar/robot_odom, centred at median first 8 seconds after cue and rotated into initial vendor heading; not independent LiDAR-only or ground truth.','orientation_reference':'Calibrated body IMU quaternion; roll/pitch absolute Euler angles, yaw unwrapped relative to initial stationary heading.','forward_range_m':[float(xy[:,0].min()),float(xy[:,0].max())],'left_range_m':[float(xy[:,1].min()),float(xy[:,1].max())],'height_change_range_m':[float(z.min()),float(z.max())],'body_rpy_range_deg':[ [float(angles[:,j].min()),float(angles[:,j].max())] for j in [0,1]]+[[float(bodyyaw.min()),float(bodyyaw.max())]],'rough_movement_intervals_source_elapsed_s':intervals,'user_confirmation_pending':True,'note':'Paths need not be straight; the calibration analysis must use actual measured motion. This plot does not establish timing synchronization.'};(DEST/'motion_overview.json').write_text(json.dumps(summary,indent=2)+'\n')
fig,axs=plt.subplots(2,2,figsize=(13,8.5),layout='constrained');fig.suptitle('Recorded motion — confirm the path, turns and body tilts',fontsize=15)
ix=np.arange(0,len(ot),8);xy0=xy[ix];segments_xy=np.stack([xy0[:-1],xy0[1:]],axis=1);lc=LineCollection(segments_xy,cmap='viridis',norm=plt.Normalize(ot[0],ot[-1]),linewidth=2);lc.set_array(ot[ix][:-1]);axs[0,0].add_collection(lc);axs[0,0].autoscale();axs[0,0].scatter(*xy[0],c='#159957',s=65,edgecolors='black',zorder=4,label='Start');axs[0,0].scatter(*xy[-1],c='#db4a38',marker='X',s=80,zorder=4,label='End');axs[0,0].set_aspect('equal',adjustable='datalim');axs[0,0].set_xlabel('Forward from initial position (m)');axs[0,0].set_ylabel('Left from initial position (m)');axs[0,0].set_title('A. Horizontal path — vendor odometry estimate');axs[0,0].legend(fontsize=8,loc='best');fig.colorbar(lc,ax=axs[0,0],label='Recording time (s)',shrink=.8)
axs[0,1].plot(ot[::8],xy[::8,0],label='Forward',color='#2171b5');axs[0,1].plot(ot[::8],xy[::8,1],label='Left',color='#d95f02');axs[0,1].plot(ot[::8],z[::8],label='Height change',color='#636363',lw=.8);axs[0,1].set_ylabel('Displacement (m)');axs[0,1].set_title('B. Translation — vendor odometry estimate');axs[0,1].legend(fontsize=8)
axs[1,0].plot(bt[::12],angles[::12,0],label='Roll — side tilt',color='#2171b5');axs[1,0].plot(bt[::12],angles[::12,1],label='Pitch — forward/back tilt',color='#d95f02');axs[1,0].set_title('C. Body tilt — body IMU orientation');axs[1,0].set_ylabel('Angle (degrees)');axs[1,0].legend(fontsize=8)
axs[1,1].plot(bt[::12],bodyyaw[::12],label='Body IMU relative yaw',color='#6a3d9a');axs[1,1].plot(ot[::8],vendor_yaw[::8],label='Vendor odometry relative yaw',color='#159957',ls='--');axs[1,1].set_title('D. Turning — change from initial heading');axs[1,1].set_ylabel('Angle (degrees)');axs[1,1].legend(fontsize=8)
for ax in [axs[0,1],axs[1,0],axs[1,1]]:
 ax.axvline(cue,color='#555555',ls=':',lw=1);ax.set_xlabel('Time since recording began (s)');ax.set_xlim(0,bt[-1]);ax.grid(alpha=.25)
axs[0,0].grid(alpha=.25);fig.savefig(DEST/'motion_overview.png',dpi=165);fig.savefig(DEST/'motion_overview.pdf');print(json.dumps(summary,indent=2))
# A self-contained interactive view of the same recorded data; no external CDN.
# Inline time scrubber only changes a cursor and position marker in a fixed SVG.
N=900;oi=np.linspace(0,len(ot)-1,N).astype(int);bi=np.linspace(0,len(bt)-1,N).astype(int);data={'ot':ot[oi].tolist(),'xy':xy[oi].tolist(),'z':z[oi].tolist(),'bt':bt[bi].tolist(),'roll':angles[bi,0].tolist(),'pitch':angles[bi,1].tolist(),'yaw':bodyyaw[bi].tolist(),'duration':float(bt[-1])};(DEST/'motion_overview_data.json').write_text(json.dumps(data))
