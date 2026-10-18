from pathlib import Path
import json,sys,numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(sys.argv[1]);run=r/(sys.argv[2] if len(sys.argv)>2 else 'run_01');a=np.load(run/'outputs.npz');s=np.load(r/'source_decoded.npz');o=int(s['raw_stamp_ns'][0]);t=(a['odom_stamp_ns']-o)/1e9;it=(s['imu_stamp_ns']-o)/1e9;vt=(s['vendor_stamp_ns']-o)/1e9;iv=s['imu_values'];dt=np.diff(it);w=(iv[1:,3:6]+iv[:-1,3:6])*.5;dq=Rotation.from_rotvec(w*dt[:,None]).as_quat();integrated=np.empty((len(it),4));integrated[0]=iv[0,6:10];reset=0
for k in range(len(dt)):
 if dt[k]>.05:integrated[k+1]=iv[k+1,6:10];reset+=1;continue
 x,y,z,qw=integrated[k];a0,b,c,d=dq[k];q=np.array([qw*a0+x*d+y*c-z*b,qw*b-x*c+y*d+z*a0,qw*c+x*b-y*a0+z*d,qw*d-x*a0-y*b-z*c]);integrated[k+1]=q/np.linalg.norm(q)
rot={'Point-LIO':Rotation.from_quat(a['odom_values'][:,3:]),'Vendor':Slerp(vt,Rotation.from_quat(s['vendor_values'][:,3:7]))(t),'Body orientation':Slerp(it,Rotation.from_quat(iv[:,6:10]))(t),'Integrated calibrated gyro':Slerp(it,Rotation.from_quat(integrated))(t)}
d=json.loads((run/'analysis/comparison.json').read_text());h0,h1=d['holds'][0],d['holds'][-1];m0=(t>=h0['analysis_window_s'][0])&(t<=h0['analysis_window_s'][1]);m1=(t>=h1['analysis_window_s'][0])&(t<=h1['analysis_window_s'][1]);yaws={};ends={};firstdif={};fig,axs=plt.subplots(2,1,figsize=(12,8),layout='constrained')
for label,q in rot.items():
 y=np.rad2deg(np.unwrap(q.as_euler('xyz')[:,2]));y-=np.median(y[m0]);yaws[label]=y;ends[label]=float(np.median(y[m1]));axs[0].plot(t,y,label=label)
for label in ['Point-LIO','Vendor','Integrated calibrated gyro']:
 diff=yaws[label]-yaws['Body orientation'];axs[1].plot(t,diff,label=label+' minus body orientation');firstdif[label]={str(level):float(t[ix[0]]) if len(ix:=np.flatnonzero(np.abs(diff)>level)) else None for level in [5,10,20,40]}
axs[0].set(ylabel='Unwrapped heading change (degrees)',title=r.name+': independent heading signals, each zeroed at first hold');axs[1].set(xlabel='Source time (s)',ylabel='Heading disagreement (degrees)',title='No heading-scale fitting or time-offset fitting')
for ax in axs:ax.grid(alpha=.25);ax.legend(fontsize=9)
fig.savefig(run/'analysis/heading_comparison.png',dpi=150);plt.close(fig);report={'heading_change_at_final_hold_deg':ends,'first_abs_disagreement_vs_body_orientation_s':firstdif,'gyro_integration_resets_at_gaps_over_50ms':reset,'method':'Trapezoidal calibrated body angular velocity integrated as body-frame quaternion increments, no new bias estimate, scale fitting or timestamp fitting. This is a diagnostic; signals share sensors and none is external surveyed heading.'};(run/'analysis/heading_comparison.json').write_text(json.dumps(report,indent=2)+'\n');print(r.name,json.dumps(report))
