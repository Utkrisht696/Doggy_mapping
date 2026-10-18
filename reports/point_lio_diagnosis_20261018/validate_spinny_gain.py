from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent;r=root/'spin_inputs/spinny';s=np.load(r/'source_decoded.npz');t=(s['imu_stamp_ns']-s['raw_stamp_ns'][0])/1e9;dt=np.diff(t);v=s['imu_values'];gain=json.loads((root/'gyro_gain_running_fit.json').read_text())['fits'][0]['gyro_z_gain'];holds=json.loads((r/'candidate_holds.json').read_text())['holds'];masks=[(t>=h['source_window_s'][0])&(t<=h['source_window_s'][1]) for h in holds]
def integrated(k):
 w=v[:,3:6].copy();w[:,2]*=k;dq=Rotation.from_rotvec((w[1:]+w[:-1])*.5*dt[:,None]).as_quat();q=np.empty((len(t),4));q[0]=[0,0,0,1]
 for j in range(len(dt)):
  x,y,z,qw=q[j];a,b,c,d=dq[j];qq=np.array([qw*a+x*d+y*c-z*b,qw*b-x*c+y*d+z*a,qw*c+x*b-y*a+z*d,qw*d-x*a-y*b-z*c]);q[j+1]=qq/np.linalg.norm(qq)
 y=np.rad2deg(np.unwrap(Rotation.from_quat(q).as_euler('xyz')[:,2]));return y-np.median(y[masks[0]])
raw,cal=integrated(1),integrated(gain);cw1=json.loads((root/'endpoint_truth/spinny_cw/wall_reference.json').read_text());cw2=json.loads((root/'endpoint_truth/spinny_ccw/wall_reference.json').read_text());ccw1=json.loads((root/'endpoint_truth/spinny_after_ccw1/wall_reference.json').read_text());end=json.loads((root/'endpoint_truth/spinny_end/wall_reference.json').read_text());assert all(d['quality_passed'] for d in [cw1,cw2,ccw1,end]);ref={0:0.,1:cw1['end_pose_in_start_frame']['yaw_deg'],2:cw1['end_pose_in_start_frame']['yaw_deg']+cw2['end_pose_in_start_frame']['yaw_deg'],3:ccw1['end_pose_in_start_frame']['yaw_deg'],6:end['end_pose_in_start_frame']['yaw_deg']}
for hi in ref:ref[hi]+=360*round((np.median(raw[masks[hi]])-ref[hi])/360)
rows=[]
for name,i,j in [('CW1',0,1),('CW2',1,2),('CCW1',2,3)]:
 actual=ref[j]-ref[i];a=float(np.median(raw[masks[j]])-np.median(raw[masks[i]]));b=float(np.median(cal[masks[j]])-np.median(cal[masks[i]]));rows.append({'block':name,'source_windows_s':[holds[i]['source_window_s'],holds[j]['source_window_s']],'independent_wall_rotation_deg':actual,'raw_gyro_rotation_deg':a,'corrected_gyro_rotation_deg':b,'raw_error_deg':a-actual,'corrected_error_deg':b-actual})
report={'gain_z':gain,'gain_training_bag':'running_mode_spins','validation_bag':'spinny','gain_refitted_on_validation_bag':False,'blocks':rows,'stationary_wall_references':{str(i):{'unwrapped_yaw_deg':float(y),'source_window_s':holds[i]['source_window_s']} for i,y in ref.items()},'limits':['CW2 global reference composes two independently accepted wall matches; CCW1 is a difference of accepted global wall references.','The second CCW stop fails wall-check quality gates and is excluded from quantitative validation.','Gain is calibrated only for body gyro Z and current hardware/configuration. All original data and active IMU adapter are untouched.']};(root/'spinny_gain_validation.json').write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(root/'spinny_gain_validation.npz',time=t,raw_gyro_heading=raw,corrected_gyro_heading=cal)
fig,axs=plt.subplots(2,1,figsize=(12,8),layout='constrained');axs[0].plot(t,raw,label='Raw calibrated-body gyro integration');axs[0].plot(t,cal,label='Measured Z gain applied');axs[1].plot(t,raw-cal,label='Raw minus corrected gyro integration')
for i,y in ref.items():
 lo,hi=holds[i]['source_window_s'];axs[0].plot([(lo+hi)/2],[y],'ko');axs[0].annotate(str(i),((lo+hi)/2,y),xytext=(4,6),textcoords='offset points')
axs[0].plot([],[],'ko',label='Independent quiet-wall references');axs[0].set(ylabel='Unwrapped heading change (degrees)',title='Held-out spinny: fixed gain measured on an earlier bag');axs[1].set(ylabel='Accumulated gain effect (degrees)')
for ax in axs:ax.set_xlabel('Source time (s)');ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.savefig(root/'spinny_gain_validation.png',dpi=150);print(json.dumps(report,indent=2))
