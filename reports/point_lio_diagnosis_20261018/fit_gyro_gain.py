from pathlib import Path
import argparse,json,numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import brentq
p=argparse.ArgumentParser();p.add_argument('input');p.add_argument('reference');p.add_argument('output');p.add_argument('--gain',type=float);a=p.parse_args();r=Path(a.input);ref=json.loads(Path(a.reference).read_text());assert ref['quality_passed'];s=np.load(r/'source_decoded.npz');origin=int(s['raw_stamp_ns'][0]);it=(s['imu_stamp_ns']-origin)/1e9;v=s['imu_values'];dt=np.diff(it);assert np.all((dt>0)&(dt<.05));wins=ref['source_windows_s'];m0=(it>=wins[0][0])&(it<=wins[0][1]);m1=(it>=wins[1][0])&(it<=wins[1][1]);bias=v[m0,3:6].mean(axis=0)
def change(gain,bias):
 w=v[:,3:6]-bias;w=w.copy();w[:,2]*=gain;dq=Rotation.from_rotvec((w[1:]+w[:-1])*.5*dt[:,None]).as_quat();q=np.empty((len(it),4));q[0]=[0,0,0,1]
 for k in range(len(dt)):
  x,y,z,qw=q[k];aa,b,c,d=dq[k];zq=np.array([qw*aa+x*d+y*c-z*b,qw*b-x*c+y*d+z*aa,qw*c+x*b-y*aa+z*d,qw*d-x*aa-y*b-z*c]);q[k+1]=zq/np.linalg.norm(zq)
 y=np.rad2deg(np.unwrap(Rotation.from_quat(q).as_euler('xyz')[:,2]));return float(np.median(y[m1])-np.median(y[m0]))
physical=ref['end_pose_in_start_frame']['yaw_deg'];rows=[]
for label,b in [('existing_fixed_bias_only',np.zeros(3)),('additional_stationary_start_bias',bias)]:
 nominal=change(1,b);turns=int(round((nominal-physical)/360));target=physical+turns*360
 gain=brentq(lambda z:change(z,b)-target,.94,1.06,xtol=1e-9) if a.gain is None else a.gain
 adjusted=change(gain,b);rows.append({'method':label,'additional_bias_rad_s':b.tolist(),'nominal_integrated_heading_change_deg':nominal,'nearest_full_turns':turns,'independent_wall_heading_modulo_deg':physical,'unwrapped_target_deg':target,'gyro_z_gain':gain,'adjusted_integrated_heading_change_deg':adjusted,'residual_deg':adjusted-target,'gain_fitted_on_this_bag':a.gain is None})
report={'source':str(r),'reference':a.reference,'fits':rows,'physical_reference':'Independent raw-cloud quiet-wall registration, with its quality gates passed.','limits':['Full-turn integer selected nearest to gyro integration; assumes total gyro error is less than 180 degrees and no missed acquisition.','A fitted gain is not validated until tested unchanged on independent turn data.','Stationary-start residual bias is measured from a detected quiet window; it is separate from the fixed adapter calibration.']};Path(a.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
