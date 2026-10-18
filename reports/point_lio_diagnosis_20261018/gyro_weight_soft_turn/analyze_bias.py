from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scipy.integrate import cumulative_trapezoid
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(__file__).resolve().parent;s=np.load(r/'source_decoded.npz');origin=int(s['raw_stamp_ns'][0]);it=(s['imu_stamp_ns']-origin)/1e9;iv=s['imu_values'];dt=np.diff(it)
assert np.all((dt>0)&(dt<.05))

def integrate(gyro):
 w=(gyro[1:]+gyro[:-1])*.5;dq=Rotation.from_rotvec(w*dt[:,None]).as_quat();q=np.empty((len(it),4));q[0]=iv[0,6:10]
 for k in range(len(dt)):
  x,y,z,qw=q[k];a,b,c,d=dq[k];v=np.array([qw*a+x*d+y*c-z*b,qw*b-x*c+y*d+z*a,qw*c+x*b-y*a+z*d,qw*d-x*a-y*b-z*c]);q[k+1]=v/np.linalg.norm(v)
 return Rotation.from_quat(q)
plain=integrate(iv[:,3:6]);rows={};fig,axs=plt.subplots(2,1,figsize=(12,8),layout='constrained')
for label,folder in [('control',Path('reports/point_lio_diagnosis_20261018/heading_trace_soft_turn').resolve()),('gyro_weight',r)]:
 c=np.genfromtxt(folder/'src/point_lio/Log/heading_trace.csv',delimiter=',',names=True);a=np.load(folder/'run_01/analysis/trace_analysis.npz');t=a['time'];bias=np.column_stack([np.interp(it,t,c[n]) for n in ['bgx','bgy','bgz']]);brot=integrate(iv[:,3:6]-bias);m=(t>=2)&(t<=6);analyzed=(t>=2)&(t<=141.557172143)
 def aligned(rot):
  y=np.rad2deg(np.unwrap(Slerp(it,rot)(t).as_euler('xyz')[:,2]));return y-np.median(y[m])
 raw=aligned(plain);sub=aligned(brot);pred=a['prediction'];bgz_integral=np.rad2deg(cumulative_trapezoid(bias[:,2],it,initial=0));zi=np.interp(t,it,bgz_integral);zi-=np.median(zi[m]);diff=pred-sub;points=[]
 for target in [40.009201288,55.899199724,61.928814888,140.002700328]:
  k=int(np.argmin(abs(t-target)));points.append({'source_s':float(t[k]),'plain_gyro_heading_deg':float(raw[k]),'gyro_minus_estimated_bias_heading_deg':float(sub[k]),'predicted_heading_contribution_deg':float(pred[k]),'estimated_z_bias_integral_deg':float(zi[k])})
 rows[label]={'prediction_vs_bias_subtracted_integration_rms_deg':float(np.sqrt(np.mean(diff[analyzed]**2))),'prediction_vs_bias_subtracted_integration_max_abs_deg':float(np.max(abs(diff[analyzed]))),'snapshots':points}
 axs[0].plot(t,pred,label=label+': prediction');axs[0].plot(t,sub,'--',label=label+': gyro minus estimated bias');axs[1].plot(t,pred-raw,label=label+': prediction minus gyro');axs[1].plot(t,sub-raw,'--',label=label+': effect of subtracting estimated bias')
rows['limits']=['Diagnostic reconstruction from per-scan final bias interpolated to IMU times; it cannot reproduce transient bias changes within scans.','This does not prove the physically correct bias or validate a bias-lock remedy. Body orientation and gyro share sensor inputs.','No estimator is modified or rerun for this analysis.'];(r/'run_01/analysis/bias_reconstruction.json').write_text(json.dumps(rows,indent=2)+'\n')
axs[0].set(ylabel='Heading change (degrees)',title='Prediction and integration with estimated residual gyro bias');axs[1].set(ylabel='Heading difference (degrees)',title='Effect of the estimator’s additional bias estimate')
for ax in axs:ax.set_xlabel('Source time (s)');ax.legend(fontsize=8);ax.grid(alpha=.25)
fig.savefig(r/'run_01/analysis/bias_reconstruction.png',dpi=150);print(json.dumps(rows,indent=2))
