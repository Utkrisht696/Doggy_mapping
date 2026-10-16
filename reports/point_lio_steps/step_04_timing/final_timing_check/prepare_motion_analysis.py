"""Prepare offline diagnostics for a closed manual posture bag; no ROS publishing."""
from pathlib import Path
import json,numpy as np
from scipy.ndimage import uniform_filter1d
ROOT=Path('/home/jain/Doggy');BASE=ROOT/'reports/point_lio_steps/step_04_timing/final_timing_check';info=json.loads((BASE/'current_motion.json').read_text());DEST=ROOT/info['directory'];assert info['status']=='stopped_saved';d=np.load(DEST/'decoded_samples.npz');ref=int(d['body_stamp_ns'][0]);bt=(d['body_stamp_ns']-ref)/1e9;g=d['body_values'][:,3:6];step=.004;grid=np.arange(bt[0],bt[-1],step);rates=np.column_stack([np.interp(grid,bt,g[:,j]) for j in range(3)]);smooth=np.sqrt(np.maximum(0,uniform_filter1d(rates**2,size=125,axis=0)));phases=[]
# Identify actual sustained axis excitation, then split independent repetitions
# at measured quiet gaps; no assumed five-degree angle or operator timing.
for axis,name in [(1,'pitch'),(0,'roll')]:
 active=np.flatnonzero((smooth[:,axis]>.035)&(smooth[:,axis]>1.15*smooth[:,1-axis]));groups=np.split(active,np.flatnonzero(np.diff(active)*step>4)+1)
 for k,group in enumerate(groups):
  if len(group)<200:continue
  lo,hi=max(float(grid[group[0]]-.8),.5),min(float(grid[group[-1]]+.8),float(grid[-1]-.5))
  if hi-lo<5:continue
  phases.append([f'{name}_{k+1}',lo,hi,axis])
phases.sort(key=lambda r:r[1]);(DEST/'motion_windows.json').write_text(json.dumps({'phases':phases,'method':'0.5 s RMS of measured gyro, >0.035 rad/s and 1.15x the other tilt axis; quiet gaps >4 s separate groups, duration at least 5 s. Used only to select excited diagnostic intervals.'},indent=2)+'\n');assert len(phases)>=2,'Insufficient pitch/roll excitation'
# Clone established analyses with explicit fresh destination and measured windows.
old=ROOT/'reports/point_lio_steps/step_04_timing'
text=(old/'analyze_posture_lag.py').read_text();a=text.index('ROOT=');b=text.index('\nref=',a);text=text[:a]+f"ROOT=Path({str(ROOT)!r});DEST=Path({str(DEST)!r});d=np.load(DEST/'decoded_samples.npz')"+text[b:];text=text.replace("phases=[('pitch',90.,126.,1),('roll',129.,165.,0)]",f'phases={phases!r}');text=text.replace('axs[0].set_xlim(75,175)',f'axs[0].set_xlim({max(0,phases[0][1]-5)!r},{phases[-1][2]+5!r})');text=text.replace('for left in np.arange(start,end-9,5):','for left in np.arange(start,end-5,3):').replace('grid<=left+10','grid<=left+6').replace("'end_s':float(left+10)","'end_s':float(left+6)").replace("a.std()<.04","a.std()<.025");(DEST/'analyze_gyro.py').write_text(text)
text=(old/'fit_cloud_floor.py').read_text();a=text.index('ROOT=');b=text.index('\npoints=',a);prefix=f"ROOT=Path({str(ROOT)!r});DEST=Path({str(DEST)!r});d=np.load(DEST/'decoded_samples.npz');ref=int(d['body_stamp_ns'][0]);bt=(d['body_stamp_ns']-ref)/1e9;ct=(d['cloud_stamp_ns']-ref)/1e9\ngeo=json.loads((ROOT/'config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics'];R=np.array(geo['mapping.extrinsic_R']).reshape(3,3);t=np.array(geo['mapping.extrinsic_T'])+np.array([-.02557,0,.04232]);bodyq=d['body_values'][:,[7,8,9,6]]";text=text[:a]+prefix+text[b:];text=text.replace('(ct>78)&(ct<172)',f'(ct>{phases[0][1]-2!r})&(ct<{phases[-1][2]+2!r})');(DEST/'fit_floor.py').write_text(text)
print(json.dumps({'directory':str(DEST.relative_to(ROOT)),'phases':phases},indent=2))
