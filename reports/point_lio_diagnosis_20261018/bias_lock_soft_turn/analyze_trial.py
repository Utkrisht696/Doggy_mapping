# Same hold analysis with a 0.05 s startup margin: the small bag has only a short initial quiet hold. Require >=8 finite output samples.
from pathlib import Path
import json,sys,numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(sys.argv[1]);run=r/(sys.argv[2] if len(sys.argv)>2 else 'run_01');out=run/'analysis';out.mkdir(exist_ok=True);s=np.load(r/'source_decoded.npz');a=np.load(run/'outputs.npz');o=int(s['raw_stamp_ns'][0]);t=(a['odom_stamp_ns']-o)/1e9;p=a['odom_values'][:,:3];lr=Rotation.from_quat(a['odom_values'][:,3:7]);vt=(s['vendor_stamp_ns']-o)/1e9;v=s['vendor_values'];vr=Slerp(vt,Rotation.from_quat(v[:,3:7]))(t);vp=np.column_stack([np.interp(t,vt,v[:,j]) for j in range(3)])+vr.apply([-.02557,0,.04232]);candidates=json.loads((r/'candidate_holds.json').read_text())['holds'];holds=[];fixed=json.loads(Path('/home/jain/Doggy/reports/point_lio_diagnosis_20261018/bias_lock_soft_turn/fixed_analysis_windows.json').read_text())
for h in candidates:
 start,end=fixed[str(h['index'])];assert start>=t[0] and end<=t[-1]
 if end-start<.5:continue
 if end-start>6:start=end-6
 mask=(t>=start)&(t<=end)
 if mask.sum()<8:continue
 row={'source_hold_index':h['index'],'analysis_window_s':[start,end],'samples':int(mask.sum())}
 for name,pts,rot in [('point_lio',p,lr),('vendor',vp,vr)]:
  pp=pts[mask];med=np.median(pp,axis=0);row[name]={'median_xyz_m':med.tolist(),'mean_quaternion_xyzw':rot[mask].mean().as_quat().tolist(),'position_p95_radius_m':float(np.percentile(np.linalg.norm(pp-med,axis=1),95)),'hold_first_last_shift_m':float(np.linalg.norm(np.median(pp[-4:],axis=0)-np.median(pp[:4],axis=0)))}
 holds.append(row)
assert len(holds)>=2
byindex={h['source_hold_index']:h for h in holds}
def relative(h0,h1,label):
 b,e=h0[label],h1[label];dp=np.array(e['median_xyz_m'])-b['median_xyz_m'];q0=Rotation.from_quat(b['mean_quaternion_xyzw']);q1=Rotation.from_quat(e['mean_quaternion_xyzw']);yaw0=q0.as_euler('xyz')[2];yaw1=q1.as_euler('xyz')[2];return {'horizontal_displacement_m':float(np.linalg.norm(dp[:2])),'height_change_m':float(dp[2]),'heading_change_deg':float(np.rad2deg((yaw1-yaw0+np.pi)%(2*np.pi)-np.pi)),'delta_xyz_m':dp.tolist(),'full_orientation_change_deg':float(np.rad2deg((q0.inv()*q1).magnitude()))}
endpoints={label:relative(holds[0],holds[-1],label) for label in ['point_lio','vendor']}
checks=[]
specs={'measured_route_2':[(5.,0,1,2),(10.,2,3,4)],'measured_route_1':[(5.,0,1,None),(10.,0,2,None)],'doggy_real1':[(5.,0,1,2),(10.,2,3,4)]}.get(r.name,[])
for distance,i,j,k in specs:
 if i not in byindex or j not in byindex:continue
 row={'nominal_distance_m':distance,'route_assignment':('user-confirmed straight sequence' if r.name in ['measured_route_1','measured_route_2'] else 'inferred from source order and motion; intermediate route unconfirmed'),'start_hold':i,'far_hold':j,'return_hold':k}
 for label in ['point_lio','vendor']:
  rr=relative(byindex[i],byindex[j],label);rr['nominal_difference_m']=rr['horizontal_displacement_m']-distance;rr['nominal_difference_percent']=100*rr['nominal_difference_m']/distance
  if k is not None and k in byindex:rr['return_to_segment_start']=relative(byindex[i],byindex[k],label)
  row[label]=rr
 checks.append(row)
start=holds[0];ly=Rotation.from_quat(start['point_lio']['mean_quaternion_xyzw']).as_euler('xyz')[2];vy=Rotation.from_quat(start['vendor']['mean_quaternion_xyzw']).as_euler('xyz')[2];lp=Rotation.from_euler('z',-ly).apply(p-start['point_lio']['median_xyz_m']);rp=Rotation.from_euler('z',-vy).apply(vp-start['vendor']['median_xyz_m']);dt=np.diff(t);step=np.linalg.norm(np.diff(p,axis=0),axis=1);angle=np.rad2deg((lr[:-1].inv()*lr[1:]).magnitude());contig=dt<.15

def travel(pts):
 total=0.
 for ix in np.split(np.arange(len(t)),np.flatnonzero(~contig)+1):
  if len(ix)<3:continue
  grid=np.arange(t[ix[0]],t[ix[-1]],.1)
  if len(grid)<2:continue
  pp=np.column_stack([np.interp(grid,t[ix],pts[ix,j]) for j in range(3)]);pp=gaussian_filter1d(pp,2,axis=0);total+=np.linalg.norm(np.diff(pp,axis=0),axis=1).sum()
 return float(total)
def stats(x):return {k:float(v) for k,v in zip(['median','p95','p99','max'],np.percentile(x,[50,95,99,100]))} if len(x) else {}
log=(run/'mapping.log').read_text();events=[line.split('RECOVERY ',1)[1] for line in log.splitlines() if 'RECOVERY ' in line];gaps={}
for k in ['raw','imu','vendor']:
 st=s[k+'_stamp_ns'];ds=np.diff(st)/1e9;gaps[k]=[{'before_source_s':float((st[ix]-o)/1e9),'after_source_s':float((st[ix+1]-o)/1e9),'duration_s':float(ds[ix])} for ix in np.flatnonzero(ds>.15)]
health={'poses':len(t),'first_pose_source_s':float(t[0]),'last_pose_source_s':float(t[-1]),'nonfinite_values':int((~np.isfinite(a['odom_values'])).sum()),'nonincreasing_stamps':int((np.diff(a['odom_stamp_ns'])<=0).sum()),'median_contiguous_rate_hz':float(1/np.median(dt[contig])),'contiguous_position_step_m':stats(step[contig]),'contiguous_rotation_step_deg':stats(angle[contig]),'output_gaps_over_150ms':[{'before_s':float(t[ix]),'duration_s':float(dt[ix]),'pose_change_m':float(step[ix]),'rotation_change_deg':float(angle[ix])} for ix in np.flatnonzero(~contig)],'missing_exact_stamp_tf':int(len(np.setdiff1d(a['odom_stamp_ns'],a['map_odom_stamp_ns']))),'reseed_events':sum(e.startswith('reseeded') for e in events),'resume_events':sum(e.startswith('resumed') for e in events),'probation_events':sum(e.startswith('probation') for e in events)}
report={'bag':json.loads((r/'input_audit.json').read_text())['source_bag'],'run':str(run),'route_confirmation':'Uses the user instruction that recordings return to the original starting pose; these new comeback filenames agree. Physical endpoint uncertainty and exact wandering route are not surveyed.','startup':json.loads((r/'stationary_start.json').read_text()),'holds':holds,'nominal_distance_checks':checks,'first_to_last_analyzed_hold':endpoints,'estimated_observed_travel_excluding_gaps_m':{'point_lio':travel(p),'vendor':travel(vp)},'health':health,'source_gaps_over_150ms':gaps,'recovery_events':events,'replay_result':json.loads((run/'result.json').read_text()),'limits':['User confirmed intended physical return to start pose, but endpoint uncertainty is not surveyed; intermediate distance labels remain inferred unless separately confirmed.','Reported straight distances are horizontal hold-to-hold displacements; curved walking path length differs.','Vendor is a comparison sharing body IMU information, not ground truth.','Same approved fixed calibration and zero uncalibrated acquisition-lag correction; no per-bag tuning.','Recovered trial uses vendor aid only at detected gaps. No loop closure.','Output gap in accelerated replay can reflect playback delivery; continuous input requires separate verification.']};(out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(out/'comparison.npz',time=t,lio_position_initial_heading=lp,vendor_position_initial_heading=rp)
fig,axs=plt.subplots(2,2,figsize=(13,9),layout='constrained');fig.suptitle(r.name+' | Point-LIO vs vendor | return-to-start recording; approximate endpoint')
for pts,label in [(lp,'Point-LIO'),(rp,'Vendor')]:
 pp=pts.copy();pp[np.r_[False,~contig]]=np.nan;axs[0,0].plot(pp[:,0],pp[:,1],label=label);axs[0,1].plot(t,pp[:,0],label=label);axs[1,0].plot(t,pp[:,2]*100,label=label)
axs[0,0].scatter(0,0,c='black',marker='*',s=70,label='First analyzed hold');axs[0,0].scatter(*lp[-1,:2],c='red',marker='X',label='Point-LIO finish');axs[0,0].set(xlabel='Initial heading X (m)',ylabel='Y (m)',title='No scale fitting; one initial alignment');axs[0,0].set_aspect('equal',adjustable='datalim');axs[0,1].set(xlabel='Source time (s)',ylabel='Forward displacement (m)',title='Full motion sequence');axs[1,0].set(xlabel='Source time (s)',ylabel='Height change (cm)',title='Height throughout route')
for rot,y0,label in [(lr,ly,'Point-LIO'),(vr,vy,'Vendor')]:
 y=np.unwrap(rot.as_euler('xyz')[:,2]);y=np.rad2deg(y-y0);y-=360*round(np.median(y[(t>=start['analysis_window_s'][0])&(t<=start['analysis_window_s'][1])])/360);y[np.r_[False,~contig]]=np.nan;axs[1,1].plot(t,y,label=label)
axs[1,1].set(xlabel='Source time (s)',ylabel='Unwrapped heading change (degrees)',title='Turns and accumulated heading change')
for ax in axs.flat:ax.grid(alpha=.25);ax.legend(fontsize=8)
for h in holds:
 begin,end=h['analysis_window_s'];mid=(begin+end)/2;m=(t>=begin)&(t<=end);axs[0,1].axvspan(begin,end,color='green',alpha=.08);axs[0,1].annotate(str(h['source_hold_index']),(mid,float(np.median(lp[m,0]))),xytext=(0,7),textcoords='offset points',ha='center',fontsize=8)
for gap in gaps['imu']:
 for ax in [axs[0,1],axs[1,0],axs[1,1]]:ax.axvspan(gap['before_source_s'],gap['after_source_s'],color='red',alpha=.12)
fig.savefig(out/'comparison.png',dpi=150);plt.close(fig);print(json.dumps({'bag':r.name,'straight_checks':checks,'endpoints':endpoints,'health':health,'travel':report['estimated_observed_travel_excluding_gaps_m']},indent=2))
