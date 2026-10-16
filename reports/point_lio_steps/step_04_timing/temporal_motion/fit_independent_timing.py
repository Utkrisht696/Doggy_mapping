"""Compare accepted independent cloud rotations to body gyro; never apply a fit."""
from pathlib import Path
import argparse,json
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=argparse.ArgumentParser();P.add_argument('--label',default='dense_full');a=P.parse_args();ROOT=Path('/home/jain/Doggy');BASE=Path(__file__).resolve().parent;info=json.loads((BASE/'current_recording.json').read_text());DEST=ROOT/info['directory'];d=np.load(DEST/'decoded_samples.npz');registration=json.loads((DEST/(a.label+'_poses.json')).read_text());poses=np.load(DEST/(a.label+'_poses.npz'))['pose'];rows=registration['rows'];ref=int(d['body_stamp_ns'][0]);bt=(d['body_stamp_ns']-ref)/1e9;gyro=d['body_values'][:,3:6];q=np.empty((len(bt),4));rot=Rotation.identity();q[0]=rot.as_quat()
for i in range(1,len(bt)):
 rot=rot*Rotation.from_rotvec((gyro[i-1]+gyro[i])*.5*(bt[i]-bt[i-1]));q[i]=rot.as_quat()
interp=Slerp(bt,Rotation.from_quat(q));lookup={r['cloud_index']:i for i,r in enumerate(rows)};pairs=[]
for j,r in enumerate(rows):
 i=lookup.get(r['cloud_index']-3)
 if i is None or not r['accepted'] or not rows[i]['accepted']:continue
 if r['time_allpoints_s']-rows[i]['time_allpoints_s']>.26:continue
 if rows[i]['time_allpoints_s']<bt[0]+.2 or r['time_allpoints_s']>bt[-1]-.2:continue
 pairs.append((i,j))
results=[];curves=[];lags=np.arange(-.1,.100001,.001);phases=[('first_walk',68,113),('first_turns',113,147),('body_tilts',154,179),('repeat_walk_turns',183,212),('train_first_blocks',68,179)]
if pairs:
 first=np.array([i for i,j in pairs]);last=np.array([j for i,j in pairs]);measured=Rotation.from_matrix(np.array([poses[i,:3,:3].T@poses[j,:3,:3] for i,j in pairs]));magnitude=measured.magnitude();alltime=np.array([rows[i]['time_allpoints_s'] for i in last]);mid=(np.array([rows[i]['time_allpoints_s'] for i in first])+alltime)*.5
else:first=last=np.array([],dtype=int);mid=np.array([])
for time_mode in ['allpoints','matched','weighted']:
 if not pairs:break
 t1=np.array([rows[i]['time_'+time_mode+'_s'] for i in first]);t2=np.array([rows[i]['time_'+time_mode+'_s'] for i in last]);dt=t2-t1;speed=magnitude/dt;excited=speed>.08;losses=[];errors=[]
 for lag in lags:
  predicted=interp(t1+lag).inv()*interp(t2+lag);angle=(measured.inv()*predicted).magnitude();h=np.deg2rad(.3);loss=np.where(angle<=h,angle**2,2*h*angle-h*h);losses.append(loss);errors.append(angle)
 losses=np.array(losses);errors=np.array(errors)
 for name,left,right in phases:
  mask=(mid>=left)&(mid<=right)&excited;count=int(mask.sum())
  if count<12:
   results.append({'time_mode':time_mode,'phase':name,'usable_excited_pairs':count,'fit_available':False});continue
  score=losses[:,mask].mean(axis=1);k=int(np.argmin(score));z=int(np.argmin(np.abs(lags)));near=lags[score<=score[k]*1.05];rmse0=float(np.rad2deg(np.sqrt(np.mean(errors[z,mask]**2))));rmsebest=float(np.rad2deg(np.sqrt(np.mean(errors[k,mask]**2))));r={'time_mode':time_mode,'phase':name,'usable_excited_pairs':count,'fit_available':True,'best_body_minus_cloud_offset_s':float(lags[k]),'boundary_hit':bool(k in [0,len(lags)-1]),'relative_improvement_over_zero_percent':float(100*(score[z]-score[k])/score[z]),'relative_rotation_rmse_zero_deg':rmse0,'relative_rotation_rmse_best_deg':rmsebest,'within_5_percent_minimum_loss_range_s':[float(near.min()),float(near.max())]};results.append(r);curves.append((time_mode,name,score))
 # Validate first-block training lag on an untouched later movement section.
 train=next((r for r in results if r['time_mode']==time_mode and r['phase']=='train_first_blocks' and r['fit_available']),None);held=(mid>=183)&(mid<=212)&excited
 if train and held.sum()>=12:
  k=int(np.argmin(abs(lags-train['best_body_minus_cloud_offset_s'])));z=int(np.argmin(abs(lags)));train['held_out_repeat_pairs']=int(held.sum());train['held_out_improvement_percent']=float(100*(losses[z,held].mean()-losses[k,held].mean())/losses[z,held].mean())
result={'label':a.label,'independent_cloud_poses':registration['accepted_clouds'],'all_registered_clouds':registration['evaluated_clouds'],'accepted_endpoint_relative_pairs':len(pairs),'interval_separation_scans':3,'phase_fits':results,'calibration_applied':False,'acquisition_offset_validated':False,'method':'Relative rotations between quality-accepted cloud poses three scan intervals apart; compare with integrated calibrated body gyro at the endpoints plus a candidate lag. Search +/-100 ms at 1 ms spacing, non-saturating Huber loss. Repeat with allpoint, matched-point and rotation-sensitivity-weighted mean times. Independent final movement block held out from first-block fit. Sensor extrinsics, bias and gain are not fitted.','sign':'Positive lag = body event timestamp minus cloud event timestamp; it would be subtracted from body headers only after validation.','limits':['Cloud poses are noisy and raw full scans are motion-distorted.','Scan-average effective time varies with geometry; changing its definition tests sensitivity, not hardware timing.','Pairs are correlated; profile-width ranges are not confidence intervals.','Some accepted registrations can still be incorrect despite geometric/local-consistency checks.','A boundary minimum, inconsistent repeated fits, broad minimum or failed held-out improvement prevents acceptance.']};(DEST/(a.label+'_timing.json')).write_text(json.dumps(result,indent=2)+'\n')
fig,axs=plt.subplots(2,1,figsize=(11,7),layout='constrained');good=np.array([r['accepted'] for r in rows]);t=np.array([r['time_allpoints_s'] for r in rows]);axs[0].plot(t,np.array([r['fitness'] for r in rows]),'.',ms=2,color='#aaaaaa',label='All registrations');axs[0].plot(t[good],np.array([r['fitness'] for r in rows])[good],'.',ms=2,label='Passed geometry/overlap/initialization checks');axs[0].set_ylabel('Scan/reference overlap');axs[0].set_xlabel('Recording source time (s)');axs[0].legend(fontsize=8);axs[0].set_title(a.label+' independent cloud-motion timing diagnostic')
for mode,name,score in curves:
 if mode=='allpoints':axs[1].plot(lags*1000,100*(score/score.min()-1),label=name)
axs[1].axvline(0,color='black',ls=':');axs[1].set_xlabel('Body timestamp minus effective cloud time (ms)');axs[1].set_ylabel('Loss above phase minimum (%)');axs[1].legend(fontsize=8)
for ax in axs:ax.grid(alpha=.2)
fig.savefig(DEST/(a.label+'_timing.png'),dpi=165);fig.savefig(DEST/(a.label+'_timing.pdf'));print(json.dumps(result,indent=2))
