"""Offline saved-output analysis; vendor estimates are comparison, not truth."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation,Slerp
base=Path('reports/point_lio_steps/step_06_saved_bag');run=Path(json.loads((base/'current_run.json').read_text())['directory']);a=np.load(run/'outputs.npz');source=Path('reports/point_lio_steps/step_04_timing/temporal_motion/20261016T205503+1100');s=np.load(source/'decoded_samples.npz');origin=int(s['cloud_stamp_ns'][0]);t=(a['odom_stamp_ns']-origin)/1e9;pose=a['odom_values'];result=json.loads((run/'result.json').read_text());metrics={'run':str(run),'stop_reason':result['stop_reason'],'source_interval_s':[float(t[0]),float(t[-1])],'odometry_messages':len(t),'registered_clouds':len(a['cloud_stamp_ns']),'estimator_tf_stamps_exact_match':bool(np.array_equal(a['odom_stamp_ns'],a['map_odom_stamp_ns'])),'multiple_tf_parents':result['multiple_tf_parents'],'default_profile_changed':not result['application_unchanged'],'gyro_variance':.1,'init_map_size':4000,'acquisition_lag_validated':False}
q=pose[:,3:7];q=q/np.linalg.norm(q,axis=1)[:,None];rots=Rotation.from_quat(q)
for label,start,end in [('initial_still',5,55),('final_still',220,250)]:
 mask=(t>=start)&(t<=end)
 if mask.sum()>2:
  p=pose[mask,:3];qr=q[mask];ref=np.median(p[:min(30,len(p))],axis=0)
  metrics[label]={'source_interval_s':[start,end],'samples':int(mask.sum()),'max_position_excursion_from_reference_m':float(np.linalg.norm(p-ref,axis=1).max()),'max_orientation_change_from_first_deg':float(np.max(2*np.arccos(np.clip(abs(qr@qr[0]),0,1))*180/np.pi)),'position_start_to_end_m':float(np.linalg.norm(p[-1]-p[0]))}
for label,fullkey,outkey in [('imu','body_stamp_ns','input_imu_stamp_ns'),('cloud','cloud_stamp_ns','input_cloud_stamp_ns')]:
 received=a[outkey];expected=s[fullkey];expected=expected[(expected>=received[0])&(expected<=received[-1])];metrics[label+'_missing_in_observed_stamp_interval']=int(len(np.setdiff1d(expected,received)))
# Align vendor body->IMU poses once, at the initial LIO pose. No trajectory fit.
odom_stamp=s['odom_stamp_ns'];odom=s['odom_values'];valid=np.isfinite(odom[:,:7]).all(axis=1)&(np.linalg.norm(odom[:,3:7],axis=1)>1e-6);odom_stamp=odom_stamp[valid];odom=odom[valid];unique,indices=np.unique(odom_stamp,return_index=True);odom=odom[indices];ot=(unique-origin)/1e9;good=(t>=ot[0])&(t<=ot[-1]);rt=t[good];qref=odom[:,[4,5,6,3]];rref=Slerp(ot,Rotation.from_quat(qref))(rt);pref=np.column_stack([np.interp(rt,ot,odom[:,j]) for j in range(3)]);mount=np.array([-.02557,0,.04232]);pref+=rref.apply(mount);align_rot=rots[good][0]*rref[0].inv();align_translation=pose[good,:3][0]-align_rot.apply(pref[0]);expected=align_rot.apply(pref)+align_translation;position_difference=np.linalg.norm(pose[good,:3]-expected,axis=1);angle_difference=(rref.inv()*align_rot.inv()*rots[good]).magnitude()*180/np.pi
metrics['vendor_comparison']={'not_external_ground_truth':True,'alignment':'One initial rigid transform, including developer body->IMU lever arm; no trajectory optimization','position_difference_median_m':float(np.median(position_difference)),'position_difference_p95_m':float(np.percentile(position_difference,95)),'position_difference_max_m':float(position_difference.max()),'orientation_difference_p95_deg':float(np.percentile(angle_difference,95)),'orientation_difference_max_deg':float(angle_difference.max())}
np.savez_compressed(run/'reference_comparison.npz',time=rt,lio_position=pose[good,:3],reference_position=expected,position_difference=position_difference,orientation_difference_deg=angle_difference)
comparisons=[];fig,axes=plt.subplots(2,2,figsize=(13,9));fig.suptitle('Point-LIO saved-bag trial: denser initial map (4000 points)',fontsize=14)
for directory,label in [('20261016T213543+1100','Original seed 10'),('20261016T214343+1100','Gyro-only trial'),(run.name,'Seed 4000, original gyro')]:
 b=np.load(base/directory/'outputs.npz');bt=(b['odom_stamp_ns']-origin)/1e9;mask=bt<=13.75;bp=b['odom_values'][mask,:3];values=np.linalg.norm(bp-bp[0],axis=1);axes[0,0].plot(bt[mask],values,label=label);comparisons.append({'label':label,'max_position_displacement_first_13_75s_m':float(values.max())})
axes[0,0].set(xlabel='Recording time (s)',ylabel='Displacement from initial estimate (m)',title='Same initial stationary segment');axes[0,0].legend();axes[0,0].grid(alpha=.3)
axes[0,1].plot(pose[:,0],pose[:,1],label='Point-LIO');axes[0,1].plot(expected[:,0],expected[:,1],linestyle='--',label='Initially aligned vendor reference');axes[0,1].scatter(pose[0,0],pose[0,1],c='green',s=35,label='Start');axes[0,1].scatter(pose[-1,0],pose[-1,1],c='red',s=35,label='End');axes[0,1].set(xlabel='Map X (m)',ylabel='Map Y (m)',title='Body-IMU trajectory');axes[0,1].axis('equal');axes[0,1].legend(fontsize=8);axes[0,1].grid(alpha=.3)
axes[1,0].plot(rt,position_difference,label='Position difference');axes[1,0].set(xlabel='Recording time (s)',ylabel='Difference (m)',title='Difference from vendor estimate (not ground truth)');axes[1,0].grid(alpha=.3)
points=a['cloud_points'];stride=max(1,(len(points)+119999)//120000);view=points[::stride];axes[1,1].scatter(view[:,0],view[:,1],c=view[:,2],s=.3,cmap='viridis',alpha=.4);axes[1,1].plot(pose[:,0],pose[:,1],c='red',lw=.8);axes[1,1].set(xlabel='Map X (m)',ylabel='Map Y (m)',title='Sampled registered scans accumulated for diagnosis');axes[1,1].axis('equal');axes[1,1].grid(alpha=.2);fig.tight_layout();fig.savefig(run/'mapping_comparison.png',dpi=160);fig.savefig(run/'mapping_comparison.pdf');plt.close(fig)
metrics['initial_segment_comparisons']=comparisons
# Diagnostic point artifact: output clouds were subsampled by the observer.
keys=np.floor(points/.05).astype(np.int64);_,index=np.unique(keys,axis=0,return_index=True);export=points[index];ply=run/'registered_scans_sampled.ply'
with ply.open('wb') as f:
 header=f'ply\nformat binary_little_endian 1.0\ncomment Diagnostic accumulation of sampled registered output; not a full internal map\nelement vertex {len(export)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n';f.write(header.encode());f.write(export.astype('<f4').tobytes())
metrics['diagnostic_accumulation_points']=len(export);metrics['diagnostic_accumulation_is_full_internal_map']=False
(run/'analysis.json').write_text(json.dumps(metrics,indent=2)+'\n');print(json.dumps(metrics,indent=2))
