"""Measured-distance comparison and repeated-floor consistency; no active changes."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
base=Path(__file__).resolve().parent;out=base/'analysis';a=np.load(out/'comparison.npz');t=a['time'];p=a['lio_position'];ref=a['reference_position'];lr=Rotation.from_quat(a['lio_quaternion']);vr=Rotation.from_quat(a['reference_quaternion'])
windows={'start':[3,12],'far_3m':[27,32],'return_3m':[59,64],'far_5m':[81.5,84.5],'return_5m':[110,120],'late_still':[165,175]}
holds={}
for name,(start,end) in windows.items():
 m=(t>=start)&(t<=end);holds[name]={'time_window_s':[start,end],'lio_median_xyz_m':np.median(p[m],axis=0).tolist(),'vendor_median_xyz_m':np.median(ref[m],axis=0).tolist(),'lio_mean_quaternion_xyzw':lr[m].mean().as_quat().tolist(),'vendor_mean_quaternion_xyzw':vr[m].mean().as_quat().tolist(),'lio_position_p95_distance_to_median_m':float(np.percentile(np.linalg.norm(p[m]-np.median(p[m],axis=0),axis=1),95))}
checks=[]
for target,start,far,end in [(3.,'start','far_3m','return_3m'),(5.,'return_3m','far_5m','return_5m')]:
 r={'nominal_distance_m':target,'start_hold':start,'far_hold':far,'return_hold':end}
 for label,key in [('point_lio','lio'),('vendor','vendor')]:
  ps=np.array(holds[start][key+'_median_xyz_m']);pf=np.array(holds[far][key+'_median_xyz_m']);pe=np.array(holds[end][key+'_median_xyz_m']);rs=Rotation.from_quat(holds[start][key+'_mean_quaternion_xyzw']);re=Rotation.from_quat(holds[end][key+'_mean_quaternion_xyzw']);delta=pf-ps;closed=pe-ps;distance=np.linalg.norm(delta[:2]);axis=rs.apply([1,0,0]);axis[2]=0;axis/=np.linalg.norm(axis)
  r[label]={'outbound_horizontal_endpoint_distance_m':float(distance),'outbound_projected_initial_heading_m':float(delta@axis),'outbound_vertical_change_m':float(delta[2]),'nominal_distance_difference_m':float(distance-target),'nominal_distance_difference_percent':float(100*(distance-target)/target),'return_horizontal_offset_m':float(np.linalg.norm(closed[:2])),'return_vertical_offset_m':float(closed[2]),'return_3d_offset_m':float(np.linalg.norm(closed)),'return_orientation_change_deg':float(np.rad2deg((rs.inv()*re).magnitude())),'outbound_displacement_xyz_m':delta.tolist(),'return_displacement_xyz_m':closed.tolist()}
 checks.append(r)
overall={}
for label,key in [('point_lio','lio'),('vendor','vendor')]:
 delta=np.array(holds['return_5m'][key+'_median_xyz_m'])-holds['start'][key+'_median_xyz_m'];late=np.array(holds['late_still'][key+'_median_xyz_m'])-holds['return_5m'][key+'_median_xyz_m'];overall[label]={'final_horizontal_offset_from_original_start_m':float(np.linalg.norm(delta[:2])),'final_vertical_offset_from_original_start_m':float(delta[2]),'final_3d_offset_from_original_start_m':float(np.linalg.norm(delta)),'additional_late_still_translation_m':float(np.linalg.norm(late))}
# Shared floor XY cells at the original place, all recorded in the same map.
cl=np.load(base/'observed_outputs.npz');ct=(cl['cloud_stamp_ns']-cl['odom_stamp_ns'][0])/1e9;points=cl['cloud_points'];offsets=cl['cloud_offsets']
def cloudwindow(start,end):
 ix=np.flatnonzero((ct>=start)&(ct<=end));return np.concatenate([points[offsets[k]:offsets[k+1]] for k in ix]).astype(float)
initial=np.array(holds['start']['lio_median_xyz_m'])
def cells(start,end):
 v=cloudwindow(start,end);v=v[(np.linalg.norm(v[:,:2]-initial[:2],axis=1)<3)&(v[:,2]>initial[2]-.9)&(v[:,2]<initial[2]-.15)];keys=np.floor(v[:,:2]/.15).astype(np.int64);ids,inv,cnt=np.unique(keys,axis=0,return_inverse=True,return_counts=True);ix=np.argsort(inv);z=v[ix,2];endpoints=np.r_[0,np.cumsum(cnt)];return {tuple(k):float(np.median(z[endpoints[i]:endpoints[i+1]])) for i,k in enumerate(ids) if cnt[i]>=5}
floor0=cells(*windows['start']);floorchecks=[]
for name in ['return_3m','return_5m','late_still']:
 f=cells(*windows[name]);common=sorted(f.keys()&floor0.keys());diff=np.array([f[k]-floor0[k] for k in common]);floorchecks.append({'hold':name,'common_15cm_xy_cells':len(common),'median_signed_floor_height_difference_m':float(np.median(diff)),'median_absolute_floor_height_difference_m':float(np.median(abs(diff))),'p90_absolute_floor_height_difference_m':float(np.percentile(abs(diff),90)),'note':'As-recorded map; no ICP or re-levelling. Same XY cells assume adequate horizontal registration. Ground candidates within 3 m of original start, 0.15–0.9 m below original estimated IMU.'})
# Heading is measured, rather than assuming the user's intention produced no yaw.
yaw=np.unwrap(lr.as_euler('xyz')[:,2]);yaw-=np.median(yaw[(t>=3)&(t<=12)]);yaw=np.rad2deg(yaw)
report={'source_bag':str(base/'bag'),'seconds_reference':'First recorded Point-LIO pose','user_route':'3 m forward/back, then 5 m forward/back; curved paths, no turn-around, attempted same heading','measurement_limitations':['Reported distances are nominal references; tape/marker method and measurement uncertainty were not provided.','Endpoint displacement differs from travelled path length on a curved route. We compare horizontal hold-to-hold displacement, not noisy integrated speed.','Physical position/heading placement at each return is not independently recorded, so return offsets combine real placement error with estimator error.','Vendor odometry shares body IMU information and is not ground truth. Both estimates refer to the IMU using the developer body->IMU lever arm.'],'holds':holds,'distance_and_return_checks':checks,'overall_return_check':overall,'revisited_start_floor_checks':floorchecks,'lio_yaw_range_relative_to_initial_deg':[float(yaw.min()),float(yaw.max())],'mapping_settings_changed':False}
(out/'distance_checks.json').write_text(json.dumps(report,indent=2)+'\n')
fig,axs=plt.subplots(2,2,figsize=(13,9),layout='constrained');fig.suptitle('3 m and 5 m out-and-back: Point-LIO vs vendor odometry',fontsize=14)
basep=np.array(holds['start']['lio_median_xyz_m']);baser=np.array(holds['start']['vendor_median_xyz_m']);lp=p-basep;rp=ref-baser
axs[0,0].plot(t,lp[:,0],label='Point-LIO forward X');axs[0,0].plot(t,rp[:,0],'--',label='Vendor forward X');axs[0,0].axhline(3,color='grey',ls=':',label='Nominal 3 m');axs[0,0].axhline(5,color='black',ls=':',label='Nominal 5 m');axs[0,0].set(xlabel='Seconds',ylabel='Forward displacement (m)',title='Out-and-back distances');axs[0,0].legend(fontsize=8)
axs[0,1].plot(lp[:,0],lp[:,1],label='Point-LIO');axs[0,1].plot(rp[:,0],rp[:,1],'--',label='Vendor, centred on its own start');axs[0,1].set(xlabel='Forward X (m)',ylabel='Lateral Y (m)',title='Curved paths; robot reverses without turning');axs[0,1].set_aspect('equal',adjustable='datalim');axs[0,1].legend(fontsize=8)
axs[1,0].plot(t,lp[:,2]*100,label='Point-LIO IMU');axs[1,0].plot(t,rp[:,2]*100,'--',label='Vendor IMU');axs[1,0].set(xlabel='Seconds',ylabel='Height change (cm)',title='Height returns near the initial value');axs[1,0].legend(fontsize=8)
labels=['3 m return','5 m return','Late still'];sign=[f['median_signed_floor_height_difference_m']*100 for f in floorchecks];absolute=[f['median_absolute_floor_height_difference_m']*100 for f in floorchecks];x=np.arange(3);axs[1,1].bar(x-.18,sign,width=.35,label='Median signed floor difference');axs[1,1].bar(x+.18,absolute,width=.35,label='Median absolute floor difference');axs[1,1].set_xticks(x,labels);axs[1,1].set(ylabel='Floor difference (cm)',title='Same start-area floor patches, no extra alignment');axs[1,1].legend(fontsize=8)
for ax in axs.ravel():ax.grid(alpha=.25)
for ax in [axs[0,0],axs[1,0]]:
 for n,(s,e) in windows.items():
  if n=='late_still':continue
  ax.axvspan(s,e,color='green',alpha=.08)
fig.savefig(out/'distance_comparison.png',dpi=165);fig.savefig(out/'distance_comparison.pdf');plt.close(fig)
print(json.dumps({'checks':checks,'overall':overall,'floor':floorchecks,'yaw_range':report['lio_yaw_range_relative_to_initial_deg']},indent=2))
