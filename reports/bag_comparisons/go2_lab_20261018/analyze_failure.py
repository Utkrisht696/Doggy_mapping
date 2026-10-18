"""Locate relative attitude divergence; vendor used as a reference, not truth."""
from pathlib import Path
import numpy as np,json
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(__file__).resolve().parent;a=np.load(r/'outputs.npz');s=np.load(r/'source_decoded.npz');origin=int(s['raw_stamp_ns'][0]);t=(a['odom_stamp_ns']-origin)/1e9;vt=(s['vendor_stamp_ns']-origin)/1e9;lp=a['odom_values'][:,:3];lr=Rotation.from_quat(a['odom_values'][:,3:7]);v=s['vendor_values'];vr=Slerp(vt,Rotation.from_quat(v[:,3:7]))(t);vp=np.column_stack([np.interp(t,vt,v[:,j]) for j in range(3)])+vr.apply([-.02557,0,.04232]);start=(t>=2.6)&(t<=3.6);align=lr[start].mean()*vr[start].mean().inv();refrot=align*vr;offset=np.median(lp[start],axis=0)-align.apply(np.median(vp[start],axis=0));ref=align.apply(vp)+offset;err=np.rad2deg((refrot.inv()*lr).magnitude());dt=np.diff(t);gaps=np.flatnonzero(dt>.15);report={'gaps':[],'timeline':[],'note':'One initial rigid orientation/translation alignment. Vendor not ground truth; orientation divergence at recording gap is the diagnostic observation.'}
for ix in gaps:
 drl=lr[ix].inv()*lr[ix+1];drv=vr[ix].inv()*vr[ix+1];report['gaps'].append({'before_s':float(t[ix]),'after_s':float(t[ix+1]),'lio_rotation_change_deg':float(drl.magnitude()*180/np.pi),'vendor_rotation_change_deg':float(drv.magnitude()*180/np.pi),'relative_rotation_disagreement_deg':float(((drv.inv()*drl).magnitude())*180/np.pi),'orientation_disagreement_before_deg':float(err[ix]),'orientation_disagreement_after_deg':float(err[ix+1]),'lio_translation_m':float(np.linalg.norm(lp[ix+1]-lp[ix])),'vendor_translation_m':float(np.linalg.norm(vp[ix+1]-vp[ix]))})
for sec in [3,10,20,27,28,30,35,40,44,49,50,55,60,70,90,120,150,180,210,216]:
 ix=np.argmin(abs(t-sec));report['timeline'].append({'source_s':float(t[ix]),'position_disagreement_m':float(np.linalg.norm(lp[ix]-ref[ix])),'orientation_disagreement_deg':float(err[ix]),'lio_position_m':lp[ix].tolist()})
(r/'analysis/failure_timeline.json').write_text(json.dumps(report,indent=2)+'\n');fig,ax=plt.subplots(2,1,figsize=(12,8),layout='constrained');fig.suptitle('Main divergence appears across the four-second recording gap',fontsize=14);errplot=err.copy();errplot[np.r_[False,dt>.15]]=np.nan;ax[0].plot(t,errplot,label='Point-LIO / vendor orientation disagreement');ax[0].set(ylabel='Orientation disagreement (degrees)',title='One initial alignment; vendor is a reference estimate');ax[0].legend();ax[1].plot(t,np.linalg.norm(lp-ref,axis=1));ax[1].set(xlabel='Seconds after first raw cloud header',ylabel='3D position disagreement (m)',title='Different estimated trajectories after the gap')
for aa in ax:
 aa.grid(alpha=.25)
 for gap in report['gaps']:aa.axvspan(gap['before_s'],gap['after_s'],color='red',alpha=.18)
fig.savefig(r/'analysis/failure_timeline.png',dpi=160);fig.savefig(r/'analysis/failure_timeline.pdf');plt.close(fig);print(json.dumps(report['gaps'],indent=2))
