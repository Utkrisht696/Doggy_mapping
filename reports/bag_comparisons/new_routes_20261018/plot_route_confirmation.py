from pathlib import Path
import numpy as np,json,sys
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(sys.argv[1]);s=np.load(r/'source_decoded.npz');a=np.load(r/'run_01/outputs.npz');o=int(s['raw_stamp_ns'][0]);t=(a['odom_stamp_ns']-o)/1e9;vt=(s['vendor_stamp_ns']-o)/1e9;v=s['vendor_values'];q=Rotation.from_quat(a['odom_values'][:,3:]);vr=Slerp(vt,Rotation.from_quat(v[:,3:7]))(t);vp=np.column_stack([np.interp(t,vt,v[:,j]) for j in range(3)])+vr.apply([-.02557,0,.04232]);holds=json.loads((r/'candidate_holds.json').read_text())['holds'];first=(t>1.5)&(t<5.5);lp=Rotation.from_euler('z',-q[first].mean().as_euler('xyz')[2]).apply(a['odom_values'][:,:3]-np.median(a['odom_values'][first,:3],axis=0));rp=Rotation.from_euler('z',-vr[first].mean().as_euler('xyz')[2]).apply(vp-np.median(vp[first],axis=0));fig,axs=plt.subplots(1,2,figsize=(14,6),layout='constrained');fig.suptitle(r.name+': route confirmation (estimated positions, not ground truth)')
segments=[(0,46,'5 m out/back'),(46,96,'10 m out/back'),(96,129,'Travel to rectangle area'),(129,160,'Rectangle A?'),(160,202,'Move/turn and rectangle B?'),(202,226,'Small rectangle?'),(226,t[-1],'Return leg?')]
for k,(begin,end,label) in enumerate(segments):
 m=(t>=begin)&(t<=end);axs[0].plot(lp[m,0],lp[m,1],label=label,lw=1.7)
axs[0].scatter(0,0,c='black',marker='*',s=130,label='Original start');axs[0].scatter(*lp[-1,:2],c='red',marker='X',s=90,label='Recorded finish');axs[0].set(xlabel='Initial heading X (m)',ylabel='Y (m)',title='Point-LIO trace: confirm the motion, not the numerical error');axs[0].set_aspect('equal',adjustable='datalim');axs[0].legend(fontsize=8)
axs[1].plot(t,lp[:,0],label='Point-LIO forward position');axs[1].plot(t,rp[:,0],'--',label='Vendor forward position');axs[1].set(xlim=(0,110),xlabel='Time from first LiDAR stamp (s)',ylabel='Forward position relative to start (m)',title='First two straight tests')
for h in holds[:5]:
 begin,end=h['source_window_s'];m=(t>=begin)&(t<=end);x=(begin+end)/2;y=np.median(lp[m,0]);axs[1].axvspan(begin,end,alpha=.1,color='green');axs[1].annotate(['Start','5 m far end','5 m return','10 m far end','10 m return'][h['index']],(x,y),xytext=(0,14),textcoords='offset points',ha='center',fontsize=8)
axs[1].legend(fontsize=8)
for ax in axs:ax.grid(alpha=.25)
fig.savefig(r/'route_confirmation.png',dpi=150);plt.close(fig)
