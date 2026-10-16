"""Offline analysis of the passive A/B/A audit. Does not read live bag files."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3];run=ROOT/json.loads((BASE/'current_run.json').read_text())['directory'];manifest=json.loads((run/'manifest.json').read_text());resource=json.loads((run/'resources.json').read_text());source=np.load(run/'source.npz');output=np.load(run/'output.npz')
st,si=np.unique(source['stamp_ns'],return_index=True);ot,oi=np.unique(output['stamp_ns'],return_index=True);common,six,oix=np.intersect1d(st,ot,return_indices=True);sh=source['host_ns'][si][six];oh=output['host_ns'][oi][oix];pub=output['dds_source_ns'][oi][oix];sm=source['mono_ns'][si][six];om=output['mono_ns'][oi][oix]
def stats(a):
 return {k:float(v) for k,v in zip(['min','median','p95','p99','max'],np.percentile(a,[0,50,95,99,100]))} if len(a) else None
phases=[];pid=str(manifest['adapter_pid']);hz=manifest['clock_ticks_per_second'];metric_rows=[]
for prev,cur in zip(resource,resource[1:]):
 if prev['phase']!=cur['phase']:continue
 dt=(cur['mono_ns']-prev['mono_ns'])/1e9;v={'phase':cur['phase'],'mono_ns':cur['mono_ns'],'cpu_percent':None,'runnable_wait_ms':None}
 if pid in prev['processes'] and pid in cur['processes']:
  a,b=prev['processes'][pid],cur['processes'][pid];v['cpu_percent']=100*((b['user_ticks']+b['sys_ticks'])-(a['user_ticks']+a['sys_ticks']))/hz/dt
  old=list(map(int,a['schedstat'].split()));new=list(map(int,b['schedstat'].split()));v['runnable_wait_ms']=(new[1]-old[1])/1e6
 av=list(map(int,prev['cpu_stat'].split()[1:]));bv=list(map(int,cur['cpu_stat'].split()[1:]));d=np.array(bv[:8])-np.array(av[:8]);v['host_cpu_busy_percent']=100*(1-(d[3]+d[4])/sum(d));metric_rows.append(v)
for phase in manifest['phases']:
 if 'end_mono_ns' not in phase:continue
 lo,hi=phase['start_mono_ns']+5*10**9,phase['end_mono_ns']-3*10**9;sel=(sm>=lo)&(sm<=hi);sst=st[(source['mono_ns'][si]>=lo)&(source['mono_ns'][si]<=hi)];missing=np.setdiff1d(sst,ot);m=[v for v in metric_rows if v['phase']==phase['phase']]
 delay=(om[sel]-sm[sel])/1e6;pre=(pub[sel]-sh[sel])/1e6;post=(oh[sel]-pub[sel])/1e6
 phases.append({'phase':phase['phase'],'analysis_interior_seconds':(hi-lo)/1e9,'observed_source_unique_samples':len(sst),'matched_output_samples':int(sel.sum()),'missing_output_timestamps':len(missing),'source_monotonic_backsteps':int(np.sum(np.diff(source['stamp_ns'][(source['mono_ns']>=lo)&(source['mono_ns']<=hi)])<0)),'output_relative_to_source_receipt_ms':stats(delay),'adapter_dds_write_relative_to_source_receipt_ms':stats(pre),'output_receipt_after_dds_write_ms':stats(post),'samples_delay_over_20ms':int(np.sum(delay>20)),'samples_delay_over_100ms':int(np.sum(delay>100)),'adapter_cpu_percent_one_core':stats([v['cpu_percent'] for v in m if v['cpu_percent'] is not None]),'host_cpu_busy_percent':stats([v['host_cpu_busy_percent'] for v in m]),'adapter_main_thread_runnable_wait_ms_per_second':stats([v['runnable_wait_ms'] for v in m if v['runnable_wait_ms'] is not None])})
def udp(text):
 lines=text.splitlines()
 for i,line in enumerate(lines[:-1]):
  if line.startswith('Udp:'):return dict(zip(line.split()[1:],map(int,lines[i+1].split()[1:])))
 return {}
a,b=udp(resource[0]['udp']),udp(resource[-1]['udp']);network={k:b[k]-v for k,v in a.items()}
result={'directory':str(run.relative_to(ROOT)),'valid':not manifest['errors'] and manifest['protected_files_unchanged'] and manifest['cdr_stamps_verified_against_deserialization']==[5,5] and all(p['matched_output_samples']>3000 for p in phases),'phases':phases,'network_global_udp_delta':network,'cdr_stamps_verified_against_deserialization':manifest['cdr_stamps_verified_against_deserialization'],'protected_files_unchanged':manifest['protected_files_unchanged'],'limits':['One continuous A/B/A trial; phase ordering, startup and host workload may confound causality.','Only observed source messages are the reference, not all samples produced inside robot.','Raw observer polls both subscriptions and reads source first; sub-millisecond relative timing can be biased by polling/order.','DDS output source_timestamp is host write time, not sensor acquisition time.','No physical clock-offset or LiDAR-IMU calibration inferred.','No application code change, process restart, robot motion command or mapping was performed.']}
(run/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
start=manifest['phases'][0]['start_mono_ns'];t=(sm-start)/1e9;delay=(om-sm)/1e6;pre=(pub-sh)/1e6
valid_plot=np.zeros(len(t),dtype=bool)
for phase in manifest['phases']:
 if 'end_mono_ns' in phase:valid_plot |= (sm>=phase['start_mono_ns']+5*10**9)&(sm<=phase['end_mono_ns']-3*10**9)
bucket=np.floor(t*10).astype(int);indices=np.arange(int(bucket.min()),int(bucket.max())+1);bt=indices/10.0;bd=np.array([np.max(delay[(bucket==k)&valid_plot]) if np.any((bucket==k)&valid_plot) else np.nan for k in indices]);bp=np.array([np.max(pre[(bucket==k)&valid_plot]) if np.any((bucket==k)&valid_plot) else np.nan for k in indices])
fig,axes=plt.subplots(2,1,figsize=(12,6.8),sharex=True,layout='constrained');axes[0].plot(bt,bd,label='Output received minus source received',lw=1);axes[0].plot(bt,bp,label='Adapter DDS write minus source received',lw=.8,alpha=.8);axes[0].set_ylabel('Relative delay (ms)');fig.suptitle('Unchanged adapter: recording and subscriber load; transition periods excluded',fontsize=12);axes[0].legend(loc='upper right',fontsize=8)
mt=np.array([(r['mono_ns']-start)/1e9 for r in metric_rows]);axes[1].plot(mt,[r['cpu_percent'] if r['cpu_percent'] is not None else np.nan for r in metric_rows],label='Adapter CPU (100% = one logical core)');axes[1].plot(mt,[r['host_cpu_busy_percent'] for r in metric_rows],label='Whole laptop CPU busy (%)',alpha=.8);axes[1].set_ylabel('CPU (%)');axes[1].set_xlabel('Elapsed host monotonic time (s)');axes[1].legend(loc='upper right',fontsize=8)
colors=['#dae8f5','#e4f1df','#ffd9ce','#e4f1df','#dae8f5']
for phase,color in zip(manifest['phases'],colors):
 if 'end_mono_ns' not in phase:continue
 lo,hi=(phase['start_mono_ns']-start)/1e9,(phase['end_mono_ns']-start)/1e9
 for ax in axes:ax.axvspan(lo,hi,color=color,alpha=.4,zorder=-1);ax.grid(alpha=.2)
 axes[0].text((lo+hi)/2,1.015,{'baseline':'Baseline','recording_only_before':'Recording only','recording_with_old_probes':'Recording + four probes','recording_only_after':'Recording only','recovery':'Recovery'}[phase['phase']],transform=axes[0].get_xaxis_transform(),ha='center',va='bottom',fontsize=7)
fig.savefig(run/'delivery_delay.png',dpi=160);fig.savefig(run/'delivery_delay.pdf');print(json.dumps(result,indent=2))
