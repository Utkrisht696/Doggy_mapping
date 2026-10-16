"""Offline comparison of observed source stamps; no active code/settings edits."""
from pathlib import Path
import json,numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3];run=ROOT/json.loads((BASE/'current_run.json').read_text())['directory'];manifest=json.loads((run/'run_manifest.json').read_text());names=['be5','be2000','reliable2000','reliable_reference','adapter_output'];cal=json.loads((ROOT/'config/point_lio/body_imu_calibration.json').read_text());bias=np.array(cal['acc_bias_xyz_m_s2']+cal['gyro_bias_xyz_rad_s'])
def stats(a):
 a=np.asarray(a,dtype=float)
 return {'min':float(a.min()),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'p99':float(np.percentile(a,99)),'max':float(a.max())} if len(a) else None
phases=[]
for phase in ['sensor_baseline','with_recording']:
 folder=run/phase;datasets={name:np.load(folder/(name+'.npz')) for name in names};lo=max(int(v['stamp_ns'].min()) for v in datasets.values())+10**9;hi=min(int(v['stamp_ns'].max()) for v in datasets.values())-10**9
 unique={};sets={};counts={}
 for name,data in datasets.items():
  sel=(data['stamp_ns']>=lo)&(data['stamp_ns']<=hi);index=np.flatnonzero(sel);t,ix=np.unique(data['stamp_ns'][sel],return_index=True);ix=index[ix];unique[name]={'stamp':t,'mono':data['mono_ns'][ix],'values':data['values'][ix]};sets[name]=set(map(int,t));counts[name]=int(sel.sum())
 union=set.union(*(sets[n] for n in names[:-1]));union_array=np.array(sorted(union),dtype=np.int64);common=set.intersection(*(sets[n] for n in names[:-1]));earliest={}
 for name in names[:-1]:
  for stamp,mono in zip(unique[name]['stamp'],unique[name]['mono']):earliest[int(stamp)]=min(earliest.get(int(stamp),int(mono)),int(mono))
 table={}
 for name in names:
  u=unique[name];t=u['stamp'];delays=[(int(m)-earliest[int(s)])/1e6 for s,m in zip(t,u['mono']) if int(s) in earliest]
  table[name]={'messages':counts[name],'unique_stamps':len(t),'duplicate_stamps':counts[name]-len(t),'missing_against_observed_source_union':len(union-sets[name]),'extra_not_observed_by_source_subscribers':len(sets[name]-union),'backward_stamps':int(np.sum(np.diff(datasets[name]['stamp_ns'])<0)),'source_gap_ms':stats(np.diff(t)/1e6),'callback_delay_from_first_source_observer_ms':stats(delays)}
 paired={}
 for other in ['be2000','reliable2000','reliable_reference','adapter_output']:
  shared,aa,bb=np.intersect1d(unique['be5']['stamp'],unique[other]['stamp'],return_indices=True)
  paired[other+'_minus_be5_callback_ms']=stats((unique[other]['mono'][bb]-unique['be5']['mono'][aa])/1e6)
 # Use all recorded reliable-reference packets and first observations for math.
 shared,oi,ri=np.intersect1d(unique['adapter_output']['stamp'],unique['reliable_reference']['stamp'],return_indices=True)
 err=float(np.max(np.abs(unique['adapter_output']['values'][oi,:6]-(unique['reliable_reference']['values'][ri,:6]-bias))))
 before=json.loads((folder/'network_before.json').read_text());after=json.loads((folder/'network_after.json').read_text());net=json.loads((folder/'phase.json').read_text());adapter_keys=[pid for pid in before['process_udp_sockets'] if pid not in {str(json.loads((folder/(n+'.json')).read_text())['pid']) for n in names}]
 adapter_drops={}
 for pid in adapter_keys:
  b={x['inode']:x for x in (before['process_udp_sockets'][pid] or [])};a={x['inode']:x for x in (after['process_udp_sockets'].get(pid) or [])}
  adapter_drops[pid]={inode:a[inode]['drops']-v['drops'] for inode,v in b.items() if inode in a}
 gaps=[{'source_s_from_first':float((union_array[i]-union_array[0])/1e9),'gap_ms':float((union_array[i+1]-union_array[i])/1e6)} for i in np.flatnonzero(np.diff(union_array)>12_000_000)]
 phases.append({'phase':phase,'shared_source_interval_s':(hi-lo)/1e9,'source_union_unique_stamps':len(union),'source_intersection_unique_stamps':len(common),'profiles':table,'paired_delays':paired,'matched_adapter_reference_packets':len(shared),'max_acc_gyro_fixed_bias_comparison_error':err,'gaps_over_12ms_present_in_all_observers':gaps,'network':net,'existing_adapter_socket_drop_deltas':adapter_drops})
checks={'all_observers_completed':all(all(rc==0 for rc in p['observer_returncodes'].values()) for p in manifest['phases']),'recording_load_exited_cleanly':manifest['phases'][1]['recorder_returncode']==0,'active_code_and_configs_unchanged':manifest['protected_files_unchanged'],'all_profiles_received_over_15000_unique_stamps':all(all(v['unique_stamps']>15000 for v in p['profiles'].values()) for p in phases),'adapter_preserves_bias_math':all(p['max_acc_gyro_fixed_bias_comparison_error']==0 for p in phases),'no_source_clock_backsteps':all(all(v['backward_stamps']==0 for v in p['profiles'].values()) for p in phases)}
result={'comparison_valid':all(checks.values()),'checks':checks,'phases':phases,'conclusions':[('Host UDP receive-buffer overflows observed; check phase counters and per-socket attribution.' if any(p['network']['host_global_udp_delta']['RcvbufErrors']>0 for p in phases) else 'No host UDP receive-buffer overflows observed during either phase; check per-socket deltas as well.'),'These are real laptop-side drops, but per-socket DDS counters include all received topics and cannot equate each dropped datagram with a lost SportModeState sample.','Deeper best-effort application queues alone do not eliminate kernel socket drops.',('Reliable subscribers still miss some observed source timestamps; compare losses and callback-delay tails.' if any(p['profiles'][n]['missing_against_observed_source_union']>0 for p in phases for n in ['reliable2000','reliable_reference']) else 'Both reliable source subscribers received every source-union timestamp in these phases; callback-delay tails remain a separate consideration.'),'No reliable source packet sequence counter was available; source-stamp union is an observed reference, not every packet the robot generated.','Common observed source-stamp gaps may be upstream publishing, shared transport loss or stamp-generation behavior; cannot assign every gap to robot firmware.'],'application_changes_applied':False}
(run/'comparison.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
