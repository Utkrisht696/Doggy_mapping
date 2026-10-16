"""Verify the bag only after recorder shutdown; no live database access."""
from pathlib import Path
import json,sqlite3,yaml
ROOT=Path('/home/jain/Doggy');BASE=Path(__file__).resolve().parent;path=BASE/'current_recording.json';info=json.loads(path.read_text());assert info['status']=='stopped_pending_verification',info['status'];folder=ROOT/info['directory'];bag=folder/'bag';meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];counts={};integrity=[];spans={}
for f in bag.glob('*.db3'):
 db=sqlite3.connect(f'file:{f.resolve()}?mode=ro&immutable=1',uri=True);integrity.append(db.execute('PRAGMA quick_check').fetchone()[0]);rows=db.execute('SELECT t.name,COUNT(m.id),MIN(m.timestamp),MAX(m.timestamp) FROM topics t LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id').fetchall()
 for name,count,lo,hi in rows:counts[name]=count;spans[name]=(hi-lo)/1e9 if lo is not None else 0
 db.close()
log=(folder/'record.log').read_text();checks={'sqlite_integrity_ok':bool(integrity) and all(v=='ok' for v in integrity),'clean_recorder_exit':info['returncode'] in (0,-2) and 'Recording stopped' in log and 'Writing remaining messages' in log,'all_13_topics_saved':set(counts)==set(info['topics']) and all(counts.values()),'metadata_message_counts_match':sum(counts.values())==meta['message_count'],'no_recorder_errors':'[ERROR]' not in log,'protected_application_files_unchanged':info['application_hashes_before']==info['application_hashes_after']}
def udp(text):
 lines=text.splitlines()
 for i,line in enumerate(lines[:-1]):
  if line.startswith('Udp:'):return dict(zip(line.split()[1:],map(int,lines[i+1].split()[1:])))
 return {}
a,b=udp(info['network_before']['snmp']),udp(info['network_after']['snmp']);network={k:b[k]-v for k,v in a.items()}
v={'passed':all(checks.values()),'checks':checks,'topic_counts':counts,'topic_duration_s':spans,'host_global_udp_delta':network,'ethernet_delta':{k:int(info['network_after']['ethernet'][k])-int(v) for k,v in info['network_before']['ethernet'].items()},'power_profile_before':info['power_profile_before'],'power_profile_after':info['power_profile_after'],'no_live_database_opened':True};(folder/'recording_verification.json').write_text(json.dumps(v,indent=2)+'\n');assert v['passed'],v
info.update(status='stopped_saved',metadata_message_count_matches=True,stored_topic_counts=counts,stored_receipt_start_ns=meta['starting_time']['nanoseconds_since_epoch'],stored_receipt_end_ns=meta['starting_time']['nanoseconds_since_epoch']+meta['duration']['nanoseconds']);path.write_text(json.dumps(info,indent=2)+'\n');(folder/'recording_manifest.json').write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(v,indent=2))
