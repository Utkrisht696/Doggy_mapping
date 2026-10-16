"""Passive A/B/A delivery audit. No application mutation, controls or mapping."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib,json,os,signal,struct,subprocess,sys,time,traceback
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from unitree_go.msg import SportModeState
ROOT=Path(__file__).resolve().parents[4];BASE=Path(__file__).resolve().parent
run=BASE/datetime.now(ZoneInfo('Australia/Melbourne')).strftime('%Y%m%dT%H%M%S%z');run.mkdir();(BASE/'current_run.json').write_text(json.dumps({'directory':str(run.relative_to(ROOT))},indent=2)+'\n')
protected=['scripts/publish_body_imu.py','config/point_lio/body_imu_calibration.json','config/point_lio/go2_body_imu.yaml','scripts/publish_go2_state.py','config/point_lio/sensor_geometry.json','config/cyclonedds.xml']
def hashes():return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in protected}
initial=hashes();adapter_pid=int((ROOT/'reports/point_lio_steps/step_01_accelerometer/body_imu_adapter.pid').read_text());clk=os.sysconf('SC_CLK_TCK')
def monitor():
 result={'host_ns':time.time_ns(),'mono_ns':time.monotonic_ns(),'loadavg':os.getloadavg(),'cpu_stat':Path('/proc/stat').read_text().splitlines()[0],'processes':{}}
 for pid in [adapter_pid]+([recorder.pid] if recorder else [])+[p.pid for p in probes]:
  try:
   text=Path(f'/proc/{pid}/stat').read_text();v=text[text.rindex(')')+2:].split();result['processes'][str(pid)]={'state':v[0],'user_ticks':int(v[11]),'sys_ticks':int(v[12]),'cpu':int(v[36]),'rss_pages':int(v[21]),'schedstat':Path(f'/proc/{pid}/schedstat').read_text().strip(),'io':Path(f'/proc/{pid}/io').read_text()}
  except (OSError,IndexError):pass
 result['frequencies_khz']=[p.read_text().strip() for p in sorted(Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq'))]
 result['temperatures_millideg']={str(p):p.read_text().strip() for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')}
 result['udp']=Path('/proc/net/snmp').read_text()
 return result
rclpy.init();n=Node('delivery_light_audit',enable_rosout=False,start_parameter_services=False);q=QoSProfile(depth=4000,reliability=ReliabilityPolicy.RELIABLE)
subs=[n.create_subscription(typ,topic,lambda _:None,q,raw=True) for typ,topic in [(SportModeState,'/sportmodestate'),(Imu,'/doggy/imu/body_calibrated')]]
rows=[[],[]];verified=[0,0];phase='startup';phases=[];samples=[];probes=[];recorder=None;logs=[];errors=[]
def stop(p):
 if p.poll() is None:p.send_signal(signal.SIGINT)
 try:return p.wait(timeout=15)
 except subprocess.TimeoutExpired:p.terminate();return p.wait(timeout=5)
def pump(seconds):
 start=time.monotonic();last_monitor=0
 while time.monotonic()-start<seconds:
  any_message=False
  for k,sub in enumerate(subs):
   for _ in range(32):
    with sub.handle:item=sub.handle.take_message(sub.msg_type,sub.raw)
    if item is None:break
    any_message=True;host,mono=time.time_ns(),time.monotonic_ns();raw,info=item
    if raw[:2] not in [b'\x00\x01',b'\x00\x00']:raise RuntimeError('Unexpected CDR encoding')
    sec,ns=struct.unpack_from('<iI' if raw[1]==1 else '>iI',raw,4)
    if verified[k]<5:
     msg=deserialize_message(raw,SportModeState if k==0 else Imu);st=msg.stamp if k==0 else msg.header.stamp
     assert (st.sec,st.nanosec)==(sec,ns);verified[k]+=1
    if sec<=0 or not 0<=ns<10**9:raise RuntimeError('Invalid CDR timestamp')
    rows[k].append((sec*10**9+ns,host,mono,int(info.get('source_timestamp',0)),int(info.get('received_timestamp',0))))
  now=time.monotonic()
  if now-last_monitor>=1:
   sample=monitor();sample['phase']=phase;sample['counts']=list(map(len,rows));samples.append(sample);last_monitor=now
   (run/'status.json').write_text(json.dumps({'phase':phase,'seconds_elapsed':now-start,'counts':list(map(len,rows)),'adapter_pid':adapter_pid,'recorder_pid':recorder.pid if recorder else None,'probe_pids':[p.pid for p in probes]}))
  if recorder and recorder.poll() is not None:raise RuntimeError('Recorder exited unexpectedly')
  if not any_message:time.sleep(.0005)
try:
 print(json.dumps({'directory':str(run.relative_to(ROOT)),'adapter_pid':adapter_pid}),flush=True)
 for phase,duration in [('baseline',30),('recording_only_before',40),('recording_with_old_probes',40),('recording_only_after',40),('recovery',20)]:
  if phase=='recording_only_before':
   old=json.loads((ROOT/'reports/point_lio_steps/step_04_timing/current_posture_recording.json').read_text());log=(run/'record.log').open('w');logs.append(log)
   recorder=subprocess.Popen(['ros2','bag','record','-s','sqlite3','--max-cache-size','67108864','--qos-profile-overrides-path',str(ROOT/old['qos']),'-o',str(run/'bag'),*old['topics']],stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL)
  if phase=='recording_with_old_probes':
   for name,reliability,depth in [('load_be5','best_effort',5),('load_be2000','best_effort',2000),('load_rel2000','reliable',2000),('load_reference','reliable',2000)]:
    log=(run/(name+'.log')).open('w');logs.append(log)
    probes.append(subprocess.Popen([sys.executable,str(BASE.parent/'dropout_qos/capture_observer_metadata.py'),'--name',name,'--depth',str(depth),'--reliability',reliability,'--topic','/sportmodestate','--seconds','39','--output',str(run/'load_probes')],stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL))
  if phase=='recording_only_after':
   statuses=[stop(p) for p in probes];probes=[];(run/'probe_exit.json').write_text(json.dumps(statuses))
  if phase=='recovery':
   rc=stop(recorder);recorder=None;(run/'recorder_exit.json').write_text(json.dumps({'returncode':rc}))
  record={'phase':phase,'start_host_ns':time.time_ns(),'start_mono_ns':time.monotonic_ns(),'planned_seconds':duration};phases.append(record);print(json.dumps({'phase':phase,'status':'started'}),flush=True)
  pump(duration);record.update({'end_host_ns':time.time_ns(),'end_mono_ns':time.monotonic_ns()});print(json.dumps({'phase':phase,'status':'finished','counts':list(map(len,rows))}),flush=True)
except Exception as exc:
 errors.append(repr(exc));traceback.print_exc()
finally:
 for p in probes:stop(p)
 if recorder is not None:stop(recorder)
 for log in logs:log.close()
 for k,name in enumerate(['source','output']):
  a=np.asarray(rows[k],dtype=np.int64);np.savez_compressed(run/(name+'.npz'),stamp_ns=a[:,0],host_ns=a[:,1],mono_ns=a[:,2],dds_source_ns=a[:,3],dds_received_ns=a[:,4])
 (run/'resources.json').write_text(json.dumps(samples))
 manifest={'directory':str(run.relative_to(ROOT)),'phases':phases,'initial_hashes':initial,'final_hashes':hashes(),'protected_files_unchanged':initial==hashes(),'cdr_stamps_verified_against_deserialization':verified,'adapter_pid':adapter_pid,'clock_ticks_per_second':clk,'errors':errors,'no_application_restart':True,'robot_commands_sent':False,'mapping_or_replay_started':False,'method':'Raw serialized subscriptions sampled directly, two topics in one observer; initial five stamps per topic verified using ROS deserialization; only source stamp/host receipt/DDS metadata stored. Observer polls at 0.5 ms idle interval. All sensor processes unchanged.'}
 (run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');n.destroy_node();rclpy.shutdown()
 print(json.dumps({'status':'saved','errors':errors,'directory':str(run.relative_to(ROOT))}),flush=True)
raise SystemExit(1 if errors else 0)
