"""Orchestrate two passive QoS phases; no runtime/application changes."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib,json,os,signal,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[4];BASE=Path(__file__).resolve().parent
run=BASE/datetime.now(ZoneInfo('Australia/Melbourne')).strftime('%Y%m%dT%H%M%S%z');run.mkdir();(BASE/'current_run.json').write_text(json.dumps({'directory':str(run.relative_to(ROOT))},indent=2)+'\n')
protected=['scripts/publish_body_imu.py','config/point_lio/body_imu_calibration.json','config/point_lio/go2_body_imu.yaml','scripts/publish_go2_state.py']
def hashes():return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in protected}
initial_hashes=hashes()
def network_snapshot(pids):
 snmp=Path('/proc/net/snmp').read_text().splitlines();udp={}
 for i,line in enumerate(snmp[:-1]):
  if line.startswith('Udp:'):udp=dict(zip(line.split()[1:],map(int,snmp[i+1].split()[1:])));break
 iface={}
 for name in ['rx_packets','rx_bytes','rx_errors','rx_dropped','rx_missed_errors','tx_packets','tx_errors','tx_dropped']:
  path=Path('/sys/class/net/enp7s0/statistics')/name
  if path.exists():iface[name]=int(path.read_text())
 # Per-process socket-drop counters are helpful if available. These include all
 # UDP data received by a participant; they do not identify a robot topic alone.
 tables=[]
 for file in ['/proc/net/udp','/proc/net/udp6']:
  for line in Path(file).read_text().splitlines()[1:]:
   v=line.split()
   if len(v)>12:tables.append({'inode':v[9],'local':v[1],'rx_queue_bytes':int(v[4].split(':')[1],16),'drops':int(v[-1])})
 process={}
 for pid in pids:
  inodes=set()
  try:
   for fd in Path(f'/proc/{pid}/fd').iterdir():
    try:target=os.readlink(fd)
    except OSError:continue
    if target.startswith('socket:['):inodes.add(target[8:-1])
   process[str(pid)]=[v for v in tables if v['inode'] in inodes]
  except (PermissionError,FileNotFoundError):process[str(pid)]=None
 return {'host_ns':time.time_ns(),'monotonic_ns':time.monotonic_ns(),'udp_host_global':udp,'ethernet':iface,'process_udp_sockets':process,'loadavg':os.getloadavg()}
profiles=[('be5','best_effort',5,'/sportmodestate'),('be2000','best_effort',2000,'/sportmodestate'),('reliable2000','reliable',2000,'/sportmodestate'),('reliable_reference','reliable',2000,'/sportmodestate'),('adapter_output','reliable',10000,'/doggy/imu/body_calibrated')]
old=json.loads((ROOT/'reports/point_lio_steps/step_04_timing/current_posture_recording.json').read_text());qos=ROOT/old['qos'];topics=old['topics'];phases=[]
for phase_name in ['sensor_baseline','with_recording']:
 folder=run/phase_name;folder.mkdir();recorder=None;record_log=None
 if phase_name=='with_recording':
  record_log=(folder/'record.log').open('w');recorder=subprocess.Popen(['ros2','bag','record','-s','sqlite3','--max-cache-size','67108864','--qos-profile-overrides-path',str(qos),'-o',str(folder/'bag'),*topics],stdout=record_log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL)
 observers=[];logs=[]
 for name,reliability,depth,topic in profiles:
  log=(folder/(name+'.log')).open('w');logs.append(log);proc=subprocess.Popen([sys.executable,str(BASE/'capture_observer.py'),'--name',name,'--depth',str(depth),'--reliability',reliability,'--topic',topic,'--seconds','68','--output',str(folder)],stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL);observers.append((name,proc))
 pids=[p.pid for _,p in observers]
 adapter_pid_file=ROOT/'reports/point_lio_steps/step_01_accelerometer/body_imu_adapter.pid'
 if adapter_pid_file.exists():pids.append(int(adapter_pid_file.read_text().strip()))
 before=network_snapshot(pids);(folder/'network_before.json').write_text(json.dumps(before,indent=2)+'\n');print(json.dumps({'phase':phase_name,'status':'started','pids':pids}),flush=True)
 while any(p.poll() is None for _,p in observers):
  time.sleep(1)
  if recorder is not None and recorder.poll() is not None:break
 statuses={name:p.poll() for name,p in observers}
 for _,p in observers:
  if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=10)
 after=network_snapshot(pids);(folder/'network_after.json').write_text(json.dumps(after,indent=2)+'\n')
 if recorder is not None:
  if recorder.poll() is None:recorder.send_signal(signal.SIGINT)
  try:rc=recorder.wait(timeout=15)
  except subprocess.TimeoutExpired:recorder.terminate();rc=recorder.wait(timeout=5)
  record_log.close()
 else:rc=None
 for log in logs:log.close()
 result={'phase':phase_name,'observer_returncodes':statuses,'recorder_returncode':rc,'host_global_udp_delta':{k:after['udp_host_global'][k]-v for k,v in before['udp_host_global'].items()},'ethernet_delta':{k:after['ethernet'][k]-v for k,v in before['ethernet'].items()}}
 (folder/'phase.json').write_text(json.dumps(result,indent=2)+'\n');phases.append(result);print(json.dumps(result),flush=True)
summary={'directory':str(run.relative_to(ROOT)),'phases':phases,'initial_hashes':initial_hashes,'final_hashes':hashes(),'protected_files_unchanged':initial_hashes==hashes(),'robot_commands_sent':False,'mapper_started':False}
(run/'run_manifest.json').write_text(json.dumps(summary,indent=2)+'\n');print('Finished passive QoS comparison',flush=True)
