from pathlib import Path
import json,os,signal,subprocess,sys,time,hashlib
root=Path('/home/jain/Doggy');base=root/'reports/point_lio_steps/step_04_timing/final_timing_check';info=json.loads((base/'current_motion.json').read_text());folder=root/info['directory'];log=(folder/'record.log').open('w');stopping=False
protected=['scripts/publish_body_imu.py','config/point_lio/go2_body_imu.yaml','config/point_lio/sensor_geometry.json','scripts/publish_go2_state.py']
info['application_hashes_before']={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in protected}
p=subprocess.Popen(['ros2','bag','record','-s','sqlite3','--max-cache-size','67108864','--qos-profile-overrides-path',str(root/info['qos']),'-o',str(root/info['bag']),*info['topics']],stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL)
info.update(status='recording',wrapper_pid=os.getpid(),pid=p.pid,started_host_ns=time.time_ns());
def save():
 for target in [base/'current_motion.json',folder/'recording_manifest.json']:target.write_text(json.dumps(info,indent=2)+'\n')
save();print(json.dumps(info),flush=True)
def interrupt(sig,frame):
 global stopping
 stopping=True
signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
start=time.monotonic()
while p.poll() is None and not stopping and time.monotonic()-start<300:time.sleep(.25)
info['stop_reason']='user_stop' if stopping else 'automatic_300_second_limit' if p.poll() is None else 'unexpected_early_exit'
if p.poll() is None:p.send_signal(signal.SIGINT)
try:rc=p.wait(timeout=20)
except subprocess.TimeoutExpired:p.terminate();rc=p.wait(timeout=5)
log.close();latest=json.loads((base/'current_motion.json').read_text());info.update({k:v for k,v in latest.items() if k not in info});info.update(status='stopped_pending_verification',returncode=rc,stopped_host_ns=time.time_ns(),application_hashes_after={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in protected});save();print(json.dumps({'status':info['status'],'returncode':rc,'stop_reason':info['stop_reason']}),flush=True)
