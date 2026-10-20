"""Run the permanent live launch against passive recorded inputs at 1x."""
from pathlib import Path
import os,json,time,signal,subprocess,hashlib,traceback
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import PointCloud2,Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
assert os.environ.get('ROS_DOMAIN_ID')=='179' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
run=HERE/'production_1x';run.mkdir(exist_ok=False)
source=np.load(ROOT/'reports/bag_comparisons/return_drift_20261018_112353/source_decoded.npz')
origin=int(source['raw_stamp_ns'][0]);streams={k:[] for k in ['raw','fused','body']}
clouds={k:{} for k in ['raw','fused']};tf_rows={};parents={};maps=Counter();status={};inputs={k:[] for k in ['cloud','imu','vendor']}
status_history=[];paths=[];launch=player=None;node=None;reason='';stopped=False
def ns(stamp):return int(stamp.sec)*10**9+int(stamp.nanosec)
def pose(key,m):
    p,q=m.pose.pose.position,m.pose.pose.orientation
    streams[key].append((ns(m.header.stamp),p.x,p.y,p.z,q.x,q.y,q.z,q.w))
    expected_frame='lio_local' if key=='raw' else 'odom'
    assert m.header.frame_id==expected_frame
def cloud(key,m):
    fields={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':m.point_step})
    a=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)[::max(1,(m.width*m.height+99)//100)]
    clouds[key][ns(m.header.stamp)]=np.column_stack([a[k] for k in ['x','y','z']])
    assert m.header.frame_id==('lio_local' if key=='raw' else 'odom')
def tf(m):
    for t in m.transforms:
        parents.setdefault(t.child_frame_id,set()).add(t.header.frame_id)
        if (t.header.frame_id,t.child_frame_id) in [('odom','base_footprint'),('base_footprint','base_link')]:
            p,q=t.transform.translation,t.transform.rotation
            tf_rows.setdefault(ns(t.header.stamp),{})[t.child_frame_id]=([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
def state(key,m):
    status[key]=json.loads(m.data)
    if key=='height':status_history.append(status[key])
def map_received(key,m):
    assert m.header.frame_id=='odom';maps[key]+=1
def progress():
    return dict(source_elapsed_s=(inputs['cloud'][-1]-origin)/1e9 if inputs['cloud'] else None,
        outputs={k:len(v) for k,v in streams.items()},clouds={k:len(v) for k,v in clouds.items()},
        maps=dict(maps),height=status.get('height'),terrain=status.get('terrain'))
logs={key:(run/(key+'.log')).open('w') for key in ['launch','playback']}
try:
    rclpy.init();node=rclpy.create_node('permanent_height_observer')
    q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs=[]
    for key,topic in [('raw','/point_lio/raw/odometry'),('fused','/point_lio/odometry'),('body','/odom')]:
        subs.append(node.create_subscription(Odometry,topic,lambda m,k=key:pose(k,m),1000))
    for key,topic in [('raw','/point_lio/raw/cloud_registered'),('fused','/point_lio/cloud_registered')]:
        subs.append(node.create_subscription(PointCloud2,topic,lambda m,k=key:cloud(k,m),1000))
    subs += [node.create_subscription(PointCloud2,'/utlidar/cloud',lambda m:inputs['cloud'].append(ns(m.header.stamp)),1000),
        node.create_subscription(Imu,'/doggy/imu/body_calibrated',lambda m:inputs['imu'].append(ns(m.header.stamp)),10000),
        node.create_subscription(Odometry,'/utlidar/robot_odom',lambda m:inputs['vendor'].append(ns(m.header.stamp)),10000),
        node.create_subscription(TFMessage,'/tf',tf,1000),node.create_subscription(TFMessage,'/tf_static',tf,q)]
    for key,topic in [('height','/point_lio/height_status'),('terrain','/terrain/status')]:
        subs.append(node.create_subscription(String,topic,lambda m,k=key:state(k,m),q))
    for key,topic in [('occupancy','/map'),('walls','/walls/map')]:
        subs.append(node.create_subscription(OccupancyGrid,topic,lambda m,k=key:map_received(k,m),q))
    launch=subprocess.Popen(['ros2','launch',str(ROOT/'launch/live_2d_mapping.launch.py'),
        'rviz:=false','output_map:='+str(run/'maps/map')],stdout=logs['launch'],stderr=subprocess.STDOUT,start_new_session=True)
    end=time.monotonic()+5
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    if launch.poll() is not None:raise RuntimeError('Launch failed before playback')
    player=subprocess.Popen(['ros2','bag','play',str(HERE/'sensor_inputs'),'--rate','1',
        '--read-ahead-queue-size','3000','--delay','2','--disable-keyboard-controls','--topics',
        '/utlidar/cloud','/sportmodestate','/utlidar/robot_odom','/lf/lowstate'],
        stdout=logs['playback'],stderr=subprocess.STDOUT,start_new_session=True)
    print(json.dumps({'event':'started','launch_pid':launch.pid,'player_pid':player.pid,'rate':1}),flush=True)
    started=time.monotonic();last=0;finished=None
    while time.monotonic()-started<420:
        rclpy.spin_once(node,timeout_sec=.01);now=time.monotonic()
        if launch.poll() is not None:raise RuntimeError('Production launch exited during replay')
        if player.poll() is not None:
            if finished is None:finished=now
            complete=(len(streams['raw'])==len(streams['fused'])==len(streams['body']) and len(clouds['raw'])==len(clouds['fused']))
            if now-finished>5 and complete:reason='playback_complete';break
            if now-finished>30:reason='output_drain_timeout';break
        if now-last>30:
            p=progress();(run/'progress.json').write_text(json.dumps(p,indent=2)+'\n');print(json.dumps(p),flush=True);last=now
    if not reason:reason='wall_time_limit'
except BaseException as e:
    reason=repr(e);(run/'exception.txt').write_text(traceback.format_exc());print(reason,flush=True)
finally:
    for proc in [player,launch]:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=35)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
    if node is not None:node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    for log in logs.values():log.close()

result=progress();result.update(stop_reason=reason,robot_commands_sent=False,playback_rate=1,
    parents={k:sorted(v) for k,v in parents.items()},tf_conflicts={k:sorted(v) for k,v in parents.items() if len(v)>1})
data={k:np.array(v,dtype=float).reshape((-1,8)) for k,v in streams.items()}
stamps={k:np.array([v[0] for v in streams[k]],dtype=np.int64) for k in streams}
result['input_delivery_exact']={key:bool(np.array_equal(np.array(inputs[key],dtype=np.int64),source[s+'_stamp_ns']))
    for key,s in [('cloud','raw'),('imu','imu'),('vendor','vendor')]}
same=bool(np.array_equal(stamps['raw'],stamps['fused']))
result['all_raw_fused_pose_stamps_match']=same
result['xy_orientation_exact']=bool(same and np.array_equal(data['raw'][:,[1,2,4,5,6,7]],data['fused'][:,[1,2,4,5,6,7]]))
result['all_raw_fused_cloud_stamps_match']=clouds['raw'].keys()==clouds['fused'].keys()
result['cloud_xy_exact']=bool(result['all_raw_fused_cloud_stamps_match'] and all(np.array_equal(p[:,:2],clouds['fused'][t][:,:2],equal_nan=True) for t,p in clouds['raw'].items()))
shift={t:data['fused'][i,3]-data['raw'][i,3] for i,t in enumerate(stamps['raw'])} if same else {}
result['max_cloud_z_correction_error_m']=float(max((np.nanmax(abs((clouds['fused'][t][:,2]-p[:,2])-shift[t])) for t,p in clouds['raw'].items() if t in shift),default=0)) if result['all_raw_fused_cloud_stamps_match'] else None
result['body_stamps_match']=bool(np.array_equal(stamps['body'],stamps['fused']))
mount=np.array(json.loads((ROOT/'config/point_lio/sensor_geometry.json').read_text())['unchanged_developer_mounts']['imu']['xyz'])
if result['body_stamps_match'] and len(stamps['body']):
    expected=data['fused'][:,1:4]-Rotation.from_quat(data['fused'][:,4:8]).apply(mount)
    result['max_body_pose_mount_error_m']=float(np.max(np.linalg.norm(data['body'][:,1:4]-expected,axis=1)))
result['missing_exact_body_tf_stamps']=sum(not {'base_footprint','base_link'}.issubset(tf_rows.get(int(t),{})) for t in stamps['body'])
dt=np.diff(stamps['fused'])/1e9
result['output_rate_hz']=float(1/np.median(dt)) if len(dt) else None
result['output_gaps_over_150ms']=int((dt>.15).sum())
result['source_origin_ns']=origin
result['code_sha256']={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in
    [ROOT/'scripts/height_fusion.py',ROOT/'scripts/publish_height_fusion.py',ROOT/'launch/live_2d_mapping.launch.py',ROOT/'config/point_lio/height_fusion.yaml']}
result['passed']=bool(reason=='playback_complete' and all(result['input_delivery_exact'].values()) and same and result['xy_orientation_exact']
    and result['cloud_xy_exact'] and result['body_stamps_match'] and result.get('max_body_pose_mount_error_m',1)<1e-10
    and result['missing_exact_body_tf_stamps']==0 and not result['tf_conflicts'] and maps['occupancy']>0 and maps['walls']>0)
np.savez_compressed(run/'outputs.npz',**{key+'_stamp_ns':v for key,v in stamps.items()},
    **{key+'_values':v[:,1:] for key,v in data.items()})
(run/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
(run/'height_status.json').write_text(json.dumps(status_history,indent=2)+'\n')
print(json.dumps({'event':'finished',**result}),flush=True)
raise SystemExit(0 if result['passed'] else 1)
