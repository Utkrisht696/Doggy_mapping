"""Short smoke check of the production recorded-output replay launch."""
from pathlib import Path
import os,signal,time,json,subprocess
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy

ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
assert os.environ.get('ROS_DOMAIN_ID')=='180' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
rclpy.init();node=rclpy.create_node('canonical_output_replay_check');seen={};poses={'raw':{},'fused':{}};clouds=0
def pose(key,m):
    p,q=m.pose.pose.position,m.pose.pose.orientation
    assert m.header.frame_id==('lio_local' if key=='raw' else 'odom')
    poses[key][(m.header.stamp.sec,m.header.stamp.nanosec)]=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
def cloud(m):
    global clouds
    assert m.header.frame_id=='odom';clouds+=1
subs=[node.create_subscription(Odometry,'/point_lio/raw/odometry',lambda m:pose('raw',m),1000),
      node.create_subscription(Odometry,'/point_lio/odometry',lambda m:pose('fused',m),1000),
      node.create_subscription(PointCloud2,'/point_lio/cloud_registered',cloud,100),
      node.create_subscription(String,'/point_lio/height_status',lambda m:seen.update(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
with (HERE/'output_replay_smoke.txt').open('w') as log:
    process=subprocess.Popen(['ros2','launch',str(ROOT/'launch/replay_2d_mapping.launch.py'),
        'rviz:=false','output_map:='+str(HERE/'output_replay_map/map')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        end=time.monotonic()+25
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if process.poll() is not None:raise RuntimeError('Output replay launch exited early')
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT);process.wait(timeout=35)
common=poses['raw'].keys()&poses['fused'].keys()
exact=all([poses['raw'][t][i] for i in [0,1,3,4,5,6]]==[poses['fused'][t][i] for i in [0,1,3,4,5,6]] for t in common)
result=dict(matched_poses=len(common),fused_clouds=clouds,xy_orientation_exact=exact,height_status=seen,robot_commands_sent=False)
result['passed']=bool(len(common)>50 and clouds>50 and exact and seen.get('filter',{}).get('updated',0)>0)
(HERE/'output_replay_validation.json').write_text(json.dumps(result,indent=2)+'\n')
node.destroy_node();rclpy.shutdown();print(json.dumps(result),flush=True)
raise SystemExit(0 if result['passed'] else 1)
