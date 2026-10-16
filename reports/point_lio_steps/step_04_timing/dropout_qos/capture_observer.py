"""Read-only independent subscriber used in the authorized QoS comparison."""
from pathlib import Path
import argparse,json,os,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Imu
from unitree_go.msg import SportModeState
parser=argparse.ArgumentParser();parser.add_argument('--name',required=True);parser.add_argument('--depth',type=int,required=True);parser.add_argument('--reliability',choices=['best_effort','reliable'],required=True);parser.add_argument('--topic',required=True);parser.add_argument('--seconds',type=float,default=68);parser.add_argument('--output',required=True);a=parser.parse_args();dest=Path(a.output);dest.mkdir(parents=True,exist_ok=True)
rclpy.init();n=Node('qos_audit_'+a.name,enable_rosout=False,start_parameter_services=False);rows=[];endpoints=[];errors=[]
qos=QoSProfile(depth=a.depth,reliability=ReliabilityPolicy.RELIABLE if a.reliability=='reliable' else ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
def receive(msg):
 host,mono=time.time_ns(),time.monotonic_ns()
 if a.topic=='/sportmodestate':
  stamp=msg.stamp;imu=msg.imu_state;values=[*imu.accelerometer,*imu.gyroscope,*imu.quaternion]
 else:
  stamp=msg.header.stamp;acc=msg.linear_acceleration;g=msg.angular_velocity;q=msg.orientation;values=[acc.x,acc.y,acc.z,g.x,g.y,g.z,q.w,q.x,q.y,q.z]
 rows.append((int(stamp.sec)*10**9+int(stamp.nanosec),host,mono,[float(v) for v in values]))
sub=n.create_subscription(SportModeState if a.topic=='/sportmodestate' else Imu,a.topic,receive,qos)
start=time.monotonic_ns();last_status=0
try:
 while (time.monotonic_ns()-start)/1e9<a.seconds:
  rclpy.spin_once(n,timeout_sec=.05)
  elapsed=(time.monotonic_ns()-start)/1e9
  if elapsed>=last_status+5:
   last_status=elapsed
   if not endpoints:endpoints=[{'node':v.node_name,'namespace':v.node_namespace,'reliability':str(v.qos_profile.reliability),'durability':str(v.qos_profile.durability)} for v in n.get_publishers_info_by_topic(a.topic)]
   (dest/(a.name+'.status.json')).write_text(json.dumps({'pid':os.getpid(),'elapsed_s':elapsed,'messages':len(rows),'last_source_stamp_ns':rows[-1][0] if rows else None}))
except Exception as exc:errors.append(repr(exc))
end=time.monotonic_ns();n.destroy_node();rclpy.shutdown()
np.savez_compressed(dest/(a.name+'.npz'),stamp_ns=np.array([r[0] for r in rows],dtype=np.int64),host_ns=np.array([r[1] for r in rows],dtype=np.int64),mono_ns=np.array([r[2] for r in rows],dtype=np.int64),values=np.array([r[3] for r in rows],dtype=float))
summary={'name':a.name,'pid':os.getpid(),'topic':a.topic,'depth':a.depth,'reliability':a.reliability,'start_mono_ns':start,'end_mono_ns':end,'duration_s':(end-start)/1e9,'messages':len(rows),'endpoints':endpoints,'errors':errors,'robot_commands_sent':False}
(dest/(a.name+'.json')).write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
raise SystemExit(0 if rows and not errors else 1)
