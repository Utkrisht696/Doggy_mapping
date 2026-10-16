"""Passive raw DDS rate/tick/body-IMU comparison; no sensor substitution."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,struct,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.serialization import serialize_message,deserialize_message
from unitree_go.msg import LowState,SportModeState
ROOT=Path('/home/jain/Doggy');BASE=Path(__file__).resolve().parent;dest=BASE/datetime.now(ZoneInfo('Australia/Melbourne')).strftime('%Y%m%dT%H%M%S%z');dest.mkdir();(BASE/'current_run.json').write_text(json.dumps({'directory':str(dest.relative_to(ROOT))},indent=2)+'\n')
marker=[.12345+i*.371 for i in range(13)];marker=list(struct.unpack('<13f',struct.pack('<13f',*marker)));offsets={}
for typ in [LowState,SportModeState]:
 m=typ();m.imu_state.quaternion=marker[:4];m.imu_state.gyroscope=marker[4:7];m.imu_state.accelerometer=marker[7:10];m.imu_state.rpy=marker[10:13]
 if typ is LowState:m.tick=0xA1B2C3D4
 raw=serialize_message(m);imu_offset=raw.find(struct.pack('<13f',*marker));assert imu_offset>=4;tick_offset=raw.find(struct.pack('<I',0xA1B2C3D4)) if typ is LowState else None;offsets[typ]=(imu_offset,tick_offset)
rclpy.init();n=Node('lowstate_passive_rate_audit',enable_rosout=False,start_parameter_services=False);q=QoSProfile(depth=4000,reliability=ReliabilityPolicy.RELIABLE);topics=['/lowstate','/lf/lowstate','/hf/lowstate','/sportmodestate'];subs=[];data={t:[] for t in topics};verified={t:0 for t in topics}
for t in topics:subs.append(n.create_subscription(SportModeState if t=='/sportmodestate' else LowState,t,lambda _:None,q,raw=True))
start=time.monotonic_ns()
while (time.monotonic_ns()-start)<15*10**9:
 anymsg=False
 for topic,sub in zip(topics,subs):
  typ=SportModeState if topic=='/sportmodestate' else LowState;im,tk=offsets[typ]
  for _ in range(64):
   with sub.handle:item=sub.handle.take_message(sub.msg_type,sub.raw)
   if item is None:break
   anymsg=True;raw,meta=item;host,mono=time.time_ns(),time.monotonic_ns();assert raw[:2]==b'\x00\x01';values=struct.unpack_from('<13f',raw,im);tick=struct.unpack_from('<I',raw,tk)[0] if tk is not None else -1
   source_stamp=(lambda x:x[0]*10**9+x[1])(struct.unpack_from('<II',raw,4)) if typ is SportModeState else 0
   if verified[topic]<5:
    msg=deserialize_message(raw,typ);actual=[*msg.imu_state.quaternion,*msg.imu_state.gyroscope,*msg.imu_state.accelerometer,*msg.imu_state.rpy];assert list(values)==actual
    if typ is LowState:assert tick==msg.tick
    else:assert source_stamp==int(msg.stamp.sec)*10**9+int(msg.stamp.nanosec)
    verified[topic]+=1
   data[topic].append((host,mono,int(meta.get('source_timestamp',0)),tick,source_stamp,values))
 if not anymsg:time.sleep(.0005)
end=time.monotonic_ns();endpoints={t:[{'node':e.node_name,'reliability':e.qos_profile.reliability.name} for e in n.get_publishers_info_by_topic(t)] for t in topics};result={'duration_s':(end-start)/1e9,'topics':{},'raw_parsing_verified':verified,'offsets':{k.__name__:v for k,v in offsets.items()},'robot_commands_sent':False,'application_changed':False}
for t,rows in data.items():
 name=t.strip('/').replace('/','_');arr={k:np.array([r[i] for r in rows],dtype=np.int64) for i,k in enumerate(['host_ns','mono_ns','dds_source_ns','tick','source_stamp_ns'])};arr['values']=np.array([r[5] for r in rows],dtype=float).reshape(-1,13);np.savez_compressed(dest/(name+'.npz'),**arr)
 lo,hi=start+2*10**9,end-10**9;mask=(arr['mono_ns']>=lo)&(arr['mono_ns']<=hi);v=arr['values'][mask];mo=arr['mono_ns'][mask];ticks=arr['tick'][mask];dds=arr['dds_source_ns'][mask];span=(int(mo[-1])-int(mo[0]))/1e9 if len(mo)>1 else 0;changes=int(np.sum(np.any(np.diff(v[:,4:10],axis=0)!=0,axis=1))) if len(v)>1 else 0
 out={'publisher_endpoints':endpoints[t],'received_messages':len(rows),'interior_messages':len(mo),'receipt_hz':(len(mo)-1)/span if span else 0,'distinct_gyro_acc_updates_hz_observed':changes/span if span else 0,'all_six_values_unchanged_consecutive_messages':max(0,len(mo)-1-changes),'has_epoch_header':t=='/sportmodestate','interior_span_s':span,'dds_publication_timestamp_nonzero':bool(len(dds) and np.all(dds>0))}
 if t!='/sportmodestate' and len(mo)>1:
  dt=np.diff(ticks);out.update(unique_ticks=len(np.unique(ticks)),tick_backsteps=int(np.sum(dt<0)),tick_step_quantiles=np.percentile(dt,[0,50,95,99,100]).tolist(),tick_span=int(ticks[-1]-ticks[0]))
 if len(v):out.update(first_imu_q_wxyz=v[0,:4].tolist(),mean_gyro_xyz=v[:,4:7].mean(axis=0).tolist(),mean_acceleration_xyz=v[:,7:10].mean(axis=0).tolist())
 result['topics'][t]=out
(dest/'comparison.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'directory':str(dest.relative_to(ROOT)),**result},indent=2));n.destroy_node();rclpy.shutdown()
