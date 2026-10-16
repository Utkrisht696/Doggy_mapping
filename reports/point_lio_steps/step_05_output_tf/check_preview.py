"""Isolated synthetic test only: no mapper, RViz, bag replay or robot contact."""
import json,subprocess,signal,time,os
from pathlib import Path
import rclpy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy
assert os.environ['ROS_DOMAIN_ID']=='197' and os.environ['ROS_LOCALHOST_ONLY']=='1'
p=Path('reports/point_lio_steps/step_05_output_tf');statuses=[];transforms=[]
with (p/'preview_smoke.log').open('w') as log:
 process=subprocess.Popen(['ros2','launch',str(Path('launch/point_lio_go2.launch.py').resolve()),'rviz:=false'],stdout=log,stderr=subprocess.STDOUT)
 node=None
 try:
  rclpy.init();node=rclpy.create_node('isolated_preview_observer')
  subs=[node.create_subscription(String,'/point_lio/tf_status',lambda m:statuses.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),node.create_subscription(TFMessage,'/tf',lambda m:transforms.extend(m.transforms),100)]
  def spin(seconds):
   until=time.monotonic()+seconds
   while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
  spin(7)
  names=node.get_node_names();publishers=node.count_publishers('/point_lio/odometry')
  assert process.poll() is None
  assert 'doggy_point_lio_tf' in names and 'doggy_point_lio' not in names,names
  assert publishers==0 and not transforms and statuses,(publishers,len(transforms),statuses)
  assert statuses[-1]['received']==0 and statuses[-1]['published']==0,statuses
  result={'preview_launch_passed':True,'isolated_domain':197,'localhost_only':True,'mapper_present':False,'estimator_odometry_publishers_before_synthetic_test':publishers,'global_tf_messages_before_synthetic_test':len(transforms),'initial_status':statuses[-1],'rviz_not_started':True,'robot_contacted':False}
  # Exercise actual ROS subscriptions/broadcasts, still only within test domain.
  tf_pub=node.create_publisher(TFMessage,'/tf',100);odom_pub=node.create_publisher(Odometry,'/point_lio/odometry',100)
  spin(.5)
  tfs=[]
  for stamp,x in [(10,1.0),(12,3.0)]:
   tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='imu';tf.header.stamp.sec=stamp;tf.transform.translation.x=x;tf.transform.rotation.w=1.0;tfs.append(tf)
  tf_pub.publish(TFMessage(transforms=tfs));spin(.3)
  m=Odometry();m.header.frame_id='map';m.child_frame_id='imu';m.header.stamp.sec=11;m.header.stamp.nanosec=123456789;m.pose.pose.orientation.w=1.0;m.pose.pose.position.x=20.0
  odom_pub.publish(m);spin(.5)
  output=[t for t in transforms if t.header.frame_id=='map' and t.child_frame_id=='odom']
  assert len(output)==1,len(output)
  assert output[0].header.stamp==m.header.stamp
  assert abs(output[0].transform.translation.x-17.876543211)<1e-9
  result.update(synthetic_transport_passed=True,exact_source_stamp_preserved=True,exact_time_tf_interpolation_passed=True,synthetic_map_to_odom_x=output[0].transform.translation.x)
 finally:
  if process.poll() is None:process.send_signal(signal.SIGINT)
  process.wait(timeout=8)
  if node is not None:node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
result['launch_returncode']=process.returncode
(p/'preview_smoke.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
