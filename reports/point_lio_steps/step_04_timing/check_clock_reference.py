"""Independent reliable capture of clock and both source/relayed odometry."""
from pathlib import Path
import time,json
import rclpy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
DEST=Path(__file__).resolve().parent
rclpy.init();n=rclpy.create_node('doggy_clock_reference_audit',enable_rosout=False,start_parameter_services=False)
rows={'clock':[],'source':[],'relay':[]};subs=[]
def receive(st,name):rows[name].append(int(st.sec)*10**9+int(st.nanosec))
subs.append(n.create_subscription(Clock,'/doggy/robot_clock',lambda m:receive(m.clock,'clock'),QoSProfile(depth=20000)))
for topic,name in [('/utlidar/robot_odom','source'),('/odom','relay')]:subs.append(n.create_subscription(Odometry,topic,lambda m,k=name:receive(m.header.stamp,k),QoSProfile(depth=20000)))
start=time.monotonic()
while time.monotonic()-start<20:rclpy.spin_once(n,timeout_sec=.02)
lo=max(min(v) for v in rows.values())+10**9; hi=min(max(v) for v in rows.values())-10**9
c=[t for t in rows['clock'] if lo<=t<=hi];source=set(rows['source']);relay=set(rows['relay'])
checks={'enough_clock_samples':len(c)>500,'clock_stamps_match_vendor_odometry':all(t in source for t in c),'clock_stamps_match_relayed_odometry':all(t in relay for t in c),'clock_strictly_increasing':all(b>a for a,b in zip(c,c[1:]))}
result={'passed':all(checks.values()),'checks':checks,'counts':{k:len(v) for k,v in rows.items()},'interior_clock_samples':len(c),'missing_source_stamps':[t for t in c if t not in source],'missing_relay_stamps':[t for t in c if t not in relay],'method':'20-second high-depth RELIABLE capture; exclude 1-second discovery/shutdown boundaries; exact integer timestamp comparison.'}
(DEST/'clock_reference_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown();raise SystemExit(0 if result['passed'] else 1)
