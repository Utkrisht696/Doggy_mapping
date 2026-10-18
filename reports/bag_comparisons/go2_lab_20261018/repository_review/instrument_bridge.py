"""Original bridge with passive audit of accepted/rejected LIO corrections."""
from pathlib import Path
import importlib.util,json,bisect,rclpy
r=Path(__file__).resolve().parent;spec=importlib.util.spec_from_file_location('original_real_bridge',r/'unitree_go2_ros2/unitree_go2_real/scripts/real_tf_bridge.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class AuditBridge(m.RealTfBridge):
 def __init__(self):
  self.events=[];self.source_stamps=[];super().__init__()
 def on_odom(self,msg):
  self.source_stamps.append(m.stamp_ns(msg.header.stamp));super().on_odom(msg)
 def on_lio(self,msg):
  stamp=m.stamp_ns(msg.header.stamp);i=bisect.bisect_left(self.rob_t,stamp);cand=[k for k in (i-1,i) if 0<=k<len(self.rob_t)];matched=bool(cand and min(abs(self.rob_t[k]-stamp) for k in cand)<=m.NS//20);before=self.lio_rejected;old=self.lio_M;super().on_lio(msg);self.events.append({'lio_source_stamp_ns':stamp,'matching_vendor_pose_within_50ms':matched,'rejected':self.lio_rejected>before,'accepted':matched and self.lio_rejected==before,'correction_before':old,'correction_after':self.lio_M})
rclpy.init();node=None
try:
 node=AuditBridge();rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 if node is not None:
  audit={'events':node.events,'source_odom_stamps_ns':node.source_stamps,'lio_rejected':node.lio_rejected,'accepted':sum(e['accepted'] for e in node.events),'unmatched':sum(not e['matching_vendor_pose_within_50ms'] for e in node.events),'final_correction':node.lio_M,'note':'Subclass calls original callback unchanged; only collects before/after counters and poses.'};(r/'results/bridge_audit.json').write_text(json.dumps(audit,indent=2)+'\n');node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
