from pathlib import Path
import json,re
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
r=Path(__file__).resolve().parent;run=r/'run_01';base=Path('/home/jain/Doggy/reports/bag_comparisons/go2_lab_20261018')
s=np.load(base/'source_decoded.npz');a=np.load(run/'outputs.npz');b=np.load(base/'outputs.npz');origin=int(s['raw_stamp_ns'][0]);t=(a['odom_stamp_ns']-origin)/1e9;bt=(b['odom_stamp_ns']-origin)/1e9;vt=(s['vendor_stamp_ns']-origin)/1e9
lr=Rotation.from_quat(a['odom_values'][:,3:]);vr=Slerp(vt,Rotation.from_quat(s['vendor_values'][:,3:7]))(t);initial=(t>=2.6)&(t<=3.6);alignment=lr[initial].mean()*vr[initial].mean().inv();angular=np.rad2deg(((alignment*vr).inv()*lr).magnitude())
stamp,ia,ib=np.intersect1d(a['odom_stamp_ns'][t<28],b['odom_stamp_ns'][bt<28],return_indices=True);pre=np.linalg.norm(a['odom_values'][ia,:3]-b['odom_values'][ib,:3],axis=1)
text=(run/'mapping.log').read_text();events=[]
for line in text.splitlines():
 if 'RECOVERY ' in line:events.append(line.split('RECOVERY ',1)[1])
comparison=json.loads((run/'analysis/comparison.json').read_text());baseline=json.loads((base/'analysis/comparison.json').read_text())
newret=comparison['return_checks']['point_lio'];oldret=baseline['return_checks']['point_lio'];vendor=comparison['return_checks']['vendor']
summary={'return_checks':{'original_point_lio':oldret,'recovery_trial':newret,'vendor':vendor},'reseed_attempts':sum(e.startswith('reseeded') for e in events),'resumed_events':sum(e.startswith('resumed') for e in events),'discarded_overlap_scans':sum(e.startswith('discard_overlap') for e in events),'recovery_events':events,'identical_before_first_gap':{'shared_stamps':len(stamp),'max_position_difference_m':float(pre.max()),'exact_pose_arrays':bool(np.array_equal(a['odom_values'][ia],b['odom_values'][ib]))},'orientation_disagreement_vs_vendor_after_initial_alignment':{'median_deg':float(np.median(angular)),'p95_deg':float(np.percentile(angular,95)),'max_deg':float(angular.max())},'health':comparison['health'],'replay':json.loads((run/'result.json').read_text()),'scope':'Vendor-aided recovery at acquisition gaps, not global relocalization or loop closure.'}
(run/'recovery_comparison.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
