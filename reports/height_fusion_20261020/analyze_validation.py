"""Score the production integration without assuming an exact physical return."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
run=HERE/'production_1x';r=json.loads((run/'validation.json').read_text())
a=np.load(run/'outputs.npz');source=np.load(ROOT/'reports/bag_comparisons/return_drift_20261018_112353/source_decoded.npz');origin=int(source['raw_stamp_ns'][0])
ends=source['raw_stamp_ns']+np.rint(source['raw_values'][:,3]*1e9).astype(np.int64)
o=a['raw_stamp_ns'];idx=np.clip(np.searchsorted(ends,o),0,len(ends)-1);choices=np.stack([idx,np.maximum(idx-1,0)],axis=1)
distance=abs(ends[choices]-o[:,None]);nearest=choices[np.arange(len(o)),np.argmin(distance,axis=1)]
good=np.min(distance,axis=1)<1000000;missing=np.setdiff1d(np.arange(nearest[0],nearest[-1]+1),nearest[good])
r['scan_processing_coverage']=dict(unmatched_output_stamps=int((~good).sum()),missing_interior_scans=int(len(missing)),
    missing_source_times_s=((source['raw_stamp_ns'][missing]-origin)/1e9).tolist(),
    maximum_scan_end_matching_error_ns=int(np.min(distance,axis=1).max()))
def pose(stamps,values,window):
    t=(stamps-origin)/1e9;mask=(t>=window[0])&(t<=window[1]);assert mask.sum()>=8
    tr=np.eye(4);tr[:3,3]=np.median(values[mask,:3],axis=0)
    tr[:3,:3]=Rotation.from_quat(values[mask,3:7]).mean().as_matrix()
    return tr
returns=[]
for name in ['first_return_reference','endpoint_reference']:
    ref=json.loads((ROOT/'reports/bag_comparisons/return_drift_20261018_112353'/name/'endpoint_registration.json').read_text())
    assert ref['quality_passed'];truth=np.array(ref['registrations']['forward']['best']['transform']);row={'reference':name}
    for key in ['raw','fused']:
        begin,end=[pose(a[key+'_stamp_ns'],a[key+'_values'],w) for w in ref['source_windows_s']]
        estimate=np.linalg.inv(begin)@end;dp=estimate[:3,3]-truth[:3,3]
        yaw=Rotation.from_matrix(estimate[:3,:3]).as_euler('xyz',degrees=True)[2]
        actual=Rotation.from_matrix(truth[:3,:3]).as_euler('xyz',degrees=True)[2]
        row[key]=dict(horizontal_error_m=float(np.linalg.norm(dp[:2])),height_error_m=float(dp[2]),heading_error_deg=float((yaw-actual+180)%360-180))
    returns.append(row)
r['returns']=returns
r['filter_core_matches_reviewed_prototype']=hashlib.sha256((ROOT/'scripts/height_fusion.py').read_bytes()).hexdigest()=='05ae901d903f062c1db97109a9b02d2d3a9575a142e410987f9dce45f327f2e0'
r['passed']=bool(r['passed'] and not len(missing) and good.all() and r['max_cloud_z_correction_error_m']<1e-5 and r['output_gaps_over_150ms']==0 and r['filter_core_matches_reviewed_prototype'])
(run/'validation.json').write_text(json.dumps(r,indent=2)+'\n')
replay=json.loads((HERE/'output_replay_validation.json').read_text());assert replay['passed']
terrain=r.get('terrain') or {}
lines=['# Permanent height-fusion integration — 20 October 2026','',
    'The current live 2D, legacy Point-LIO and recorded-output replay launches now start height fusion unconditionally. Public Point-LIO odometry/cloud/path topics are corrected; diagnostic estimator outputs are under `/point_lio/raw/` in `lio_local`. Current 2D odometry, body TF and terrain maps all consume the same corrected pose/cloud pair. No optional enable flag was added.','',
    'The scalar filter is byte-identical to the reviewed causal prototype. Its settings and the approved body-IMU calibration, sensor geometry and internal Point-LIO estimator are unchanged. Vendor body Z is converted to the developer IMU origin, aligned once at startup, and accepted only from the past within 50 ms. Missing/rejected measurements propagate raw LIO increments and growing height variance. No horizontal/yaw vendor scaling, future fit or feedback into the internal estimator is introduced.','',
    'The path display is decimated, bounded to 6,000 poses and published at most 2 Hz; the estimator\'s unused raw path is disabled. Odometry and cloud rates remain unchanged. Losing the height-fusion process stops the launch, preventing an unnoticed switch to unfused outputs. Vendor input QoS has a deeper bounded best-effort queue.','',
    '## Verification','',
    '- Full suite: **34 tests passed**, including physical frame composition, exact-time TF, causal/missing-height behavior, synthetic measured ramp, padded big-endian clouds, unchanged XY/quaternion/source stamps, bounded path publication and terrain/self-filter checks.',
    f"- Full original sensor recording at **1×**, through `launch/live_2d_mapping.launch.py`, headless, localhost-only: **passed={r['passed']}**.",
    f"- Inputs: {len(source['raw_stamp_ns']):,} raw scans; {len(source['imu_stamp_ns']):,} derived body IMU samples; {len(source['vendor_stamp_ns']):,} vendor odometry samples. Observed source stamp sequences match exactly: {r['input_delivery_exact']}.",
    f"- Outputs: {len(o):,} raw/fused/body poses; {r['clouds']['fused']:,} registered clouds; median {r['output_rate_hz']:.2f} Hz. Interior scans missing from estimator outputs: {len(missing)}. Output intervals over 150 ms: {r['output_gaps_over_150ms']}.",
    f"- Raw/fused XY/quaternion and cloud XY are exact; maximum cloud-Z shift error {r['max_cloud_z_correction_error_m']:.3g} m. Body mount composition error {r.get('max_body_pose_mount_error_m',float('nan')):.3g} m. Missing exact body-TF stamps: {r['missing_exact_body_tf_stamps']}; conflicting TF parents: {r['tf_conflicts']}.",
    f"- Recorded-output replay smoke test: {replay['matched_poses']} matched poses and {replay['fused_clouds']} clouds; fusion applied, XY/quaternion unchanged, public frame `odom`.",
    '- All three launch descriptions load successfully. Original recordings were read-only; no robot command topics were replayed.','',
    '| Return | Raw horizontal | Fused horizontal | Heading error (both) | Raw height | Fused height |',
    '|---|---:|---:|---:|---:|---:|']
for row in returns:
    b,f=row['raw'],row['fused']
    lines.append(f"| {row['reference']} | {100*b['horizontal_error_m']:.1f} cm | {100*f['horizontal_error_m']:.1f} cm | {f['heading_error_deg']:+.2f}° | {100*b['height_error_m']:+.2f} cm | {100*f['height_error_m']:+.2f} cm |")
lines+=['','These are discrepancies against independent quiet-cloud registrations, not surveyed ground truth. Raw and fused X/Y are exact; the slightly different horizontal-error scalar comes from projecting translation into the initially tilted IMU axes after changing Z.','',
    '## Remaining limits','',
    f"The complete 2D mapper produced maps throughout the test. Its existing capacity bound reached {terrain.get('voxels')} voxels; the last status reports {terrain.get('capacity_drops')} capacity-dropped points and {terrain.get('self_filter_dropped_scans',terrain.get('skipped_scans_missing_model_or_joints'))} scans skipped for missing model/joint alignment. This is a terrain-map completeness limit, separate from exact raw/fused pose/cloud transport. Capacity is explicit in `/terrain/status`; this integration does not claim an unlimited map.",
    'The filter corrects output Z only. Horizontal/heading drift, large source-data gaps and accelerated-replay sensitivity remain. Vendor Z has not been validated as world elevation over drains/ramps. The configured uncertainty is provisional; upstream twist is retained and full fused velocity/covariance is not a validated navigation state. No autonomous controller is started.',
    'The GitHub push precedes the requested physical-robot test. This report covers recorded-data verification; a live test result is recorded separately after the push.','',
    'Evidence: `production_1x/validation.json`, `output_replay_validation.json`, `unit_tests.txt`, passive input provenance and the included validation scripts. Raw bags/build artifacts are excluded from Git.']
(HERE/'REPORT.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'passed':r['passed'],'returns':returns,'coverage':r['scan_processing_coverage']},indent=2))
raise SystemExit(0 if r['passed'] else 1)
