"""Summarize fixed-profile replays; no fitting or trajectory corrections."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
cases = ['soft', 'small', 'measured_route_2', 'spinny', 'running_regression']
labels = ['Soft turns', 'Small wander', '5/10 m + rectangles', 'New spins', 'Earlier spins']
old = {
    'soft': root.parent/'bag_comparisons/wander_comeback_20261018/wander_soft_turn_comeback',
    'small': root.parent/'bag_comparisons/wander_comeback_20261018/small_wander_comeback',
    'measured_route_2': root.parent/'bag_comparisons/new_routes_20261018/measured_route_2',
}
rows = []
fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), layout='constrained')
for name, label in zip(cases, labels):
    base = root/'final_validation'/name/'run_01'
    report = json.loads((base/'analysis/comparison.json').read_text())
    row = {'case': name, 'label': label,
           'endpoint': report['first_to_last_analyzed_hold'], 'health': report['health'],
           'nominal_distance_checks': report['nominal_distance_checks'],
           'exact_cloud_delivery': report['replay_result']['cloud_delivery_exact'],
           'exact_imu_delivery': report['replay_result']['imu_delivery_exact']}
    if name in old:
        control = json.loads((old[name]/'run_01/analysis/comparison.json').read_text())
        row['historical_endpoint'] = control['first_to_last_analyzed_hold']
    rows.append(row)
    if name in cases[:3]:
        ax = axes[cases.index(name)]
        for folder, title, color in [(old[name]/'run_01', 'Previous profile', '#b35242'),
                                      (base, 'Calibrated profile', '#197a76')]:
            d = np.load(folder/'analysis/comparison.npz')
            xyz = d['lio_position_initial_heading']
            ax.plot(xyz[:, 0], xyz[:, 1], color=color, label=title, linewidth=1)
            ax.scatter(xyz[-1, 0], xyz[-1, 1], color=color, marker='x', s=45)
        ax.scatter(0, 0, marker='*', color='black', s=70, label='Starting hold')
        ax.set(title=label, xlabel='Initial-heading X (m)', ylabel='Y (m)')
        ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.25); ax.legend(fontsize=8)
fig.suptitle('One fixed calibration/profile across bags; no position scaling or loop closure')
fig.savefig(root/'final_validation/route_comparison.png', dpi=170)
plt.close(fig)
(root/'final_validation/summary.json').write_text(json.dumps(rows, indent=2)+'\n')
for row in rows:
    p = row['endpoint']['point_lio']
    print(row['case'], 'return_m', p['horizontal_displacement_m'],
          'height_m', p['height_change_m'], 'heading_deg', p['heading_change_deg'])
