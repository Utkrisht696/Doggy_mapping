# Cache resize fix: soft-turn bag test

The cache resize correction did not change this recording’s trajectory or drift. All 2,176 pose values are exactly identical in the corrected run, fresh unchanged control and saved baseline. Horizontal return offset remains 3.510261 m and height change remains −0.432620 m. This removes a confirmed C++ defect, but it is not an effective remedy for the observed soft-turn failure.

Only two estimator source lines changed in an isolated copy: point caches now use `resize` before indexed access. The IMU initialization defect, point timing, calibration, noise settings, TF bridge and 4,000-point startup override are unchanged. The active workspace source/install and the original recovery baseline are protected and unchanged.

## Results

| Case | Horizontal return offset | Height change | Heading change |
|---|---:|---:|---:|
| Saved baseline | 3.510261 m | -43.262 cm | +0.278° |
| Fresh unchanged control | 3.510261 m | -43.262 cm | +0.278° |
| Cache resize fix | 3.510261 m | -43.262 cm | +0.278° |

Fix vs fresh control: 2176 shared output stamps, exact pose-value equality = True, maximum direct position difference 0 m and orientation difference 6.41138337e-15°.

The code correction removes invalid vector indexing. Its effect on drift must be judged from these measured results rather than assumed from the bug. A remaining metre-scale return offset means the complete estimation failure is unresolved, even if a metric improves. If the fresh control and fixed run differ, replay variability/map evolution must also be considered; a single pair does not prove causality.

## Verification

- Fresh unchanged control: 2176 finite poses; raw/IMU observer delivery exact = True/True; output gaps >150 ms = 0; missing exact-stamp TF = 0; recovery reseeds = 0.
- Cache resize fix: 2176 finite poses; raw/IMU observer delivery exact = True/True; output gaps >150 ms = 0; missing exact-stamp TF = 0; recovery reseeds = 0.

Protected active/baseline files unchanged: True. Original source bag files unchanged: True.

Both fresh runs use the exact same derived input bag, source stamps, QoS overrides, profile and fixed quiet-pose analysis windows at 1x, on separate localhost-only DDS domains. No controls are replayed, no robot commands are sent and RViz is not opened. Output arrays checkpoint every 15 seconds. The source diff audit confirms only the two cache lines changed; source image assets and .git were excluded from the private copy. CMake embeds the new private source directory as the runtime log root.

The checked minimal cache-access reproduction previously failed for reserve and passed for resize. The full trial uses the same Release optimization mode as the old estimator; it is not an estimator build with debug container assertions. Observer delivery does not separately instrument every estimator callback; output continuity is audited.

[Comparison plot](comparison.png) · [Numeric comparison](comparison.json) · [Exact applied diff](applied.patch) · [Changed-source audit](source_change_audit.json)

The startup-IMU clearing patch remains unapplied. Any next estimator change requires separate confirmation under the user’s stepwise review instruction.
