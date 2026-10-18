# Longer driving bag: drift recheck, 18 October 2026

The 134.417-second live driving capture supports small local surface mismatches, but **does not provide a defensible whole-route horizontal drift measurement**. No robot motion, estimator rerun, TF change, application edit or parameter tuning was performed.

## Source verification

Re-read all 2,074 `/point_lio/odometry` messages directly from the closed SQLite bag, opened read-only. Original timestamps and all XYZ/XYZW pose values exactly match the cached output used for the additional checks. Source: `../../20261016T220926+1100/bag`.

## Additional horizontal check

Built disjoint 5-second windows at 12 times and screened wall overlap for pairs separated by at least 20 seconds. Shared surfaces were selected using geometric plane quality, near-vertical normals, nearest-point distance and normal agreement. This focuses on walls instead of letting floor overlap dominate a horizontal pose fit.

For the initial 10-second reference versus later windows at 35, 45, 55 and 65 seconds, the **as-recorded median absolute wall-plane mismatch is 1.98, 1.65, 1.56 and 1.06 cm**, respectively. Corresponding 90th percentiles are 5.66, 4.97, 3.23 and 2.86 cm. This measures local surface consistency, not position drift; along-wall sliding can leave these residuals small. The correspondence gate can also exclude badly mismatched surfaces.

A horizontal pose fit was checked from three starting yaw guesses, with alternating source points held out. Only the 10→35-second comparison passed the predefined local acceptance checks; its correction was about 4.1 cm / 0.37 degrees. That comparison covers an early short portion, not the complete longer drive. Later fits are ambiguous or lack support: e.g. 10→45 seconds yielded translation solutions differing by 33 cm despite apparently converged local fits. Therefore none supplies a valid final drift value.

The final return/extension views have particularly little common wall support with the earlier views. At 65→115 seconds only 22 wall matches survive, and the tested 45→125 and 55→132 comparisons have zero. Different viewpoints and field of view prevent a trustworthy whole-route horizontal registration. These failures are not evidence that drift equals zero.

## Existing vertical evidence remains valid

At the drain, the outward and return passes share 467 floor cells: median absolute floor-height difference **1.08 cm**, signed median **+1.46 mm**, and 90th percentile absolute difference **3.61 cm**. These are map-XY-associated floor repeats and depend on sufficient horizontal registration.

The user identified this drain/first-right-turn area as the low point of the route. The final section maps about **14–15 cm below it** in raw map Z. Diagnostic correction using the approximately 0.76-degree initial floor tilt leaves **7–8 cm** of mismatch, assuming that starting floor patch was truly level. This flags a possible vertical/attitude consistency issue. It is not a surveyed vertical drift measurement, and the real decline must not be treated as error.

## What cannot be concluded

The bag finishes in a different physical area, so net displacement from the start is not loop-closure drift. The **1.93 m** final difference from vendor odometry is estimator disagreement, not a measured Point-LIO error. A specific overall horizontal drift in cm or percent cannot be derived reliably from this recording. The newer 3 m/5 m out-and-back test provides a better physical endpoint constraint, but does not prove long-route performance.

A measured loop returning to the same physical position/heading, or surveyed landmarks along the route, is needed for a whole-route drift estimate.

## Artifacts

- [Shared-wall check](shared_wall_check.png)
- [All wall fits and rejection/acceptance fields](shared_wall_fits.json)
- [Complete overlap screen](wall_overlap_screen.json)
- [Direct bag/cache equality check](source_recheck.json)
- [Summary metrics](summary.json)
