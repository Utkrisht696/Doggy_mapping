# Offline geometry review — 16 October 2026

No application code, configuration, original URDF, running TF or robot setting was changed. This review created only reports. The application-file SHA256 values are saved in `geometry_audit.json` and were checked unchanged at completion.

## Result

| Item | Result |
|---|---|
| Original developer radar mount | Preserved; it is not an identity alias for the DDS cloud frame |
| Manufacturer IMU offset | L1 and L2 SDK coordinate definitions agree |
| Point-LIO extrinsic direction | Verified from its executable point transformation |
| Raw gyro/cloud axis agreement | Supported by the independent manual posture bag |
| Existing CMU processed input geometry | Does not compose with identity extrinsics under the current raw-axis relationship |
| Exact DDS IMU-to-cloud origin | Still a manufacturer-profile assumption, not independently measured through topics |
| Ready to apply code changes | No edits applied; implementation design and assumptions still need user confirmation |

## Transform notation and offset direction

`T_A_B` maps coordinates from B into A: `p_A = R_A_B p_B + t_A_B`. Its translation is the origin of B expressed in A. A TF edge with parent A and child B uses this convention.

Unitree defines the SDK LiDAR origin at the centre of the bottom mounting surface. The SDK IMU axes are parallel to the SDK LiDAR axes, with the IMU origin at:

```text
IMU origin in LiDAR coordinates = [-0.007698, -0.014655, +0.006670] m
```

Sources:

- [Official L1 coordinate definition](https://github.com/unitreerobotics/unilidar_sdk#coordinate-system-definition)
- [Official L2 coordinate definition](https://github.com/unitreerobotics/unilidar_sdk2#2-coordinate-system-definition)

Therefore the nominal SDK TF parent LiDAR / child IMU uses that negative-X, negative-Y, positive-Z offset with identity rotation. Point-LIO requires the inverse relationship, because `pointBodyLidarToIMU` actually evaluates:

```text
p_IMU = extrinsic_R * p_LiDAR + extrinsic_T
```

For those original parallel SDK frames, this means:

```text
extrinsic_R = identity
extrinsic_T = [+0.007698, +0.014655, -0.006670] m
```

The actual local equation is in `third_party/autonomy_stack_go2/src/slam/point_lio_unilidar/src/laserMapping.cpp`, function `pointBodyLidarToIMU`. This equation takes precedence over the ambiguous YAML prose comment. The official SDK2 ROS driver also publishes parent IMU / child LiDAR using the inverse positive-X/positive-Y/negative-Z translation.

**These values are for the stated SDK frames. A different origin or any coordinate preprocessing changes the required relationship.**

## What the local CMU preprocessing does

The same geometry exists in the [upstream CMU source](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/foxy-humble/src/utilities/transform_sensors/transform_sensors/transform_everything.py).

Cloud conversion:

```text
p_Lprocessed = R_C * p_Lraw + c
R_C = rotation about Y by 2.878202585 radians
c = [0, 0, -0.046825] m
```

Gyro vector conversion, before bias/projection corrections:

```text
g_Iprocessed = R_G * g_Iraw
R_G = rotation about Y by approximately -15.1 degrees,
      multiplied by diag(1, -1, -1)
```

These are different rotations. For parallel raw cloud/gyro axes, the full relative processed transform must follow:

```text
T_Iprocessed_Lprocessed =
    T_Iprocessed_Iraw * T_Iraw_Lraw * inverse(T_Lprocessed_Lraw)
```

The numerical composition produces a relative processed rotation of **180 degrees**, whereas the current Point-LIO YAML uses identity. This is a mismatch between the processing and the extrinsics for this raw input profile. It is not a new physical mount value to install. Fixing it requires one coherent choice of input axes/origins and then recomputing the relative transform; adding TF alone does not change the preprocessing or estimator equations.

For the explicit diagnostic assumption that the gyro frame origin remains at the physical IMU and only its axes are rotated, the expected processed relative translation is:

```text
[-0.017853006685473197, -0.014655, 0.04891837593037775] m
```

That diagnostic value is NOT a recommended replacement configuration. Both processed headers saying `body` does not prove their axes or physical origins are identical. The upstream vector-conversion code does not apply an IMU origin translation. The cloud callback does shift its point origin. A physical base-link frame and an IMU-centred estimator state must not be conflated.

Also, the configured quaternion left-rotation differs from the rotation actually applied to gyro vectors by approximately 30.19 degrees. Point-LIO's processed topic later overwrites orientation with identity, so this is not itself the current estimator's orientation input; it is another reason not to treat that quaternion branch as a validated frame conversion. The later cross-axis gyro calibration mapping is generally nonorthonormal and must not be encoded as a TF rotation.

## Avoiding double application of geometry

A clean geometry design retains raw sensor frames and applies the LiDAR-to-IMU extrinsic once inside Point-LIO, with mounting TF used for robot integration and visualization. If processed inputs are retained instead, their new axes AND origins need explicit definitions and the full composition above.

Even applying the same rotation to both sensors changes the coordinates of a nonzero lever arm. For example, using the earlier raw-to-vendor-body rotation for both sets of axes while retaining their distinct physical sensor origins would yield the Point-LIO translation:

```text
[-0.009547508617967191, -0.01443781724086259, 0.004348751718515664] m
```

It would not remain the unchanged SDK translation. Extra cloud-only origin shifts must be included too. Angular-velocity vectors themselves do not acquire a translation, and merely giving them a `base_link` header does not relocate the accelerometer's measurement point to the body origin.

## What the bag establishes

The independently recovered conversion between vendor raw and body clouds remained valid across pitch, roll and yaw in the new bag: 11,838 matched points across ten cloud pairs, maximum numerical conversion residual about 2.45e-6 m. This is equivalence of two published coordinate streams, not surveyed physical mounting accuracy.

Applying that same rotation to raw gyro vectors also follows body gyro motion, supporting common raw cloud/gyro axes. This does not determine IMU translation. The raw acceleration and fused orientation remain inconsistent with physical-body gravity and need separate input-model/calibration handling.

## Remaining origin assumption

Manufacturer documentation specifies the internal offset relative to the SDK bottom-mount-surface origin. The accessible DDS topics do not publish an explicit factory extrinsic or certify that their raw cloud origin is that exact SDK origin. The manual rotation test cannot identify that translation, particularly with unvalidated acceleration. No public authoritative document found in this review explicitly bridges this robot's DDS origin to the SDK origin.

The nominal SDK offset can be proposed as a documented manufacturer-profile assumption, with that limit made explicit. It cannot be called independently measured or fully verified for this DDS origin. This uncertainty is separate from the strongly supported raw-axis relationship. A further pitch/roll/yaw test would not remove it.

The earlier proposed `radar -> utlidar_lidar` measurement edge remains review-only and unapplied. A conditional subsequent `utlidar_lidar -> utlidar_imu` edge is listed in `geometry_audit.json`; it depends on adopting the SDK-origin assumption. Neither changes the original developer `radar` or body `imu` mount.

## Numerical checks

All eight algebra checks passed, including forward/inverse composition, rigid-rotation validity, processed-transform composition against 2,000 arbitrary points, rotated lever-arm coordinates, and composition beneath the unchanged developer mount. These checks validate the equations under the stated assumptions; they do not validate unknown physical origins or successful mapping.

No Point-LIO, SLAM, replay, calibration routine or live robot test was started in this review.
