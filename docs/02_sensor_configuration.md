# 02. Sensor Configuration and Fusion Architecture

## 2.1 Objective

Define the baseline sensor suite for RUPAK by flight regime and establish an Error-State Kalman Filter (ESKF)-based navigation architecture.

## 2.2 Sensor Suite Allocation by Flight Regime

| Flight Regime | IMU | NavIC/GPS | Radar Altimeter | Flash LiDAR / Cameras | Primary Navigation Mode |
| --- | --- | --- | --- | --- | --- |
| Liftoff and Ascent | Primary high-rate source | Used when available for drift correction | Not primary | Not primary | Inertial + GNSS aiding |
| Exo/coast and Reorientation | Primary source | Intermittent/geometry dependent | Not used | Optional attitude/scene cues | Inertial propagation |
| Entry and High-Altitude Descent | Primary source | Reacquired for state correction | Activated below operational ceiling | Optional terrain context | Inertial + GNSS blended |
| Terminal Descent (Low Altitude) | Primary source | Used if quality allows | Primary altitude/vertical-rate source | Primary hazard/relative-nav source | Multi-sensor precision landing mode |
| Final Flare and Touchdown | Primary source | Optional cross-check | Primary for altitude gate | Primary for hazard lockout and pad-relative pose | Radar/LiDAR-anchored landing solution |

## 2.3 Sensor Roles and Constraints

| Sensor | Strengths | Constraints | Mitigation |
| --- | --- | --- | --- |
| IMU | High-rate dynamics and short-term observability | Bias/scale drift over time | Continuous ESKF bias estimation |
| NavIC/GPS | Absolute position/velocity observability | Dropout, multipath, degraded geometry | Innovation gating + confidence-weighted fusion |
| Radar Altimeter | Reliable AGL in terminal descent | Limited range and beam geometry effects | Regime-dependent enable + sanity checks |
| Flash LiDAR/Cameras | High-resolution terrain/hazard cues | Lighting, dust/plume effects, compute load | Region-of-interest processing + quality flags |

## 2.4 ESKF-Based Fusion Architecture

## 2.4.1 State Definition (Representative)

| State Block | Example Elements |
| --- | --- |
| Nominal State | Position, velocity, attitude quaternion |
| Error State | Small-angle attitude errors, position/velocity errors |
| Sensor Biases | Gyro bias, accelerometer bias |
| Optional Augmented States | Radar bias, vision scale/latency correction |

## 2.4.2 Processing Pipeline

1. **Propagation (IMU rate):** Integrate nominal dynamics and propagate covariance.
2. **Measurement Update (asynchronous):** Apply NavIC/GPS, radar altimeter, and vision/LiDAR innovations with gating.
3. **Error Injection:** Inject estimated error state into nominal state and reset error-state mean.
4. **Health and Consistency Monitoring:** Use innovation statistics (NIS/NEES-style checks) and sensor quality flags.
5. **Output Publication:** Publish navigation state and covariance to guidance/control loops.

## 2.4.3 Fusion Timing and Data Rates

| Function | Nominal Rate | Notes |
| --- | --- | --- |
| IMU Propagation | 400-1000 Hz | Highest-rate estimator backbone |
| GNSS/NavIC Update | 5-20 Hz | Quality-gated absolute aiding |
| Radar Altimeter Update | 20-100 Hz | Activated by altitude regime |
| Vision/LiDAR Update | 10-30 Hz | Compute-constrained, event/quality driven |
| Navigation Solution Output | 100 Hz | Consumed by guidance and control |

## 2.5 Degraded-Mode Navigation Strategy

| Failure Case | Degraded Strategy |
| --- | --- |
| GNSS dropout | Inertial propagation with radar/vision aiding when available |
| Radar invalid in terminal phase | Increase vision weighting and tighten descent envelope |
| Vision degradation due to plume/lighting | Fall back to radar + inertial with conservative touchdown target |
| IMU anomaly | Trigger supervisory fault mode and inhibit aggressive maneuvers |
