# 02. Sensor Configuration and Fusion Architecture

## 2.1 Objective

Define the baseline sensor suite by flight regime and the sensor data blending strategy used for robust navigation.

## 2.2 Sensor Suite Allocation by Flight Regime

| Flight Regime | IMU | NavIC/GPS | Radar Altimeter | Flash LiDAR / Cameras | Primary Navigation Mode |
| --- | --- | --- | --- | --- | --- |
| Liftoff and Ascent | Primary high-rate source | Used when available for drift correction | Not primary | Not primary | Inertial + GNSS aiding |
| Exo/coast and Reorientation | Primary source | Intermittent/geometry dependent | Not used | Optional attitude/scene cues | Inertial propagation |
| Entry and High-Altitude Descent | Primary source | Reacquired for state correction | Activated below operational ceiling | Optional terrain context | Inertial + GNSS blended |
| Terminal Descent (Low Altitude) | Primary source | Used if quality allows | Primary altitude/vertical-rate source | Primary hazard/relative-nav source | Multi-sensor precision landing mode |
| Final Flare and Touchdown | Primary source | Optional cross-check | Primary altitude gate source | Primary hazard lockout and pad-relative pose source | Radar/LiDAR-anchored landing solution |

## 2.3 Sensor Roles and Constraints

| Sensor | Strengths | Constraints | Mitigation |
| --- | --- | --- | --- |
| IMU | High-rate motion tracking and short-term stability | Bias and scale drift over time | Continuous bias estimation and health checks |
| NavIC/GPS | Absolute position/velocity reference | Dropout, multipath, degraded geometry | Quality gating and confidence-weighted updates |
| Radar Altimeter | Reliable altitude-above-ground support in descent | Range and beam-geometry limits | Altitude-dependent activation and sanity checks |
| Flash LiDAR/Cameras | High-resolution terrain and hazard context | Lighting, dust/plume effects, compute load | Region-of-interest processing with quality flags |

## 2.4 Sensor Data Blending Architecture

The navigation stack combines a high-rate inertial backbone with asynchronous corrections from aiding sensors.

### 2.4.1 Core State Elements

| State Block | Typical Elements |
| --- | --- |
| Navigation State | Position, velocity, attitude |
| Correction State | Small residual errors applied to navigation state |
| Sensor Bias Tracking | Gyro and accelerometer bias estimates |
| Optional Augmented Terms | Radar or vision alignment correction terms |

### 2.4.2 Processing Pipeline

1. **Propagation (IMU rate):** Advance the navigation estimate continuously using IMU data.
2. **Asynchronous updates:** Blend NavIC/GPS, radar, and vision/LiDAR measurements when valid.
3. **Correction step:** Apply bounded corrections to keep the estimate stable and consistent.
4. **Health monitoring:** Use innovation and sensor-quality checks to reject bad updates.
5. **Output publication:** Publish navigation state at control-consumption rate.

### 2.4.3 Timing and Data Rates

| Function | Nominal Rate | Notes |
| --- | --- | --- |
| IMU propagation | 400-1000 Hz | Estimation backbone |
| GNSS/NavIC update | 5-20 Hz | Quality-gated absolute aiding |
| Radar altimeter update | 20-100 Hz | Active by altitude regime |
| Vision/LiDAR update | 10-30 Hz | Compute- and quality-driven |
| Navigation solution output | 100 Hz | Consumed by guidance and control |

## 2.5 Degraded-Mode Navigation Strategy

| Failure Case | Degraded Strategy |
| --- | --- |
| GNSS dropout | Continue inertial propagation and use radar/vision where available |
| Radar invalid in terminal phase | Increase vision weighting and tighten descent envelope |
| Vision degradation due to plume/lighting | Fall back to radar + inertial with conservative touchdown target |
| IMU anomaly | Trigger supervisory fault mode and inhibit aggressive maneuvers |
