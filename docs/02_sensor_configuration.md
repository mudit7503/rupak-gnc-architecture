# 02 — Sensor Configuration and Fusion Framework

## 1. Sensor Suite Overview
The navigation architecture uses redundant and complementary sensing to support robust operation from boostback/descent through terminal landing.

| Sensor | Configuration | Primary Role |
|---|---|---|
| IMUs | Cross-strapped dual/triple IMU set | High-rate attitude and specific-force propagation |
| NavIC/GPS | Dual-frequency GNSS receiver chain | Absolute position/velocity updates during high-altitude and mid-altitude regimes |
| FMCW Radar Altimeter | Downward-looking, robust in dust/plume conditions | Precision altitude and descent-rate aiding near ground |
| Flash LiDAR / Cameras | Terrain-relative sensing for terminal phase | Hazard detection, relative pose, landing site refinement |

## 2. Mapping Sensors to Flight Regimes

| Flight Regime | Primary Sensors | Secondary/Validation Sensors | Key Outputs |
|---|---|---|---|
| High-altitude descent | Cross-strapped IMUs + NavIC/GPS | Starved/quality-checked vision updates (if available) | Inertial state with GNSS-corrected drift |
| Mid-altitude powered descent | IMUs + NavIC/GPS + FMCW radar altimeter | Vision range/feature cues | Improved vertical channel observability |
| Terminal approach / landing | IMUs + FMCW radar altimeter + Flash LiDAR/Cameras | GNSS as bounded reference | Relative position, attitude, hazard-aware touchdown targeting |
| Touchdown transient | IMUs + radar altimeter (validity-gated) | Vision consistency checks | Stable attitude/rate and vertical velocity suppression |

## 3. Error-State Kalman Filter (ESKF) Architecture

### 3.1 Nominal State (example)
The nominal navigation state propagates using inertial mechanization:
- Position in navigation frame
- Velocity in navigation frame
- Attitude (quaternion)
- IMU bias terms (gyro bias, accelerometer bias)

### 3.2 Error-State Definition
A small-error state is maintained for linearized estimation updates:
- Position error, velocity error, attitude error (small-angle)
- Gyro and accelerometer bias errors
- Optional sensor scale/misalignment terms as calibration matures

### 3.3 Propagation Step
- Propagate nominal state with IMU measurements at high rate.
- Propagate error covariance with linearized dynamics and process noise.
- Use cross-strapped IMU consistency checks for fault isolation and confidence weighting.

### 3.4 Measurement Update Models
- **NavIC/GPS update:** position/velocity innovation.
- **FMCW radar update:** altitude/range-rate innovation.
- **Vision/LiDAR update:** relative pose/terrain constraints with quality gating.

### 3.5 Quality Gating and Fault Handling
- Innovation-based gating (NIS/chi-square style thresholding).
- Sensor validity flags propagated to guidance/control users.
- Graceful degradation sequencing (e.g., GNSS dropouts, vision obscuration, radar multipath anomalies).

## 4. Interface to GNC Consumers

| Consumer | Required Data from ESKF |
|---|---|
| Guidance (10 Hz) | Position, velocity, mission-frame covariance, validity flags |
| Attitude control (100 Hz) | Attitude, body rates, bias-compensated angular states |
| Actuation mixer (400 Hz equivalent update) | Fast state derivatives, actuator effectiveness context, timing metadata |

## 5. Design Notes
- Cross-strapping supports redundancy management and continuity under single-sensor failure.
- Regime-dependent weighting improves robustness as observability changes with altitude and environment.
- Covariance and health outputs are treated as first-class interface products, not byproducts.
