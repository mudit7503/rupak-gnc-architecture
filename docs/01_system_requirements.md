# 01 — System Requirements

## 1. Scope
This document defines high-level GNC requirements for RUPAK VTVL descent and landing, and formalizes the propulsion-avionics hardware-data interface required for closed-loop control.

## 2. High-Level GNC Requirements

| Req ID | Requirement Statement | Verification Method |
|---|---|---|
| GNC-SYS-001 | The vehicle shall achieve vertical landing within a **10 m radius (3-sigma CEP)** of the designated touchdown target under nominal winds and mass properties. | Monte Carlo simulation + flight test |
| GNC-SYS-002 | The attitude control system shall maintain bounded attitude stability throughout powered descent and landing, including terminal flare and touchdown transients. | Nonlinear simulation + HIL |
| GNC-SYS-003 | The navigation solution shall provide continuous position, velocity, and attitude estimates with fault flags and covariance outputs for guidance/control consumption. | SIL/HIL + sensor fault injection |
| GNC-SYS-004 | The control system shall support multi-engine thrust allocation and maintain controllability in the presence of a single-engine-out event within defined envelope limits. | Failure-case simulation + integrated test |
| GNC-SYS-005 | Guidance and control functions shall operate deterministically in rate-partitioned loops with bounded end-to-end latency and timestamp consistency. | Timing analysis + real-time bench test |
| GNC-SYS-006 | The system shall provide health-aware throttle and attitude command limiting to preserve control authority and structural/thermal margins. | Analysis + closed-loop simulation |

## 3. Attitude and Stability Performance Targets

| Metric | Target |
|---|---|
| Attitude tracking during terminal descent | Maintain commanded attitude with bounded error suitable for stable touchdown |
| Angular-rate damping | Rapid damping of body-rate disturbances from plume/wind/gust perturbations |
| Limit-cycle avoidance | No sustained control-induced oscillations in pitch, yaw, or roll channels |

## 4. Propulsion–Avionics Hardware-Data Interface

### 4.1 Interface Objective
Enable closed-loop GNC to use real-time electric pump and engine telemetry for thrust estimation, actuator effectiveness tracking, fault detection, and control reallocation.

### 4.2 Telemetry Channels (Per Engine)

| Signal | Description | Typical Consumer |
|---|---|---|
| `engine_id` | Unique engine index | Control allocator, FDIR |
| `pump_motor_rpm` | Electric pump shaft speed | Thrust estimator, INDI effectiveness adaptation |
| `pump_motor_current_A` | Motor current draw | Health monitor, anomaly detection |
| `pump_motor_voltage_V` | Motor bus voltage | Efficiency/derating logic |
| `chamber_pressure` | Combustion state proxy | Thrust model correction |
| `propellant_valve_state` | Valve command/feedback status | Sequencer, fault manager |
| `throttle_cmd` / `throttle_fb` | Commanded vs measured throttle | Actuator monitoring |
| `engine_health_flag` | Engine status (nominal/degraded/failed) | FDIR + control reallocation |
| `timestamp` | Time synchronization marker | Estimator/controller alignment |

### 4.3 Data and Timing Requirements

| Attribute | Requirement |
|---|---|
| Transport | Deterministic avionics bus with integrity checks (CRC, sequence counter) |
| Time alignment | Hardware timestamping with bounded skew to GNC timebase |
| Update rates | Telemetry rate compatible with 100 Hz attitude loop and 400 Hz actuation mixer updates (with hold/interpolation as required) |
| Fault signaling | Explicit quality/status bits and stale-data detection |
| Safety behavior | Defined fail-safe outputs for telemetry timeout, out-of-range values, and contradictory status flags |

### 4.4 Control-Loop Usage of Pump Telemetry
- **Motor RPM** informs thrust production dynamics and actuator effectiveness estimation.
- **Current draw** provides early anomaly cues (pump degradation, cavitation proxies, electrical faults).
- **Health/status flags** trigger engine-out thrust redistribution and guidance envelope tightening.
- **Timestamped feedback** supports coherent multi-rate fusion and stable INDI increment computation.

## 5. Traceability Seed (Example)

| Requirement | Allocated Function | Primary Interface |
|---|---|---|
| GNC-SYS-001 | Terminal guidance and landing targeting | Nav state + engine availability |
| GNC-SYS-002 | Attitude stabilization and rate damping | IMU states + TVC/pump actuation feedback |
| GNC-SYS-004 | Fault-tolerant control allocation | Engine health + pump telemetry |
