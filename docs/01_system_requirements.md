# 01. System Requirements and Interface Definitions

## 1.1 Scope

This document defines baseline GNC-related system requirements for RUPAK and the explicit data interfaces between the Propulsion Subsystem and Avionics.

## 1.2 High-Level System Requirements

| Requirement ID | Requirement Statement | Target/Limit | Verification Method |
| --- | --- | --- | --- |
| GNC-SYS-001 | Vertical landing position error at touchdown shall remain within a bounded dispersion radius. | 2.0 m (1-sigma), 5.0 m (max) | Monte Carlo + flight test |
| GNC-SYS-002 | Vertical touchdown velocity shall be constrained for landing gear survivability. | 1.0 m/s | SIL/HIL + flight test |
| GNC-SYS-003 | Attitude stability shall maintain bounded body rates during terminal descent. | 3.0 deg/s on all axes | HIL + flight test |
| GNC-SYS-004 | Guidance shall track commanded descent corridor and meet corridor exit criteria. | Lateral/vertical corridor violations < 1% mission time | Simulation campaign |
| GNC-SYS-005 | Engine-out in terminal phase shall trigger safe reallocation and degraded mode guidance. | Fault response latency 100 ms | Fault-injection testing |
| GNC-SYS-006 | Avionics shall reject stale sensor/actuator data and declare degraded mode. | Timeout threshold 50 ms at control-critical interfaces | SIL/HIL |

## 1.3 Flight-Phase Requirement Allocation

| Flight Phase | Primary Objective | Critical Requirement IDs |
| --- | --- | --- |
| Boost/Ascent | Maintain attitude and trajectory envelope | GNC-SYS-003, GNC-SYS-004 |
| Coast/Reorientation | State estimation continuity and attitude authority | GNC-SYS-003, GNC-SYS-006 |
| Entry/Descent | Energy and corridor management | GNC-SYS-001, GNC-SYS-004 |
| Terminal Landing | Precision touchdown and disturbance rejection | GNC-SYS-001, GNC-SYS-002, GNC-SYS-003, GNC-SYS-005 |

## 1.4 Propulsion-Avionics Explicit Interface Mapping

### 1.4.1 Interface Principles

- All messages include `timestamp`, `sequence_id`, and `validity` bitmask.
- SI units are mandatory.
- Avionics rejects stale packets beyond defined timeout thresholds.

### 1.4.2 Propulsion-to-Avionics Telemetry (Rx)

| Signal Group | Example Fields | Units | Typical Rate | Consumer |
| --- | --- | --- | --- | --- |
| Electric Pump Telemetry | `pump_voltage`, `pump_current`, `pump_temp`, `pump_status` | V, A, degC, enum | 100 Hz | Health monitor + controller limiter |
| BLDC Motor Telemetry | `bldc_rpm_measured`, `bldc_torque_est`, `bldc_fault_flags` | rpm, N*m, bitfield | 200 Hz | Thrust estimator + anomaly monitor |
| Engine Thrust Telemetry | `thrust_estimate`, `chamber_pressure`, `mixture_ratio_est` | N, Pa, ratio | 100 Hz | Guidance/control allocation |
| TVC Feedback | `tvc_pitch_meas`, `tvc_yaw_meas`, `tvc_servo_health` | deg, deg, enum | 200 Hz | Attitude loop + mixer |

### 1.4.3 Avionics-to-Propulsion Commands (Tx)

| Command Group | Example Fields | Units | Typical Rate | Notes |
| --- | --- | --- | --- | --- |
| Throttle Commands | `throttle_cmd_engine_i`, `throttle_rate_limit` | %, %/s | 400 Hz (actuator loop) | Supports differential throttling by engine |
| BLDC RPM Targets | `bldc_rpm_target_engine_i` | rpm | 200-400 Hz | Feedforward + closed-loop trim |
| TVC Angle Commands | `tvc_pitch_cmd`, `tvc_yaw_cmd` | deg | 400 Hz | Dual-axis command path |
| Arm/Safe and Mode | `engine_enable_mask`, `prop_mode_cmd` | bitmask, enum | 10-50 Hz or event-driven | Safety-critical state machine |

## 1.5 Interface Timing and Fault Handling

| Condition | Detection Logic | Required Action |
| --- | --- | --- |
| Telemetry stale | `now - timestamp > timeout` | Freeze integrators, hold-safe command, flag degraded mode |
| Invalid data flag set | `validity != nominal` | Exclude signal from estimator/controller update |
| Engine channel dropout | Missing channel > 2 cycles at 200 Hz | Trigger thrust reallocation path |
| TVC actuator disagreement | `|cmd - meas| > threshold` for persistence window | Limit attitude commands and escalate fault |
