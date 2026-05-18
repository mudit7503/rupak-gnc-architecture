# 01. System Requirements and Interface Definitions

## 1.1 Scope

This document defines baseline GNC requirements for RUPAK and the key data interfaces between propulsion and avionics.

## 1.2 High-Level System Requirements

| Requirement ID | Requirement Statement | Target/Limit | Verification Method |
| --- | --- | --- | --- |
| GNC-SYS-001 | Touchdown position error shall remain within a controlled landing dispersion area. | 2.0 m (1-sigma), 5.0 m (max) | Monte Carlo + flight test |
| GNC-SYS-002 | Vertical touchdown velocity shall stay within landing gear limits. | 1.0 m/s | SIL/HIL + flight test |
| GNC-SYS-003 | Attitude stabilization shall keep body rates bounded during terminal descent. | 3.0 deg/s on all axes | HIL + flight test |
| GNC-SYS-004 | Guidance shall stay inside the approved descent corridor profile. | Corridor violations < 1% mission time | Simulation campaign |
| GNC-SYS-005 | Engine-out in terminal phase shall trigger safe control reallocation. | Fault response latency 100 ms | Fault-injection testing |
| GNC-SYS-006 | Avionics shall reject stale control-critical data and enter degraded mode. | Timeout threshold 50 ms | SIL/HIL |

## 1.3 Flight-Phase Requirement Allocation

| Flight Phase | Primary Objective | Critical Requirement IDs |
| --- | --- | --- |
| Boost/Ascent | Maintain attitude and trajectory envelope | GNC-SYS-003, GNC-SYS-004 |
| Coast/Reorientation | Preserve stable state estimation and control authority | GNC-SYS-003, GNC-SYS-006 |
| Entry/Descent | Manage energy and corridor tracking | GNC-SYS-001, GNC-SYS-004 |
| Terminal Landing | Deliver precision touchdown with disturbance rejection | GNC-SYS-001, GNC-SYS-002, GNC-SYS-003, GNC-SYS-005 |

## 1.4 Propulsion-Avionics Interface Mapping

### 1.4.1 Interface Principles

- All messages carry `timestamp`, `sequence_id`, and `validity` fields.
- SI units are required.
- Stale packets are rejected at the avionics boundary.

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
| Throttle Commands | `throttle_cmd_engine_i`, `throttle_rate_limit` | %, %/s | 400 Hz | Supports differential throttling by engine |
| BLDC RPM Targets | `bldc_rpm_target_engine_i` | rpm | 200-400 Hz | Feedforward + closed-loop trim |
| TVC Angle Commands | `tvc_pitch_cmd`, `tvc_yaw_cmd` | deg | 400 Hz | Dual-axis command path |
| Arm/Safe and Mode | `engine_enable_mask`, `prop_mode_cmd` | bitmask, enum | 10-50 Hz or event-driven | Safety-critical state machine |

## 1.5 Interface Timing and Fault Handling

| Condition | Detection Logic | Required Action |
| --- | --- | --- |
| Telemetry stale | `now - timestamp > timeout` | Freeze integrators, hold-safe command, flag degraded mode |
| Invalid data flag set | `validity != nominal` | Exclude signal from estimator/controller update |
| Engine channel dropout | Missing channel > 2 cycles at 200 Hz | Trigger thrust reallocation path |
| TVC actuator disagreement | Persistent command/measurement mismatch | Limit attitude commands and escalate fault |
