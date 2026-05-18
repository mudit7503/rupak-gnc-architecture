# 03 — Flight Control Laws and Multi-Rate Hierarchy

## 1. Control Architecture Overview
RUPAK GNC adopts a multi-rate hierarchy to separate mission-level trajectory objectives from fast inner-loop stabilization and actuator-level realization.

| Layer | Nominal Rate | Primary Function |
|---|---|---|
| Guidance | 10 Hz | Trajectory shaping, target state generation, constraint handling |
| Attitude Control | 100 Hz | Body attitude/rate tracking and disturbance rejection |
| Actuator Mixing & Allocation | 400 Hz | Translate demanded moments/thrust into TVC and per-engine throttle commands |

## 2. Core Control Method: INDI
Incremental Nonlinear Dynamic Inversion (INDI) is used as the core attitude control strategy.

### Why INDI
- Reduces sensitivity to modeling errors by using incremental measurements and effectiveness updates.
- Supports rapid compensation for changing propulsion effectiveness during deep throttling.
- Integrates naturally with telemetry-informed actuator adaptation.

### INDI Functional Flow
1. Compute attitude and rate tracking errors.
2. Form incremental control demand using measured angular acceleration/rates and desired dynamics.
3. Apply actuator effectiveness matrix (updated using propulsion feedback health/effectiveness signals).
4. Send achievable moment/thrust requests to allocation and limit management.

## 3. Control Effectors and Axis Strategy

| Axis | Primary Effector | Secondary/Assist Effector | Notes |
|---|---|---|---|
| Pitch | Dual-axis TVC | Differential throttle assist (if needed) | TVC provides dominant authority and precision |
| Yaw | Dual-axis TVC | Differential throttle assist (if needed) | Similar structure to pitch for symmetry |
| Roll | Rapid differential throttling of peripheral electric pumps | TVC cross-coupled residual compensation | Pump-throttle asymmetry gives high-bandwidth roll moment generation |

## 4. Control Allocation and Mixing
- Allocation maps demanded forces/moments to available actuators under limits and health constraints.
- Actuator saturation logic prioritizes touchdown-critical objectives (attitude stability, vertical velocity suppression).
- Engine-out or degraded-pump scenarios trigger online reallocation and command reshaping.
- Mixer outputs include rate-limited throttle and TVC commands with anti-windup-aware feedback.

## 5. Multi-Rate Data Exchange Contract

| Producer → Consumer | Data |
|---|---|
| Guidance (10 Hz) → Attitude Loop (100 Hz) | Desired attitude, thrust vector, descent profile references |
| ESKF (high-rate) → Attitude Loop | Attitude, rates, covariance/health flags |
| Attitude Loop → Mixer (400 Hz) | Required body moments and collective thrust increments |
| Propulsion Feedback → Mixer/INDI | Pump RPM/current, engine health, throttle feedback |

## 6. Safety and Robustness Guards
- Command limiters enforce structural, thermal, and controllability margins.
- Mode logic supports nominal, degraded, and contingency control modes.
- Cross-channel monitoring detects divergence between commanded and realized control effects.
