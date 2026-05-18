# 03. Flight Control Laws and Multi-Rate Architecture

## 3.1 Control Objective

Provide a robust, reusable flight-control framework using **Incremental Nonlinear Dynamic Inversion (INDI)** with explicit multi-rate loop separation for guidance, attitude stabilization, and actuator-level command realization.

## 3.2 Multi-Rate Control Loop Hierarchy

| Loop Layer | Rate | Primary Inputs | Primary Outputs | Core Method |
| --- | --- | --- | --- | --- |
| Guidance Loop | 10 Hz | Navigation state, mission phase, constraints | Desired acceleration/attitude/thrust profile | Constrained trajectory guidance |
| Attitude & Angular-Rate Loop | 100 Hz | Desired attitude/rates, estimated attitude/rates | Incremental moment/thrust commands | INDI attitude control |
| Actuator Mixing & Allocation | 400 Hz | Incremental moment/thrust commands, actuator states | Engine throttle split + TVC deflection commands | Control allocation + rate/limit handling |

## 3.3 INDI-Centered Law Structure

INDI is applied with layered responsibilities so each loop uses the state and actuator information available at its update rate.

## 3.3.1 Guidance Layer (10 Hz)

- Generates feasible references under vehicle and landing constraints.
- Publishes smooth setpoints for translational acceleration and attitude.
- Applies mode logic across descent, terminal guidance, and flare.

## 3.3.2 Attitude Control Layer (100 Hz)

- Computes incremental control action from measured angular accelerations/rates.
- Reduces model dependence by using measured response to previous actuation.
- Enforces bandwidth and gain scheduling versus mass, thrust, and dynamic pressure.

Representative incremental structure:

\[
\Delta u = G^{-1} \left( \nu_{cmd} - \nu_{meas} \right)
\]

Where \(\Delta u\) is incremental control input, \(\nu_{cmd}\) desired angular acceleration surrogate, and \(\nu_{meas}\) measured response.

## 3.3.3 Actuator Mixing Layer (400 Hz)

- Resolves commanded body moments and thrust into available actuators.
- Applies saturation, rate limiting, and prioritization (stability first).
- Produces per-engine throttle and TVC servo commands with feedback closure.

## 3.4 Differential Throttling and TVC Allocation

Roll, pitch, and yaw authority are distributed across propulsion throttling and TVC while preserving total thrust objectives and actuator limits.

## 3.4.1 Roll Control via Differential Throttling

| Item | Definition |
| --- | --- |
| Controlled Axis | Roll (body X) |
| Primary Mechanism | Differential throttle among symmetrically placed engines |
| Feedback Variable | Roll rate and roll acceleration |
| Key Constraint | Maintain total thrust target while generating roll moment |

Command split concept:

- `throttle_total` satisfies translational thrust demand.
- `delta_throttle_roll_i` terms generate roll control moment.
- Mixer enforces per-engine min/max and slew constraints.

## 3.4.2 Pitch/Yaw Control via Dual-Axis TVC

| Item | Definition |
| --- | --- |
| Controlled Axes | Pitch (body Y), Yaw (body Z) |
| Primary Mechanism | Dual-axis TVC gimbal commands (`tvc_pitch_cmd`, `tvc_yaw_cmd`) |
| Feedback Variables | Pitch/yaw rates and attitude errors |
| Coordination | Coupled with throttle allocation to avoid cross-axis saturation |

## 3.5 Control Allocation Priorities

1. Preserve attitude stability and angular-rate limits.
2. Maintain touchdown corridor feasibility.
3. Track thrust demand subject to engine and pump constraints.
4. Minimize actuator wear and unnecessary command chatter.

## 3.6 Mode Transitions and Protection

| Transition | Trigger | Control Action |
| --- | --- | --- |
| Descent -> Terminal | Altitude/velocity gate satisfied | Tighten attitude bounds and enable precision landing gains |
| Nominal -> Engine-Out Degraded | Engine fault persistence threshold crossed | Recompute control allocation with reduced actuator set |
| Terminal -> Flare | Final altitude gate | Bias thrust and attitude for touchdown velocity minimization |
