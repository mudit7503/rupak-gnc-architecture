# 03. Flight Control Laws and Multi-Rate Architecture

## 3.1 Control Objective

Provide a clear, robust flight-control framework with separation between guidance, attitude stabilization, and actuator command execution.

## 3.2 Multi-Rate Control Loop Hierarchy

| Loop Layer | Rate | Primary Inputs | Primary Outputs | Core Method |
| --- | --- | --- | --- | --- |
| Guidance Loop | 10 Hz | Navigation state, mission phase, constraints | Desired acceleration, attitude, thrust profile | Practical trajectory guidance |
| Attitude & Angular-Rate Loop | 100 Hz | Desired attitude/rates, estimated attitude/rates | Incremental moment/thrust demand | Smooth orientation adjustments |
| Actuator Mixing & Allocation | 400 Hz | Incremental moment/thrust demand, actuator states | Engine throttle split + TVC deflection commands | Fast control allocation with limits |

## 3.3 Control Law Structure

### 3.3.1 Guidance Layer (10 Hz)

- Builds references that are feasible under vehicle and landing constraints.
- Publishes smooth setpoints for translational acceleration and attitude.
- Handles mode logic across descent, terminal guidance, and flare.

### 3.3.2 Attitude Control Layer (100 Hz)

- Computes incremental control effort from measured body response.
- Uses feedback updates to keep behavior stable across vehicle changes.
- Applies gain scheduling across mass, thrust, and dynamic pressure changes.

### 3.3.3 Actuator Mixing Layer (400 Hz)

- Converts moment and thrust requests into actuator-level commands.
- Applies saturation, rate limiting, and priority rules (stability first).
- Publishes per-engine throttle commands and TVC commands with fast feedback closure.

## 3.4 Differential Throttling and TVC Allocation

Roll, pitch, and yaw control authority is distributed across propulsion throttling and TVC while preserving total thrust goals.

### 3.4.1 Roll Control via Differential Throttling

| Item | Definition |
| --- | --- |
| Controlled Axis | Roll (body X) |
| Primary Mechanism | Differential throttling across 9 electric propellant pump-driven engine channels |
| Feedback Variable | Roll rate and roll acceleration |
| Operational Description | Rapidly altering motor speeds to stop the rocket from spinning while maintaining target thrust |

### 3.4.2 Pitch/Yaw Control via Dual-Axis TVC

| Item | Definition |
| --- | --- |
| Controlled Axes | Pitch (body Y), Yaw (body Z) |
| Primary Mechanism | Dual-axis TVC gimbal commands (`tvc_pitch_cmd`, `tvc_yaw_cmd`) |
| Feedback Variables | Pitch/yaw rates and attitude errors |
| Coordination | Coupled with throttle allocation to avoid cross-axis saturation |

## 3.5 Control Allocation Priorities

1. Preserve attitude stability and angular-rate limits.
2. Maintain touchdown corridor feasibility.
3. Track thrust demand within engine and pump constraints.
4. Minimize actuator wear and unnecessary command chatter.

## 3.6 Mode Transitions and Protection

| Transition | Trigger | Control Action |
| --- | --- | --- |
| Descent -> Terminal | Altitude/velocity gate satisfied | Tighten attitude bounds and enable precision landing gains |
| Nominal -> Engine-Out Degraded | Engine fault persistence threshold crossed | Recompute control allocation with reduced actuator set |
| Terminal -> Flare | Final altitude gate | Bias thrust and attitude to reduce touchdown velocity |
