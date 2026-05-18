# 04. AI Supervisory Layer (Isolated)

## 4.1 Purpose and Isolation Philosophy

This layer provides mission-level optimization and anomaly assistance while remaining **isolated from direct low-level control authority**.

- The certified deterministic GNC stack remains primary for real-time stabilization.
- Supervisory AI sends bounded recommendations to approved interfaces.
- Hard safety interlocks can reject or clip AI outputs at all times.

## 4.2 Runtime Architecture

| Component | Function | Isolation Boundary |
| --- | --- | --- |
| Trajectory Optimization Module | Real-time guidance refinement using warm-start Successive Convexification (SCvx) | Writes advisory trajectory deltas only |
| Anomaly Detection and Recovery Module | Detects off-nominal propulsion/sensor behavior and proposes thrust reallocation plans | Cannot bypass deterministic fault manager |
| Vision Hazard Detection and Avoidance (HDA) Module | Detects landing hazards and proposes safe divert targets | Constrained by pre-approved divert envelope |
| Safety Governor | Validates AI outputs against flight rules and margins | Final gate before deterministic guidance/control ingestion |

## 4.3 Real-Time Trajectory Optimization (Warm-Start SCvx)

## 4.3.1 Inputs and Outputs

| Inputs | Outputs |
| --- | --- |
| Current state estimate, covariance, fuel state, thrust limits, no-fly/keep-out constraints | Time-indexed feasible trajectory update, confidence score, solver status |

## 4.3.2 Operational Characteristics

- Uses warm starts from previously feasible solutions to reduce solve latency.
- Operates in receding-horizon mode with strict compute budget and deadline checks.
- Falls back to baseline deterministic guidance if solver confidence or timing degrades.

## 4.4 Edge Anomaly Detection and Engine-Out Thrust Reallocation

## 4.4.1 Detection Scope

| Domain | Example Indicators |
| --- | --- |
| Propulsion | Pump current anomalies, BLDC RPM mismatch, thrust under-performance |
| Actuation | TVC lag/disagreement, servo health flags |
| Navigation | Innovation spikes, sensor disagreement, intermittent dropouts |

## 4.4.2 Engine-Out Recovery Flow

1. Detect persistent anomaly using temporal consistency checks.
2. Classify severity and isolate failed engine channel(s).
3. Propose thrust redistribution and updated attitude corridor.
4. Submit proposal to safety governor for validation.
5. If accepted, deterministic control allocation transitions to degraded profile.

## 4.5 Vision-Based Landing Hazard Detection and Avoidance (HDA)

## 4.5.1 HDA Pipeline

| Stage | Description |
| --- | --- |
| Scene Perception | Process flash LiDAR/camera feeds for terrain and obstacle segmentation |
| Hazard Scoring | Generate risk maps for slope, roughness, debris, and plume interaction |
| Candidate Site Selection | Compute feasible touchdown candidates with dynamic constraints |
| Divert Recommendation | Provide bounded divert target and confidence to guidance stack |

## 4.5.2 Safety Constraints

- HDA proposals must remain inside mission-defined landing envelope.
- Diverts must preserve fuel and attitude margins required for safe touchdown.
- If confidence drops below threshold, revert to nominal landing target with conservative descent profile.

## 4.6 Verification and Assurance Approach

| Area | Assurance Method |
| --- | --- |
| Supervisory AI timing | Worst-case execution and deadline-miss testing |
| Recommendation safety | Formal rule checks + Monte Carlo envelope validation |
| Engine-out assistance | Fault-injection campaigns in SIL/HIL |
| HDA behavior | High-diversity synthetic + field-like visual datasets |
