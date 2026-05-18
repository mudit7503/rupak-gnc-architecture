# 04. AI Supervisory Layer (Isolated)

## 4.1 Purpose and Isolation Philosophy

This layer provides mission-level optimization and anomaly support while remaining isolated from direct low-level control authority.

- The deterministic GNC stack stays primary for real-time stabilization.
- Supervisory AI sends bounded recommendations through approved interfaces.
- Hard safety interlocks can reject or clip AI outputs at any time.

## 4.2 Runtime Architecture

| Component | Function | Isolation Boundary |
| --- | --- | --- |
| Trajectory Optimization Module | Refines guidance references in real time within compute limits | Writes advisory trajectory deltas only |
| Anomaly Detection and Recovery Module | Detects off-nominal propulsion/sensor behavior and proposes safe reallocation plans | Cannot bypass deterministic fault manager |
| Vision Hazard Detection and Avoidance Module | Detects landing hazards and recommends safe divert targets | Limited to pre-approved divert envelope |
| Safety Governor | Validates AI recommendations against flight rules and margins | Final gate before deterministic stack ingestion |

## 4.3 Real-Time Trajectory Optimization

This module continuously improves the current trajectory plan while maintaining feasibility and timing guarantees.

### 4.3.1 Inputs and Outputs

| Inputs | Outputs |
| --- | --- |
| Current state estimate, covariance, fuel state, thrust limits, no-fly/keep-out constraints | Time-indexed trajectory update, confidence score, solver status |

### 4.3.2 Operational Characteristics

- Uses warm starts from recent feasible solutions to reduce latency.
- Operates in receding-horizon mode with strict compute budget checks.
- Falls back to baseline deterministic guidance if confidence or timing degrades.

## 4.4 Edge Anomaly Detection and Engine-Out Reallocation

Anomaly handling emphasizes fast detection, bounded confidence, and deterministic handoff to certified fault management.

### 4.4.1 Detection Scope

| Domain | Example Indicators |
| --- | --- |
| Propulsion | Pump current anomalies, BLDC RPM mismatch, thrust under-performance |
| Actuation | TVC lag/disagreement, servo health flags |
| Navigation | Innovation spikes, sensor disagreement, intermittent dropouts |

### 4.4.2 Engine-Out Recovery Flow

1. Detect persistent anomaly using temporal consistency checks.
2. Classify severity and isolate failed engine channel(s).
3. Propose thrust redistribution and updated attitude corridor.
4. Submit recommendation to safety governor for validation.
5. If accepted, deterministic control allocation transitions to degraded profile.

## 4.5 Vision-Based Landing Hazard Detection and Avoidance

This function provides perception-driven touchdown-site assessment and bounded divert recommendations for terminal guidance.

### 4.5.1 Hazard Processing Pipeline

| Stage | Description |
| --- | --- |
| Scene Perception | Process flash LiDAR/camera feeds for terrain and obstacle mapping |
| Hazard Scoring | Build risk maps for slope, roughness, debris, and plume interaction |
| Candidate Site Selection | Identify feasible touchdown candidates under dynamic limits |
| Divert Recommendation | Publish bounded divert target and confidence to guidance stack |

### 4.5.2 Safety Constraints

- Recommendations must remain inside the mission landing envelope.
- Diverts must preserve fuel and attitude margins needed for safe touchdown.
- If confidence drops below threshold, revert to nominal target and conservative descent profile.

## 4.6 Verification and Assurance Approach

| Area | Assurance Method |
| --- | --- |
| Supervisory AI timing | Worst-case execution and deadline-miss testing |
| Recommendation safety | Rule checks plus Monte Carlo envelope validation |
| Engine-out assistance | Fault-injection campaigns in SIL/HIL |
| Hazard detection behavior | High-diversity synthetic and field-like visual datasets |
