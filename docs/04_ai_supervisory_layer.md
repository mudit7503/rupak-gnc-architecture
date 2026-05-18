# 04 — AI Supervisory Layer (Safe, Non-End-to-End)

## 1. Supervisory Philosophy
AI is applied as a **bounded supervisory layer**, not as a direct end-to-end flight controller. Classical, certifiable GNC remains in primary control authority.

### Core Safety Principles
- AI outputs are advisory or constrained commands that must pass deterministic guardrails.
- Hard safety envelopes (attitude, velocity, thrust/margin limits) remain enforced by conventional flight software.
- Fallback to deterministic baseline logic is immediate on confidence loss, data-quality faults, or policy violations.

## 2. Supervisory Functions

## 2.1 Real-Time Trajectory Optimization
**Objective:** Improve fuel/margin usage and touchdown quality under disturbances.

- Uses warm-start **Successive Convexification (SCvx)** seeded by the current guidance trajectory.
- Runs in receding horizon with strict solve-time budgets and feasibility checks.
- Candidate trajectory must satisfy hard constraints before adoption.

| Item | Supervisory Requirement |
|---|---|
| Runtime budget | Must complete within bounded guidance-cycle allocation |
| Feasibility | Infeasible outputs are rejected automatically |
| Safety gate | Constraint violations trigger reversion to baseline trajectory |

## 2.2 Edge LSTM Anomaly Detection + Engine-Out Reallocation
**Objective:** Detect propulsion anomalies early and trigger robust thrust redistribution.

- Onboard LSTM processes telemetry streams (pump RPM, current draw, chamber pressure trends, response lag).
- Produces anomaly score + confidence + affected engine/pump context.
- Decision layer confirms anomaly with rule-based checks before triggering FDIR actions.
- Automatic **engine-out thrust reallocation** updates control allocation priorities and descent envelope limits.

| Input Signals | AI Output | Deterministic Action |
|---|---|---|
| RPM/current/pressure trends | Anomaly score, fault class, confidence | Validate, isolate engine, reallocate thrust, tighten guidance envelope |

## 2.3 Vision-Based Hazard Detection and Avoidance (HDA)
**Objective:** Improve landing safety by rejecting hazardous touchdown zones.

- Flash LiDAR/camera data is used to derive terrain slope, roughness, obstacle maps, and confidence masks.
- HDA selects safe candidate landing cells in a bounded divert region.
- Guidance receives only validated target updates that satisfy kinematic and fuel margins.

| HDA Stage | Output |
|---|---|
| Perception | Hazard map + uncertainty layers |
| Site ranking | Feasible candidate set with risk score |
| Supervisor gate | Approved landing retarget command or no-update fallback |

## 3. Integration with Baseline GNC

| Interface Path | Data Contract |
|---|---|
| ESKF/Propulsion Health → AI Supervisor | State estimate, covariance, telemetry trends, health flags |
| AI Supervisor → Guidance | Constraint-compliant trajectory updates or candidate retarget points |
| AI Supervisor → FDIR | Structured anomaly events and confidence metadata |
| Baseline FSW → AI Supervisor | Operational mode, compute budget, safety envelope definitions |

## 4. Assurance and Verification Strategy
- Offline training/validation with domain-randomized simulation and recorded telemetry.
- Runtime monitors for out-of-distribution behavior and confidence collapse.
- Independent verification of fallback behavior in SIL/HIL and mission rehearsal campaigns.
- AI modules treated as augmentative components with clear authority boundaries.

## 5. Non-End-to-End Compliance Statement
The supervisory AI stack shall not directly replace state estimation, control law computation, or actuator command generation. Its role is constrained to optimization, anomaly advisories, and hazard-aware recommendations subject to deterministic acceptance logic.
