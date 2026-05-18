# RUPAK VTVL — MATLAB Verification Models

| | |
|---|---|
| **Document ID** | RUPAK-MATLAB-GNC-VER-001 |
| **Revision** | Rev C |
| **Classification** | CONFIDENTIAL — PROGRAMME RESTRICTED |
| **Prepared by** | GNC Simulation and Verification Team |
| **Date** | 2026-05-18 |

---

## 1. Purpose and Scope

This document describes MATLAB verification models used to exercise two critical behaviors in the RUPAK control stack:

1. Roll stabilization through differential propulsion commands
2. Sensor data blending stability under drift and intermittent corrections

The intent is quick engineering validation, not full flight certification.

---

## 2. Repository Layout

| Path | Purpose |
|---|---|
| `models/differential_thrust_roll_sim.m` | Roll-axis stabilization behavior using differential engine command shaping |
| `models/sensor_fusion_drift_filter.m` | Navigation drift correction behavior under blended sensor updates |

---

## 3. Model 1 — Differential Thrust Roll Controller

### 3.1 Verification Goal

Confirm that roll disturbances can be damped quickly while respecting actuator limits and preserving overall thrust intent.

### 3.2 Functional Description

- Uses a simplified roll-axis plant for response evaluation.
- Applies incremental control logic for roll-rate correction.
- Uses differential command shaping across the 9-engine Shakti configuration.
- Enforces practical limits on command amplitude and rate.

### 3.3 What to Check

- Smooth decay of roll angle error
- Bounded roll-rate response
- Differential throttle remains within limits
- No sustained command chatter near steady-state

### 3.4 Typical Expected Outcome

- Fast convergence toward low roll error
- Brief initial command peak followed by stable trimming
- Command profiles that remain within allowed bounds

---

## 4. Model 2 — Sensor Fusion Drift Filter

### 4.1 Verification Goal

Confirm that navigation drift is reduced when intermittent absolute measurements are blended into the high-rate inertial estimate.

### 4.2 Functional Description

- Propagates a nominal navigation estimate at high rate.
- Applies asynchronous corrections when aiding measurements are valid.
- Tracks bias terms and correction confidence over time.
- Publishes a stable estimate suitable for downstream control loops.

### 4.3 What to Check

- Drift growth is reduced after measurement updates
- Bias estimates move toward stable values
- Correction behavior remains smooth without unstable jumps
- Estimate quality degrades gracefully during sensor dropouts

### 4.4 Typical Expected Outcome

- Clear reduction in long-term attitude/position drift trends
- Stable correction cycles after each aiding update
- Healthy estimator behavior during temporary measurement loss

---

## 5. Running the Scripts

From repository root:

```bash
matlab -batch "run('models/differential_thrust_roll_sim.m')"
matlab -batch "run('models/sensor_fusion_drift_filter.m')"
```

If MATLAB is unavailable, these files can also be opened in GNU Octave for initial review, with plotting differences expected.

---

## 6. Verification Status and Traceability

| Verification Topic | Script | Status |
|---|---|---|
| Roll stabilization and differential command behavior | `differential_thrust_roll_sim.m` | Baseline model available |
| Drift correction and sensor blending behavior | `sensor_fusion_drift_filter.m` | Baseline model available |

Use this document as a lightweight traceability bridge between architecture-level control intent and model-level behavior checks.

---

## 7. Change Log

| Revision | Date | Summary |
|---|---|---|
| Rev C | 2026-05-18 | Simplified language, removed dense math, kept functional verification intent |

---

*End of Document — RUPAK-MATLAB-GNC-VER-001 Rev C*
