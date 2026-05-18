# RUPAK VTVL — MATLAB Verification Models

| | |
|---|---|
| **Document ID** | RUPAK-MATLAB-GNC-VER-001 |
| **Revision** | Rev D |
| **Classification** | CONFIDENTIAL — PROGRAMME RESTRICTED |
| **Prepared by** | GNC Simulation and Verification Team |
| **Date** | 2026-05-18 |

---

## 1. Purpose and Scope

This document describes the MATLAB verification model used to check RUPAK roll control behavior.

The focus is practical engineering validation of roll stabilization using differential throttling across the 9 Shakti electric propellant pump-driven engine channels.

---

## 2. Repository Layout

| Path | Purpose |
|---|---|
| `models/differential_thrust_roll_sim.m` | Roll-axis stabilization behavior using differential engine command shaping |

---

## 3. Model — Differential Thrust Roll Controller

### 3.1 Verification Goal

Confirm that roll disturbances are corrected quickly while maintaining command limits and total thrust intent.

### 3.2 Functional Description

- Uses a simplified roll-axis plant for response evaluation.
- Applies roll-rate feedback for smooth orientation adjustments.
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

## 4. Running the Script

From repository root:

```bash
matlab -batch "run('models/differential_thrust_roll_sim.m')"
```

If MATLAB is unavailable, the file can also be opened in GNU Octave for initial review, with plotting differences expected.

---

## 5. Verification Status and Traceability

| Verification Topic | Script | Status |
|---|---|---|
| Roll stabilization and differential command behavior | `differential_thrust_roll_sim.m` | Baseline model available |

Use this document as a lightweight traceability bridge between architecture-level roll-control intent and model-level behavior checks.

---

## 6. Change Log

| Revision | Date | Summary |
|---|---|---|
| Rev D | 2026-05-18 | Removed sensor-fusion model references and retained roll-control verification scope |
| Rev C | 2026-05-18 | Simplified language, removed dense math, kept functional verification intent |

---

*End of Document — RUPAK-MATLAB-GNC-VER-001 Rev D*
