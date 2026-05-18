# RUPAK VTVL — Propulsion / Avionics Interface Control Document

| | |
|---|---|
| **Document ID** | RUPAK-ICD-PROP-AVI-001 |
| **Revision** | Rev C |
| **Classification** | CONFIDENTIAL — PROGRAMME RESTRICTED |
| **Prepared by** | GNC / Propulsion Integration Team |
| **Approved by** | Chief Systems Engineer |
| **Date** | 2025-07-01 |
| **Applicable Vehicle** | RUPAK, Serial 001 onwards |
| **Parent Spec** | RUPAK-SYS-SPEC-001 Rev B |

---

## Table of Contents

1. [Scope and Purpose](#1-scope-and-purpose)
2. [Reference Documents](#2-reference-documents)
3. [Acronyms and Definitions](#3-acronyms-and-definitions)
4. [System Architecture Overview](#4-system-architecture-overview)
5. [Physical Interface Definition](#5-physical-interface-definition)
6. [Data Routing Matrix](#6-data-routing-matrix)
   - 6.1 Motor Shaft Encoder Signals
   - 6.2 Hall-Effect Phase Current Sensors
   - 6.3 Propellant Pressure & Temperature Transducers
   - 6.4 ESC Command & Telemetry Signals
   - 6.5 Thermal Management & Health Monitoring
   - 6.6 System Synchronisation & Timing
7. [Protocol Specifications](#7-protocol-specifications)
8. [Power Interface](#8-power-interface)
9. [Fault Detection, Isolation & Recovery (FDIR) Interfaces](#9-fault-detection-isolation--recovery-fdir-interfaces)
10. [Verification & Validation Matrix](#10-verification--validation-matrix)
11. [Configuration Management & Change Log](#11-configuration-management--change-log)

---

## 1. Scope and Purpose

This Interface Control Document (ICD) formally defines all **electrical, data, and logical interfaces** between the **Shakti Electric Pump-Fed Propulsion Subsystem** (nine (9) Shakti-1E engine modules, each comprising a brushless DC motor-driven LOX and kerosene pump set, an Electronic Speed Controller (ESC), and associated instrumentation) and the **GNC Flight Computer (GFC)** aboard the RUPAK Vertical-Takeoff Vertical-Landing (VTVL) launch vehicle.

This document is the authoritative binding specification for:

- All signal definitions crossing the Propulsion / Avionics interface boundary (IFB-PA)
- Connector pinout allocations (normative)
- Protocol framing, baud-rate, and timing budget allocations
- Fault flag encodings and safe-state commanding contracts
- Verification and acceptance test requirements

All subordinate drawings, harness routing specifications, and software interface requirement documents (IRDs) shall be consistent with the definitions herein. Conflicts shall be escalated to the Configuration Control Board (CCB) via the RUPAK engineering change notice (ECN) process.

---

## 2. Reference Documents

| Ref ID | Document Title | Revision |
|--------|---------------|----------|
| [RD-01] | RUPAK System Requirements Specification | Rev B |
| [RD-02] | Shakti-1E Engine Interface Requirements Document | Rev A |
| [RD-03] | GNC Flight Computer Hardware Specification | Rev D |
| [RD-04] | RUPAK Electrical Power System Architecture | Rev A |
| [RD-05] | CAN-FD Network Topology & Address Map | Rev B |
| [RD-06] | RUPAK FDIR Policy Document | Rev A |
| [RD-07] | Bosch Sensortec BST-BMI088 IMU Datasheet | — |
| [RD-08] | ISRO NavIC L5/S-band Receiver ICD | v2.1 |
| [RD-09] | DO-178C Software Considerations in Airborne Systems | — |
| [RD-10] | MIL-STD-1553B Digital Time Division Command/Response Multiplex Data Bus | — |

---

## 3. Acronyms and Definitions

| Acronym | Definition |
|---------|-----------|
| BLDC | Brushless Direct Current (motor) |
| CAN-FD | Controller Area Network — Flexible Data Rate (ISO 11898-1:2015) |
| ECU | Engine Control Unit |
| ESC | Electronic Speed Controller |
| ESKF | Error-State Kalman Filter |
| FDIR | Fault Detection, Isolation & Recovery |
| GFC | GNC Flight Computer |
| GNC | Guidance, Navigation & Control |
| IFB-PA | Interface Boundary — Propulsion / Avionics |
| LOX | Liquid Oxygen (propellant oxidiser) |
| OBC | On-Board Computer |
| PPS | Pulse Per Second (timing reference) |
| RPM | Revolutions Per Minute |
| RTD | Resistance Temperature Detector |
| SPI | Serial Peripheral Interface |
| TVC | Thrust Vector Control |
| VTVL | Vertical Takeoff, Vertical Landing |

---

## 4. System Architecture Overview

RUPAK employs **nine (9) Shakti-1E engine modules** arranged in a **1 (central) + 8 (peripheral ring)** configuration. Each engine module includes:

- One (1) LOX turbopump driven by a 180 kW BLDC motor
- One (1) RP-1 turbopump driven by a 90 kW BLDC motor
- One (1) dedicated Engine Control Unit (ECU) hosting an ESC and local telemetry aggregation
- Embedded instrumentation: shaft encoders, current sensors, pressure transducers

The **GNC Flight Computer (GFC)** is a radiation-tolerant, dual-redundant processing unit running at 1 GHz. It interfaces to the propulsion subsystem exclusively through:

1. A **redundant CAN-FD bus** (primary and shadow) at 8 Mbit/s for command and telemetry
2. A **hardwired PWM arming/inhibit line** per engine (safety-critical, independent of CAN bus)
3. **Discrete GPIO lines** for engine fire/abort commands (latching opto-isolated relays)

```
GFC (Primary + Shadow)
    │
    ├── CAN-FD Bus A (primary)  ─────────────────────────────────────────────────────┐
    │                                                                                 │
    ├── CAN-FD Bus B (shadow)  ──────────────────────────────────────────────────────┤
    │                                                                                 │
    ├── PWM Arm/Inhibit [×9]  ─────┬──────────────────────────────────────────────  │
    │                               │                                                 │
    └── GPIO Fire/Abort [×9]  ──────┘                                                 │
                                                                                      │
                         ┌─────────────── CAN-FD Network ──────────────────────────┘
                         │
             ECU-1 ... ECU-9
          (each hosting ESC + local ADC + encoder interface)
```

---

## 5. Physical Interface Definition

### 5.1 Connector Allocation

| Connector ID | Location | Mating Part | Shell Size | Contact Type | Function |
|---|---|---|---|---|---|
| J-GFC-PROP-A | GFC Chassis, Bulkhead | MIL-DTL-38999 Series III | Size 25 | 22D Crimp | CAN-FD Bus A + Power |
| J-GFC-PROP-B | GFC Chassis, Bulkhead | MIL-DTL-38999 Series III | Size 25 | 22D Crimp | CAN-FD Bus B (Shadow) |
| J-GFC-PROP-C | GFC Chassis, Bulkhead | MIL-DTL-38999 Series III | Size 17 | 22D Crimp | PWM Arm/Inhibit [×9] |
| J-GFC-PROP-D | GFC Chassis, Bulkhead | MIL-DTL-38999 Series III | Size 17 | 22D Crimp | GPIO Fire/Abort [×9] |
| J-ECU-n-A | ECU n Chassis (n=1..9) | MIL-DTL-38999 Series III | Size 25 | 22D Crimp | CAN-FD Bus A + PWM |
| J-ECU-n-B | ECU n Chassis (n=1..9) | MIL-DTL-38999 Series III | Size 17 | 22D Crimp | Sensor Inputs |

### 5.2 Cable Harness Specification

- All inter-subsystem harnesses: **shielded twisted-pair (STP)**, 24 AWG, Teflon (PTFE) insulation, 200°C rated
- CAN-FD differential pairs: characteristic impedance 120 Ω ± 10%, terminated at both ends
- PWM and GPIO lines: single-ended, 28 AWG with individual drain wire
- Harness shields: grounded at GFC chassis end only (single-point ground)

---

## 6. Data Routing Matrix

### Notation

- **Signal ID** format: `[Subsystem]-[Category]-[Index]-[Direction]`  
  Direction suffix: `_TX` (source transmits) / `_RX` (destination receives)
- **Sampling Rate**: The rate at which the GFC samples or the ECU transmits the signal
- Throttle is expressed as **Normalised Throttle Command (NTC)** where 1.00 = rated thrust

---

### 6.1 Motor Shaft Encoder Signals

> **Physical Implementation:** Each pump motor shaft carries a 4096-count-per-revolution optical incremental encoder with A/B/Z quadrature channels. The ECU performs real-time pulse counting and velocity calculation, publishing the resultant RPM value over CAN-FD.

| Signal ID | Source Subsystem | Destination Subsystem | Protocol / Signal Type | Sampling Rate (Hz) | Description / Range |
|---|---|---|---|---|---|
| `ECU1-ENC-LOX-RX` | ECU-1 (Engine 1 LOX pump encoder) | GFC Navigation Core | CAN-FD Frame ID: 0x101 | 500 | LOX pump shaft speed, Engine 1. Range: 0–15,000 RPM, resolution 1 RPM. Encoded as uint16, LSB = 1 RPM. Stale-data flag bit[15] set if encoder dropout detected. |
| `ECU1-ENC-RP1-RX` | ECU-1 (Engine 1 RP-1 pump encoder) | GFC Navigation Core | CAN-FD Frame ID: 0x102 | 500 | RP-1 pump shaft speed, Engine 1. Range: 0–12,000 RPM, resolution 1 RPM. Encoder loss triggers FDIR Level-1 alert. |
| `ECU2-ENC-LOX-RX` | ECU-2 (Engine 2 LOX pump encoder) | GFC Navigation Core | CAN-FD Frame ID: 0x111 | 500 | LOX pump shaft speed, Engine 2. Same encoding as ECU1-ENC-LOX-RX. |
| `ECU2-ENC-RP1-RX` | ECU-2 (Engine 2 RP-1 pump encoder) | GFC Navigation Core | CAN-FD Frame ID: 0x112 | 500 | RP-1 pump shaft speed, Engine 2. Same encoding as ECU1-ENC-RP1-RX. |
| `ECU3-ENC-LOX-RX` | ECU-3 | GFC | CAN-FD Frame ID: 0x121 | 500 | Engine 3 LOX pump shaft speed. Range: 0–15,000 RPM. |
| `ECU4-ENC-LOX-RX` | ECU-4 | GFC | CAN-FD Frame ID: 0x131 | 500 | Engine 4 LOX pump shaft speed. Range: 0–15,000 RPM. |
| `ECU5-ENC-LOX-RX` | ECU-5 | GFC | CAN-FD Frame ID: 0x141 | 500 | Engine 5 LOX pump shaft speed (central engine). Range: 0–15,000 RPM. |
| `ECU6-ENC-LOX-RX` | ECU-6 | GFC | CAN-FD Frame ID: 0x151 | 500 | Engine 6 LOX pump shaft speed. Range: 0–15,000 RPM. |
| `ECU7-ENC-LOX-RX` | ECU-7 | GFC | CAN-FD Frame ID: 0x161 | 500 | Engine 7 LOX pump shaft speed. Range: 0–15,000 RPM. |
| `ECU8-ENC-LOX-RX` | ECU-8 | GFC | CAN-FD Frame ID: 0x171 | 500 | Engine 8 LOX pump shaft speed. Range: 0–15,000 RPM. |
| `ECU9-ENC-LOX-RX` | ECU-9 | GFC | CAN-FD Frame ID: 0x181 | 500 | Engine 9 LOX pump shaft speed. Range: 0–15,000 RPM. |
| `GFC-ENC-SETPT-TX` | GFC Propulsion Manager | All ECUs (broadcast) | CAN-FD Frame ID: 0x010 | 100 | Desired RPM setpoint broadcast to all ECUs for feed-forward shaft speed regulation. Individual engine setpoints packed into 64-byte payload, 7 bytes per engine. Throttle range: 10%–110% of rated RPM. |

---

### 6.2 Hall-Effect Phase Current Sensors

> **Physical Implementation:** Each BLDC motor phase (U, V, W) incorporates an isolated Hall-effect current transducer (Allegro ACS770 or equivalent, ±200 A range). The ECU samples all three phase currents at 20 kHz internally for motor protection, then publishes aggregated RMS and peak values over CAN-FD.

| Signal ID | Source Subsystem | Destination Subsystem | Protocol / Signal Type | Sampling Rate (Hz) | Description / Range |
|---|---|---|---|---|---|
| `ECU1-CURR-LOX-RMS-RX` | ECU-1 (LOX motor phase current RMS) | GFC Power & Health Monitor | CAN-FD Frame ID: 0x103 | 200 | RMS phase current, Engine 1 LOX motor. Range: 0–250 A (uint16, LSB = 0.01 A). Over-current threshold: 230 A for >50 ms triggers FDIR Level-2. |
| `ECU1-CURR-LOX-PEAK-RX` | ECU-1 (LOX motor peak current) | GFC FDIR Engine | CAN-FD Frame ID: 0x104 | 200 | Peak instantaneous current, Engine 1 LOX motor. Range: 0–400 A. Instantaneous limit: 380 A. Encoded uint16, LSB = 0.1 A. |
| `ECU1-CURR-RP1-RMS-RX` | ECU-1 (RP-1 motor RMS current) | GFC Power & Health Monitor | CAN-FD Frame ID: 0x105 | 200 | RMS phase current, Engine 1 RP-1 motor. Range: 0–160 A (uint16, LSB = 0.01 A). Rated: 90 kW ÷ 400 V ≈ 225 A peak, 159 A RMS. |
| `ECU2-CURR-LOX-RMS-RX` | ECU-2 | GFC | CAN-FD Frame ID: 0x113 | 200 | Engine 2 LOX motor RMS phase current. Same encoding. |
| `ECU3-CURR-LOX-RMS-RX` | ECU-3 | GFC | CAN-FD Frame ID: 0x123 | 200 | Engine 3 LOX motor RMS phase current. |
| `ECU4-CURR-LOX-RMS-RX` | ECU-4 | GFC | CAN-FD Frame ID: 0x133 | 200 | Engine 4 LOX motor RMS phase current. |
| `ECU5-CURR-LOX-RMS-RX` | ECU-5 (central engine) | GFC | CAN-FD Frame ID: 0x143 | 200 | Central engine LOX motor RMS phase current. |
| `ECU6-CURR-LOX-RMS-RX` | ECU-6 | GFC | CAN-FD Frame ID: 0x153 | 200 | Engine 6 LOX motor RMS phase current. |
| `ECU7-CURR-LOX-RMS-RX` | ECU-7 | GFC | CAN-FD Frame ID: 0x163 | 200 | Engine 7 LOX motor RMS phase current. |
| `ECU8-CURR-LOX-RMS-RX` | ECU-8 | GFC | CAN-FD Frame ID: 0x173 | 200 | Engine 8 LOX motor RMS phase current. |
| `ECU9-CURR-LOX-RMS-RX` | ECU-9 | GFC | CAN-FD Frame ID: 0x183 | 200 | Engine 9 LOX motor RMS phase current. |
| `ECU1-CURR-FAULT-RX` | ECU-1 (current fault register) | GFC FDIR Engine | CAN-FD Frame ID: 0x106 | 50 | Bit-packed fault register: bit[0]=over-current, bit[1]=phase-loss, bit[2]=desat-event, bit[3]=temperature-derating, bit[4]=encoder-fault, bits[5:7]=reserved. |

---

### 6.3 Propellant Pressure & Temperature Transducers

> **Physical Implementation:** Measurement points include pump inlet/outlet manifolds, combustion chamber head-end, and injector face. Pressure transducers are Kistler 4043A piezoresistive type (0–500 bar), temperature via K-type thermocouple with local cold-junction compensation in ECU. All analog signals are digitised by a 16-bit ADC local to each ECU and transmitted over CAN-FD.

| Signal ID | Source Subsystem | Destination Subsystem | Protocol / Signal Type | Sampling Rate (Hz) | Description / Range |
|---|---|---|---|---|---|
| `ECU1-PT-LOX-INLET-RX` | ECU-1 (LOX pump inlet PT) | GFC Propulsion Manager | CAN-FD Frame ID: 0x107 | 200 | LOX pump inlet pressure, Engine 1. Range: 0–50 bar absolute (int16, LSB = 0.01 bar). Minimum operational: 2.5 bar (NPSH margin). Low-pressure flag triggers engine-out procedure. |
| `ECU1-PT-LOX-OUTLET-RX` | ECU-1 (LOX pump outlet PT) | GFC Propulsion Manager | CAN-FD Frame ID: 0x108 | 200 | LOX pump discharge pressure, Engine 1. Range: 0–350 bar (int16, LSB = 0.05 bar). Nominal discharge: 280 bar at rated RPM. |
| `ECU1-PT-RP1-INLET-RX` | ECU-1 (RP-1 pump inlet PT) | GFC Propulsion Manager | CAN-FD Frame ID: 0x109 | 200 | RP-1 pump inlet pressure, Engine 1. Range: 0–50 bar. Min operational: 1.5 bar. |
| `ECU1-PT-RP1-OUTLET-RX` | ECU-1 (RP-1 pump outlet PT) | GFC Propulsion Manager | CAN-FD Frame ID: 0x10A | 200 | RP-1 pump discharge pressure, Engine 1. Range: 0–320 bar, nominal 270 bar at rated RPM. |
| `ECU1-PT-CHAMBER-RX` | ECU-1 (combustion chamber PT) | GFC Propulsion Manager | CAN-FD Frame ID: 0x10B | 500 | Chamber pressure, Engine 1. Range: 0–200 bar (int16, LSB = 0.05 bar). Nominal: 160 bar. Hard abort threshold: >185 bar or <40 bar during mainstage. |
| `ECU1-TT-LOX-INLET-RX` | ECU-1 (LOX inlet thermocouple) | GFC Thermal Manager | CAN-FD Frame ID: 0x10C | 50 | LOX pump inlet propellant temperature. Range: −200°C to +50°C (int16, LSB = 0.1°C). Nominal: −183°C. Over-temperature >−150°C triggers FDIR. |
| `ECU1-TT-MOTOR-WIND-RX` | ECU-1 (BLDC winding RTD) | GFC Thermal Manager | CAN-FD Frame ID: 0x10D | 50 | LOX motor winding temperature, Engine 1. Range: −60°C to +180°C (int16, LSB = 0.5°C). Thermal derating begins at 150°C; hard shutdown at 175°C. |
| `ECU2-PT-CHAMBER-RX` | ECU-2 | GFC | CAN-FD Frame ID: 0x11B | 500 | Chamber pressure Engine 2. Same encoding and limits. |
| `ECU3-PT-CHAMBER-RX` | ECU-3 | GFC | CAN-FD Frame ID: 0x12B | 500 | Chamber pressure Engine 3. |
| `ECU4-PT-CHAMBER-RX` | ECU-4 | GFC | CAN-FD Frame ID: 0x13B | 500 | Chamber pressure Engine 4. |
| `ECU5-PT-CHAMBER-RX` | ECU-5 (central engine) | GFC | CAN-FD Frame ID: 0x14B | 500 | Chamber pressure central engine. Most critical for vehicle trim. |
| `ECU6-PT-CHAMBER-RX` | ECU-6 | GFC | CAN-FD Frame ID: 0x15B | 500 | Chamber pressure Engine 6. |
| `ECU7-PT-CHAMBER-RX` | ECU-7 | GFC | CAN-FD Frame ID: 0x16B | 500 | Chamber pressure Engine 7. |
| `ECU8-PT-CHAMBER-RX` | ECU-8 | GFC | CAN-FD Frame ID: 0x17B | 500 | Chamber pressure Engine 8. |
| `ECU9-PT-CHAMBER-RX` | ECU-9 | GFC | CAN-FD Frame ID: 0x18B | 500 | Chamber pressure Engine 9. |

---

### 6.4 ESC Command & Telemetry Signals

> **Physical Implementation:** GFC transmits throttle commands to each ECU via CAN-FD. Each ECU hosts a field-oriented control (FOC) ESC synthesising sinusoidal 3-phase drive waveforms at up to 20 kHz PWM switching frequency. A hardwired ARM/INHIBIT line provides a safety-critical interlock independent of the CAN bus. The PWM duty cycle on this dedicated line must remain between 1000–2000 µs (1500 µs = armed, no thrust). Below 900 µs or above 2100 µs, the ECU unconditionally de-energises the motor bridge.

| Signal ID | Source Subsystem | Destination Subsystem | Protocol / Signal Type | Sampling Rate (Hz) | Description / Range |
|---|---|---|---|---|---|
| `GFC-ESC1-THROT-TX` | GFC Propulsion Manager | ECU-1 | CAN-FD Frame ID: 0x011 | 100 | Throttle command, Engine 1 (LOX and RP-1 motors, co-commanded per mixture-ratio law). Range: 10%–110% NTC (uint8 packed: 0x00=10%, 0xFF=110% NTC). Values 0x00–0x09 interpreted as inhibit; values above 0xFA inhibited by ECU limiter. |
| `GFC-ESC2-THROT-TX` | GFC Propulsion Manager | ECU-2 | CAN-FD Frame ID: 0x012 | 100 | Throttle command, Engine 2. Same encoding. |
| `GFC-ESC3-THROT-TX` | GFC | ECU-3 | CAN-FD Frame ID: 0x013 | 100 | Throttle command, Engine 3. |
| `GFC-ESC4-THROT-TX` | GFC | ECU-4 | CAN-FD Frame ID: 0x014 | 100 | Throttle command, Engine 4. |
| `GFC-ESC5-THROT-TX` | GFC | ECU-5 | CAN-FD Frame ID: 0x015 | 100 | Throttle command, central engine. Central engine throttle authority ≤ ±5% NTC for roll control authority; ≥10% range maintained for attitude trim. |
| `GFC-ESC6-THROT-TX` | GFC | ECU-6 | CAN-FD Frame ID: 0x016 | 100 | Throttle command, Engine 6. |
| `GFC-ESC7-THROT-TX` | GFC | ECU-7 | CAN-FD Frame ID: 0x017 | 100 | Throttle command, Engine 7. |
| `GFC-ESC8-THROT-TX` | GFC | ECU-8 | CAN-FD Frame ID: 0x018 | 100 | Throttle command, Engine 8. |
| `GFC-ESC9-THROT-TX` | GFC | ECU-9 | CAN-FD Frame ID: 0x019 | 100 | Throttle command, Engine 9. |
| `GFC-ARM-INH-PWM-TX` | GFC Safety Controller | All ECUs (hardwired, × 9 lines) | PWM, 50 Hz, 400 V isolated | 50 | Safety-critical ARM/INHIBIT. Pulse width 1500 µs = armed; 1000 µs = inhibit. ECU monitors for loss-of-signal >100 ms; INHIBIT on timeout. Each ECU has dedicated physical line; no bus arbitration. |
| `GFC-ENGINE-FIRE-TX` | GFC Sequencer | All ECUs (hardwired GPIO, ×9) | 28 VDC discrete (opto-isolated) | Event-driven | Engine fire command. Active-HIGH latch. ECU ignition sequencer begins pre-ignition pyrotechnic valve sequencing upon assertion. Signal must be held HIGH ≥ 200 ms to complete ignition handshake. |
| `GFC-ENGINE-ABORT-TX` | GFC Abort Controller | All ECUs (hardwired GPIO, ×9) | 28 VDC discrete (opto-isolated, fail-safe) | Event-driven | Hard abort command. Active-HIGH causes immediate motor power cutoff, propellant valve closure, and pyrotechnic inhibit. Normally-OFF relay; command is a latching action requiring explicit reset from Ground Support Equipment (GSE). |
| `ECU1-ESC-STAT-RX` | ECU-1 | GFC FDIR Engine | CAN-FD Frame ID: 0x201 | 100 | ESC telemetry frame, Engine 1. Payload: actual NTC (uint8), ESC internal temperature (int8 °C, range −40°C to +125°C), DC link voltage (uint16, LSB = 0.1 V, range 200–450 V), ESC fault register (16-bit bitmask). |
| `ECU2-ESC-STAT-RX` | ECU-2 | GFC FDIR Engine | CAN-FD Frame ID: 0x202 | 100 | ESC telemetry frame, Engine 2. Same payload structure. |
| `ECU3-ESC-STAT-RX` | ECU-3 | GFC | CAN-FD Frame ID: 0x203 | 100 | ESC telemetry frame, Engine 3. |
| `ECU4-ESC-STAT-RX` | ECU-4 | GFC | CAN-FD Frame ID: 0x204 | 100 | ESC telemetry frame, Engine 4. |
| `ECU5-ESC-STAT-RX` | ECU-5 | GFC | CAN-FD Frame ID: 0x205 | 100 | ESC telemetry frame, central engine. |
| `ECU6-ESC-STAT-RX` | ECU-6 | GFC | CAN-FD Frame ID: 0x206 | 100 | ESC telemetry frame, Engine 6. |
| `ECU7-ESC-STAT-RX` | ECU-7 | GFC | CAN-FD Frame ID: 0x207 | 100 | ESC telemetry frame, Engine 7. |
| `ECU8-ESC-STAT-RX` | ECU-8 | GFC | CAN-FD Frame ID: 0x208 | 100 | ESC telemetry frame, Engine 8. |
| `ECU9-ESC-STAT-RX` | ECU-9 | GFC | CAN-FD Frame ID: 0x209 | 100 | ESC telemetry frame, Engine 9. |

---

### 6.5 Thermal Management & Health Monitoring

| Signal ID | Source Subsystem | Destination Subsystem | Protocol / Signal Type | Sampling Rate (Hz) | Description / Range |
|---|---|---|---|---|---|
| `ECU1-HEALTH-RX` | ECU-1 | GFC Health Monitor | CAN-FD Frame ID: 0x301 | 10 | Aggregated health frame. Payload: ECU CPU temperature (int8 °C), ECU supply voltage (uint8, LSB=0.05V, nominal 5.0V), watchdog counter (uint8, rolls over at 255), ECU uptime seconds (uint32), cumulative fault count (uint16). |
| `ECU1-VIBR-RX` | ECU-1 (Accelerometer on motor body) | GFC Structural Health | CAN-FD Frame ID: 0x302 | 1000 | High-bandwidth vibration acceleration on LOX motor housing. Range: ±50 g, resolution 0.01 g. Used to detect pump cavitation onset and bearing wear. Transmitted as compressed delta-coded 16-byte payload. |
| `ECU2-HEALTH-RX` | ECU-2 | GFC | CAN-FD Frame ID: 0x311 | 10 | Aggregated health, Engine 2. Same payload. |
| `ECU3-HEALTH-RX` to `ECU9-HEALTH-RX` | ECU-3 through ECU-9 | GFC | CAN-FD Frame IDs: 0x321–0x381 | 10 | Aggregated health frames for Engines 3–9. Same payload format. |
| `GFC-THERM-SETPT-TX` | GFC Thermal Manager | All ECUs | CAN-FD Frame ID: 0x030 | 1 | Broadcast thermal management setpoints. Payload: maximum winding temperature limit (int8 °C), minimum LOX inlet temperature alert (int8 °C), thermal derating curve select (uint8, 0x00=nominal, 0x01=conservative, 0x02=emergency). |

---

### 6.6 System Synchronisation & Timing

| Signal ID | Source Subsystem | Destination Subsystem | Protocol / Signal Type | Sampling Rate (Hz) | Description / Range |
|---|---|---|---|---|---|
| `GFC-TIMESYNC-TX` | GFC Master Clock | All ECUs | CAN-FD Frame ID: 0x001 (highest priority) | 1000 | IEEE 1588v2 PTP sync message transported over CAN-FD. Enables sub-microsecond ECU clock alignment for synchronised throttle command execution. Time-of-validity field: 64-bit TAI nanoseconds. |
| `GFC-PPS-TX` | GFC GPS Disciplined Oscillator | All ECUs (hardwired) | RS-422 PPS pulse, 3.3 V, 100 ms width | 1 | Hardware pulse-per-second from NavIC-disciplined OCXO. Rising edge marks UTC second boundary. ECUs use this as absolute time reference for data timestamping. Accuracy: ±50 ns RMS. |
| `ECU1-TIMESYNC-ACK-RX` | ECU-1 | GFC Master Clock | CAN-FD Frame ID: 0x401 | 1000 | PTP follow-up / delay-request response from ECU-1. Contains measured propagation delay for offset correction computation. |
| `ECU2–ECU9 TIMESYNC-ACK-RX` | ECU-2 through ECU-9 | GFC | CAN-FD Frame IDs: 0x402–0x409 | 1000 | Same as ECU-1 time-sync ACK. |

---

## 7. Protocol Specifications

### 7.1 CAN-FD Bus Parameters

| Parameter | Value |
|---|---|
| Standard | ISO 11898-1:2015 |
| Arbitration Phase Bit Rate | 500 kbit/s |
| Data Phase Bit Rate | 8 Mbit/s |
| Maximum Payload Length | 64 bytes |
| Bus Termination | 120 Ω at both endpoints |
| Node Count | 11 (GFC ×1, ECUs ×9, OBC ×1 as listener) |
| Bus Redundancy | Dual independent busses (Bus A primary, Bus B shadow), ECU auto-failover <5 ms |
| Frame Format | CANFD Extended (29-bit CAN ID) |
| Error Confinement | ISO CAN error state machine; passive-error nodes flagged to GFC via health frame |
| Acceptance Filter | Hardware mask-based per ECU; ECU accepts only GFC-originating command frames |

### 7.2 PWM ARM/INHIBIT Line

| Parameter | Value |
|---|---|
| Signal Type | RC-servo-style PWM |
| Carrier Frequency | 50 Hz (20 ms period) |
| Armed Pulse Width | 1500 µs ± 50 µs |
| Inhibit Pulse Width | 1000 µs ± 50 µs |
| Maximum Thrust Command via PWM | Not applicable (thrust commanded via CAN-FD only) |
| Loss-of-Signal Timeout | 100 ms → ECU transitions to INHIBIT |
| Voltage Level | 3.3 V logic (tolerant to 5 V) |

### 7.3 Discrete GPIO Lines

| Signal | Logic Level | Drive Type | Isolation | Notes |
|---|---|---|---|---|
| ENGINE-FIRE | 28 VDC HIGH / 0 V LOW | Push-pull, 500 mA source | Opto-isolated at ECU input, 1500 V isolation | Minimum pulse width 200 ms for valid ignition handshake |
| ENGINE-ABORT | 28 VDC HIGH | Push-pull, 500 mA | Opto-isolated, fail-safe (ECU asserts ABORT if line is floating or open) | Normally de-energised; hardwired to pyrotechnic inhibit circuit |

---

## 8. Power Interface

| Rail | Nominal Voltage | Source | Consumer | Max Load | Connector |
|---|---|---|---|---|---|
| High Voltage (HV) Bus | 400 VDC | Battery Pack (Li-S, 120 kWh) | ECU Motor Drives (×9) | 270 kW peak | MIL-SPEC HV connector per ECU |
| Low Voltage (LV) Control | 28 VDC | Regulated from HV via DC-DC | ECU Logic, Sensors, CAN Transceivers | 50 W per ECU | MIL-DTL-38999 |
| GFC Logic Supply | 5 VDC ± 1% | Dual-redundant 28V→5V converter | GFC CPU & I/O | 25 W | Internal |

---

## 9. Fault Detection, Isolation & Recovery (FDIR) Interfaces

The GFC implements a three-tier FDIR architecture:

| FDIR Level | Trigger Condition | GFC Action | ECU Action | Recovery Path |
|---|---|---|---|---|
| **Level 1 — Warning** | Single sensor off-nominal (>1σ from expected); minor CAN frame latency (>2 missed frames) | Log event, increment fault counter, alert OBC | None (continue operation) | Auto-clear if condition resolves within 2 seconds |
| **Level 2 — Caution** | Over-current sustained >50 ms; chamber pressure deviation >10%; encoder dropout >100 ms | Activate differential throttle compensation on adjacent engines; increase GNC loop bandwidth | Attempt ESC reset; apply current de-rate | CCB review post-flight |
| **Level 3 — Engine Out** | Chamber pressure <40 bar during mainstage; catastrophic encoder failure; thermal shutdown | Assert ENGINE-ABORT on faulted engine; redistribute thrust to remaining engines per pre-computed engine-out trajectory table | Motor bridge disabled; propellant valves commanded closed | No in-flight recovery; contingency trajectory engaged |
| **Level 4 — Vehicle Abort** | ≥3 simultaneous engine-out events; structural health anomaly; GFC watchdog expiry | Assert ABORT on all engines; activate Flight Termination System (FTS) if appropriate | All ECUs commanded to safe state | Post-abort investigation |

---

## 10. Verification & Validation Matrix

| Requirement ID | Requirement Summary | Verification Method | Verification Status |
|---|---|---|---|
| RUPAK-ICD-REQ-001 | All CAN-FD signals shall be received within 2× nominal sample period before stale-data flag | Analysis + Test (HIL bench) | Open |
| RUPAK-ICD-REQ-002 | PWM ARM/INHIBIT loss-of-signal timeout shall be ≤100 ms | Test (timing analyser on bench) | Open |
| RUPAK-ICD-REQ-003 | Engine-ABORT GPIO shall assert hard-interlock within 5 ms of command | Test (oscilloscope verification on integrated harness) | Open |
| RUPAK-ICD-REQ-004 | CAN-FD Bus B shall assume full primary function within 5 ms of Bus A failure | Test (fault injection on HIL bench) | Open |
| RUPAK-ICD-REQ-005 | Chamber pressure telemetry shall have end-to-end latency ≤4 ms from sensor ADC to GFC buffer | Analysis + Test | Open |
| RUPAK-ICD-REQ-006 | Time-synchronisation accuracy across all ECU nodes shall be ≤1 µs RMS | Test (logic analyser, IEEE 1588 test tool) | Open |
| RUPAK-ICD-REQ-007 | All signals shall function across operating temperature range −40°C to +85°C (electronics enclosure) | Environmental test (thermal vacuum chamber) | Open |

---

## 11. Configuration Management & Change Log

| Revision | Date | Author | Description of Change |
|---|---|---|---|
| Rev A | 2024-01-15 | GNC/Prop Integration | Initial release for internal design review |
| Rev B | 2024-06-01 | GNC/Prop Integration | Added vibration signal ECU1-VIBR-RX; revised CAN-FD bit rate from 5 to 8 Mbit/s; added timing sync section |
| Rev C | 2025-07-01 | Systems Engineering | Incorporated CCB-ECN-047: added FDIR Level-4 definition; updated thermal derating thresholds per Shakti-1E endurance test results; added GPIO isolation voltage spec |

---

*End of Document — RUPAK-ICD-PROP-AVI-001 Rev C*

*All numerical values are provisional and subject to update following Shakti-1E Series Qualification Testing.*
