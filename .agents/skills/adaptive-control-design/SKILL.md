---
name: adaptive-control-design
description: Compare and design classical, robust, adaptive, and model-predictive control architectures for the hydraulically pushed milling process using pump RPM and spindle RPM commands with pressure, resolver speed, and Iq-derived torque feedback. Use for hierarchy, state machine, constraints, tuning, observers, anti-windup, and controller comparison. Do not choose an advanced method before establishing a classical baseline.
---

# Adaptive Control Design

## Control objective

Regulate cutting load/productivity without stalling or damaging the bit while respecting the pusher's asymmetric hydraulic dynamics.

Typical manipulated variables:

- pump RPM setpoint;
- spindle/bit RPM setpoint.

Typical measured/estimated feedback:

- bit speed;
- Iq-derived spindle torque;
- hydraulic pressure;
- optional pusher displacement;
- estimated WOB/contact force.

## Recommended hierarchy

### Layer 0 — embedded drive loops

Assume PMSM current/FOC and speed loops exist for both motors unless the project explicitly asks to design them.

### Layer 1 — fast protection and limits

- spindle torque/current limits;
- pump pressure limits;
- speed limits;
- rate limits;
- anti-windup;
- stall/near-stall detection.

### Layer 2 — hydraulic / force regulation

Regulate pressure or estimated WOB with pump RPM.

Use asymmetric logic because decreasing pump RPM cannot instantly decrease contact force.

### Layer 3 — milling-load regulation

Use torque/load error to adjust WOB/pressure target and, more slowly or by schedule, bit RPM.

### Layer 4 — supervisory hybrid state machine

Modes may include:

- approach;
- contact acquisition;
- normal milling;
- pressure relaxation;
- overload recovery;
- stall recovery;
- completion/retract.

## Baseline controller

Always create a gain-scheduled PI/PID baseline with:

- filtered measurement;
- feedforward from operating point;
- anti-windup;
- asymmetric slew limits;
- integral hold/reset across modes;
- bumpless transfer.

This is the benchmark all advanced methods must beat.

## Candidate A — LADRC/ADRC

Strong candidate for an outer load/pressure loop when:

- model mismatch is significant;
- a low-order input-output model is available;
- disturbances can be lumped into an extended state.

Evaluate carefully against measurement noise and tooth-passing ripple. Observer bandwidth must not chase cutting ripple that the actuator cannot control.

## Candidate B — constrained MPC / NMPC

Strong candidate when:

- pressure relaxation and pump limits are important;
- commands are coupled;
- model predictions are credible;
- constraints and asymmetric dynamics dominate behavior.

MPC can explicitly reason about future pressure/force and actuator limits. Use a simpler controller model than the truth plant.

## Candidate C — adaptive/gain-scheduled PI

Likely practical and robust if the plant changes primarily through identifiable operating-point parameters. Scheduling variables may include engagement, pressure, torque level, or identified plant gain/time constant.

## Candidate D — MRAC

Use as a research comparator only when:

- a meaningful desired reference model exists;
- uncertainty structure/matching assumptions are defensible;
- the controlled state/output is adequately observable;
- actuator saturation and hybrid contact modes are handled explicitly.

Do not treat MRAC as automatically superior because the plant is uncertain.

## Spindle RPM strategy

Start with machining-informed RPM scheduling based on material/tool operating envelope.

Allow adaptive trimming only within safe bounds.

Do not rapidly modulate spindle RPM tooth-to-tooth unless there is a specific validated purpose; spindle inertia and drive bandwidth generally make pump/WOB the more direct slow load-control channel.

## Torque filtering

Separate:

- mean/low-frequency torque used for load control;
- ripple/features used for diagnostics or contact detection.

The outer controller should not fight uncontrollable tooth-passing ripple.

## Performance metrics

Compare controllers on:

- peak torque;
- RMS/mean torque tracking;
- pressure overshoot;
- stall rate;
- material removal / completion time;
- actuator activity;
- time spent in overload/relaxation;
- robustness to parameter/material changes;
- recovery from entry transients.

See `references/REFERENCE.md`.
