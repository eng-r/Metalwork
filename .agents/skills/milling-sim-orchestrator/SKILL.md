---
name: milling-sim-orchestrator
description: Plan and coordinate development of a hybrid-physics milling simulator with hydraulic pusher, PMSM spindle/pump drives, machining contact, SysID/DoE calibration, adaptive control, deterministic co-simulation, and web visualization. Use for architecture, work breakdown, design-document generation, or when several specialist skills must be composed. Do not use it as the source of detailed domain equations; delegate those to the specialist skills.
---

# Milling Simulator Orchestrator

## Mission

Turn the project request into a staged engineering workflow that produces a credible simulator and control-design environment rather than a visually convincing but physically weak demo.

## First rule: preserve separation of concerns

Treat these as separate concerns with explicit interfaces:

1. physical plant truth model;
2. sensor/actuator emulation;
3. controller;
4. estimator / soft sensors;
5. simulation scheduler/backplane;
6. calibration and SysID;
7. experiment/scenario definition;
8. data logging and metrics;
9. web UI.

Never let the UI become the simulator clock or source of truth.

## Skill routing

Read these skills when needed:

- `../multiphysics-plant-modeling/SKILL.md`
- `../hydraulic-pusher-modeling/SKILL.md`
- `../machining-cutting-mechanics/SKILL.md`
- `../pmsm-drive-and-soft-sensing/SKILL.md`
- `../sysid-doe-calibration/SKILL.md`
- `../hybrid-greybox-modeling/SKILL.md`
- `../adaptive-control-design/SKILL.md`
- `../cosim-software-architecture/SKILL.md`
- `../validation-and-experimentation/SKILL.md`

## Required planning sequence

### Phase 0 — clarify model scope without blocking progress

Resolve assumptions explicitly in the design instead of hiding them. At minimum flag:

- actual NiCr / nickel-alloy grade and hardness;
- cutter diameter, flute/tooth count, helix/rake geometry, cutter material/coating;
- whether "cylindrical bit" means end mill, rotary burr, hole-mill, abrasive tool, or custom milling head;
- pusher hydraulic topology, effective piston area, line volume, relief/check valves, leakage path;
- pump map or test points;
- rod/pusher stroke and equivalent moving mass;
- shaft/rod torsional and axial compliance;
- actual controller and telemetry sample rates;
- available pressure, resolver, current and displacement signals.

If unknown, create parameter placeholders and uncertainty ranges. Do not invent precise values.

### Phase 1 — define minimal credible plant

The first plant must contain:

- variable-speed centrifugal pump pressure-flow behavior;
- fluid compressibility and trapped volume;
- leakage / pressure relaxation;
- rod/piston force balance;
- static + dynamic seal friction;
- unilateral contact with target;
- shaft/bit rotational dynamics;
- mechanistic or calibrated cutting torque/force;
- target material removal / engagement update;
- PMSM actuator and sensor approximations adequate for the control bandwidth.

### Phase 2 — establish fidelity levels

Require at least two model fidelities:

- **control-oriented model**: averaged cutting load, fast enough for Monte Carlo, SysID, controller tuning;
- **high-fidelity reference model**: tooth/engagement-resolved or otherwise richer, used to test robustness and model mismatch.

A controller must not be validated only against the exact same equations used to design it.

### Phase 3 — calibrate before advanced control

Use staged DoE/SysID to identify pump, hydraulics, friction, compliance, cutting coefficients and sensor dynamics.

Do not fit one giant black-box model to all data unless it is only a benchmark.

### Phase 4 — establish baseline controller

Always implement/evaluate a classical baseline before adaptive methods:

- spindle speed regulation;
- pump/pressure or force regulation;
- torque limiting;
- anti-windup;
- asymmetric command/rate constraints;
- supervisory contact/cutting/stall/relaxation state machine.

Then compare advanced approaches under identical scenarios.

### Phase 5 — advanced control candidates

Prioritize:

1. constrained model-predictive outer control if the calibrated model is adequate;
2. LADRC/ADRC as a robust lower-model-dependence comparator;
3. gain-scheduled/adaptive PI as practical baseline extension;
4. MRAC as a research comparator when a valid reference-model/matching structure can be justified.

Do not choose a controller by name or novelty.

### Phase 6 — software architecture

The simulation kernel must run headless. UI is an adapter around it.

Use deterministic virtual time for numerical studies. Explicitly model sample periods and holds.

### Phase 7 — validation

Require open-loop, component-level, integrated-plant and closed-loop validation with held-out data and scenarios.

## Planning deliverables

Before code, produce:

- assumptions and unknowns table;
- plant state vector;
- input/output signal contract;
- model hierarchy and equations list;
- parameter table with source/calibration method;
- controller candidate matrix;
- DoE/SysID plan;
- simulation timing diagram;
- software component diagram;
- verification/validation matrix;
- implementation milestones.

When the user later asks for implementation, create project `README.md` and `design.md` before major coding.

## Failure modes to avoid

- treating pump RPM as pressure directly;
- treating pressure as instantaneous WOB;
- allowing negative "pulling" contact force when the mechanism only pushes;
- using handbook cutting data as identified force coefficients;
- feeding ideal internal plant states to the controller when only sensors are available;
- tuning and validating on the same plant model and parameter set;
- free-running threads that make numerical results nondeterministic;
- jumping to MRAC/AI before a baseline controller exists.

See `references/REFERENCE.md`.
