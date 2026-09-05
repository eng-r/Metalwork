---
name: hydraulic-pusher-modeling
description: Model and calibrate the variable-speed centrifugal-pump pusher, hydraulic pressure dynamics, fluid compressibility, leakage, rod motion, pressure-dependent seal friction, asymmetric load/unload behavior, and conversion from pressure to axial force/WOB. Use for hydraulic subsystem equations, parameters, tests, or pressure/force estimation.
---

# Hydraulic Pusher Modeling

## Core modeling principle

Pump RPM is **not** pressure. A centrifugal pump creates a pressure-flow characteristic. The actual operating point is determined jointly by pump speed, hydraulic restriction/compliance, leakage, rod motion, contact load and any relief/check-valve behavior.

## Minimum pump model

Represent a family of head/pressure versus flow curves, parameterized by speed.

A useful first calibrated form is a polynomial map such as:

- pressure/head term proportional approximately to speed squared;
- flow term proportional approximately to speed;
- coefficients fitted to pump data.

Use affinity laws only to scale a known pump map and only within their assumptions. Prefer measured pump curves when available.

## Hydraulic continuity

For each relevant trapped volume:

- include effective bulk modulus;
- include volume as a function of rod/piston position when applicable;
- include pump inflow/outflow;
- include displacement flow due to rod motion;
- include internal/external leakage if present;
- include relief/check-valve flows if present.

The pressure rate must emerge from continuity, not from an arbitrary first-order pressure lag unless that lag has been deliberately identified as a reduced model.

## Force balance

Axial hydraulic force is based on effective pressure area(s). Axial rod dynamics should include:

- moving mass/effective inertia;
- hydraulic force;
- seal friction;
- structural/contact force;
- gravity or external axial bias if relevant;
- end stops / travel constraints.

## Friction

Do not use only viscous damping if lab behavior shows stick-slip or breakaway.

Start with a Stribeck + Coulomb + viscous model if simplicity is important.

Escalate to LuGre or a pressure-dependent/hysteretic seal-friction model when needed by data.

Because hydraulic-cylinder seal friction can depend on chamber pressure and lubrication state, identify friction across pressure and velocity operating regions rather than as one constant.

## Crucial asymmetry

The pusher is effectively unilateral in the cutting direction:

- it can build compressive force;
- reducing pump RPM does not instantaneously remove stored hydraulic pressure;
- pressure may relax through leakage, compliance, rod motion and continued material removal;
- the system generally cannot command a symmetric negative axial force.

The plant and controller must preserve this asymmetry.

## Contact coupling

When the cutter is in contact, pressure is not identical to WOB. WOB depends on hydraulic force minus friction/inertia and is coupled to contact/cutting mechanics.

A pressure-based WOB soft sensor must therefore be dynamic and calibrated.

## Calibration tests

Plan separate tests for:

1. pump head-flow-speed map;
2. effective bulk modulus / trapped volume;
3. leakage versus pressure;
4. rod friction versus velocity and pressure;
5. no-contact pressure transients;
6. contact compression / compliance;
7. pressure relaxation after RPM reduction.

See `references/REFERENCE.md`.
