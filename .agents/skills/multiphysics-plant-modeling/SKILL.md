---
name: multiphysics-plant-modeling
description: Define the coupled mechanical, hydraulic, rotational, contact, material-removal, and sensor dynamics for a milling plant. Use when selecting states, equations, fidelity levels, numerical integration strategy, hybrid modes, energy/sign conventions, or plant interfaces. Do not select final controller gains here.
---

# Multiphysics Plant Modeling

## Objective

Build a plant model whose states and couplings are physically interpretable and calibratable from lab data.

## Start with a state inventory

Candidate states include:

- pusher/rod axial position and velocity;
- hydraulic pressure(s) or load pressure;
- optional pump flow state if pump/transmission dynamics matter;
- spindle angular speed;
- optional pump-motor angular speed;
- shaft torsional twist / relative angular velocity if compliance is relevant;
- friction internal state if using LuGre-type friction;
- contact indentation/compliance state if not purely rigid;
- target geometry/material-removal state;
- thermal/wear state only when required by experiment duration;
- sensor filter/delay states if they materially affect control.

Separate **physical truth states** from **measured outputs**.

## Use hybrid modes

At minimum consider:

- free approach;
- first contact;
- active cutting;
- stick / static friction;
- sliding;
- insufficient chip load / rubbing;
- pressure relaxation;
- spindle overload/stall region;
- completion or loss of contact.

Mode transitions must be event-driven from physical conditions, not arbitrary timers.

## Coupling order

A credible step conceptually resolves:

1. actuator command hold;
2. pump/motor response;
3. hydraulic flow and pressure;
4. pusher force and axial motion;
5. contact/engagement geometry;
6. chip load / material removal;
7. cutting force and torque;
8. spindle/shaft dynamics;
9. target geometry update;
10. sensor emulation.

Because several terms are mutually coupled, implementation may require implicit/iterative evaluation or sufficiently small integration steps. Do not assume a single explicit algebraic pass is always stable.

## Fidelity hierarchy

### Level A — commissioning/control model

Use averaged cutting torque/axial force over tooth passages. Keep nonlinear engagement, friction, pressure, and actuator limits.

### Level B — disturbance-rich model

Add tooth-passing ripple, eccentricity/runout, stronger nonlinear friction, parameter variability, sensor noise/delay.

### Level C — reference model

Add refined cutter-target geometry, structural modes, richer pump map, and target/material heterogeneity if supported by data.

Do not add finite-element detail unless it answers a control-relevant question.

## Numerical model requirements

- explicit SI units internally;
- documented sign conventions;
- conservation/energy sanity checks;
- event detection at contact and limits;
- solver-step convergence tests;
- deterministic random seeds for stochastic disturbances;
- parameter bounds with physical meaning.

For stiff combinations of compressibility, contact stiffness and drivetrain stiffness, assess an implicit/stiff ODE method.

## Interface contract

Plant input should be actuator-level commands such as:

- pump RPM setpoint;
- spindle/bit RPM setpoint;
- optional enable/mode commands.

Plant output to controller should be **sensor-equivalent channels**, e.g.:

- pressure;
- resolver-derived bit speed;
- q-axis current / torque estimate;
- pusher displacement if available;
- estimated WOB only through a defined estimator.

The plant may expose truth states separately for diagnostics, but the controller must not silently use them.

See `references/REFERENCE.md`.
