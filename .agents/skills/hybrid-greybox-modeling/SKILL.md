---
name: hybrid-greybox-modeling
description: Build physics-informed grey-box and hybrid residual models for the milling plant, combining known hydraulic/mechanical/cutting equations with data-driven corrections while preserving physical constraints, contact modes, interpretability, and controller usefulness. Use after the core physical model and calibration data exist.
---

# Hybrid Grey-Box Modeling

## Default architecture

Use:

`physics model + calibrated parameters + constrained residual correction`

rather than:

`black-box model replaces the plant`.

## Where residual learning is most useful

Candidate residuals include:

- cutting-force coefficient variation with material/tool condition;
- pressure-dependent friction mismatch;
- leakage not captured by a simple law;
- unmodeled drivetrain/load losses;
- sensor bias/lag correction.

## Where residual learning should be constrained

Examples:

- cutting-force residual must not create cutting force when there is no engagement;
- learned pressure dynamics must not violate obvious pressure limits or create energy;
- learned WOB should respect unilateral contact;
- learned torque correction should remain within physically defensible bounds.

## Residual structures

Prefer in this order:

1. parameter scheduling/maps;
2. low-order nonlinear residual regression;
3. sparse dynamic residual model (SINDy);
4. NARMAX/NARX;
5. GP/neural residual when data volume and validation justify it.

Do not jump to a neural state-space model merely because it fits training data better.

## Hybrid mode awareness

Different model residuals may be required for:

- no contact;
- contact/cutting;
- pressure relaxation;
- near-stall.

A single smooth correction across all modes can blur physically meaningful switching.

## Calibration workflow

1. fit/measure physical parameters;
2. compute residuals on estimation data;
3. inspect residual structure;
4. choose the smallest residual model class that explains repeatable structure;
5. train with physical bounds/regularization;
6. validate on unseen material/engagement/pressure conditions;
7. stress test extrapolation.

## Controller use

Maintain two models:

- **truth/reference plant** for simulation and robustness testing;
- **controller model** that is simpler and intentionally imperfect.

Do not let an MPC controller use the exact same hidden residual model as the plant unless explicitly testing an upper-bound case.

## Online adaptation

Online residual/parameter adaptation may be evaluated later, but gate it with:

- observability/identifiability;
- excitation;
- bounded updates;
- supervisory logic;
- fallback controller.

See `references/REFERENCE.md`.
