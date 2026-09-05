---
name: sysid-doe-calibration
description: Design laboratory experiments and system-identification workflows for the milling simulator, including excitation design, staged parameter identification, grey-box fitting, identifiability checks, validation datasets, nonlinear models, uncertainty, and controller-relevant model selection. Use before tuning adaptive or model-based controllers from lab data.
---

# SysID and DoE Calibration

## Principle

Identify **subsystems and parameter groups in stages**. Do not ask an optimizer to estimate pump-map coefficients, bulk modulus, friction, compliance, cutting coefficients and sensor lags simultaneously from one cutting test unless an identifiability study proves that it can.

## Staged campaign

### Stage 1 — sensors and drives

Identify:

- sample delays;
- resolver speed dynamics;
- Iq/torque-estimate scaling and lag;
- pump and spindle speed-loop dynamics.

### Stage 2 — pump/hydraulics without cutting

Identify:

- pump head-flow-speed map;
- leakage versus pressure;
- effective bulk modulus;
- trapped-volume effects;
- pressure-relaxation dynamics.

### Stage 3 — pusher mechanics/friction

Identify:

- moving mass/equivalent inertia;
- Coulomb/Stribeck/static friction;
- pressure dependence of seal friction;
- structural compliance.

### Stage 4 — cutting coefficients

Use controlled cuts to identify mechanistic shear/edge coefficients and their dependence on speed, feed/WOB, engagement and material/tool condition.

### Stage 5 — coupled residuals

Only after the physics parameters are credible, identify residual model terms for remaining systematic mismatch.

## Excitation design

For linear/local dynamic identification, consider:

- PRBS;
- multisine;
- chirp;
- stepped operating points.

For nonlinear/static and interaction effects, use:

- factorial/fractional factorial screening;
- central composite / response-surface designs;
- Latin hypercube / space-filling designs where appropriate.

Respect actuator, tool and material limits.

## Persistency and bandwidth

Ensure input excitation covers the controller-relevant bandwidth and enough operating regions.

A single step is rarely sufficient for full dynamic identification.

## Dataset discipline

Split data into:

- parameter estimation/training;
- validation;
- final challenge/test scenarios.

Do not tune the model repeatedly against the final validation set.

## Grey-box identification

Prefer nonlinear grey-box parameter estimation when the governing equations are known but parameters are uncertain.

Use bounds and parameter transformations to maintain physical validity.

## Nonlinear residual alternatives

For residual dynamics or benchmark black-box models, consider:

- NARMAX/NARX via SysIdentPy;
- SINDy/PySINDy for sparse interpretable residual dynamics;
- Gaussian-process or small neural residual models only with adequate data.

## Identifiability checks

Track:

- parameter correlation;
- profile likelihood / confidence;
- sensitivity of outputs to parameters;
- whether multiple parameter sets yield nearly identical output.

Freeze weakly identifiable parameters to measured/prior values rather than allowing arbitrary compensation.

## Model selection criterion

The best model is not the one with the smallest training error. Prefer the simplest model that:

- passes residual/validation checks;
- predicts held-out regimes;
- preserves physical behavior;
- is usable for the intended controller.

See `references/REFERENCE.md`.
