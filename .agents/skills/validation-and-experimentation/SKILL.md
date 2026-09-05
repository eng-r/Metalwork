---
name: validation-and-experimentation
description: Verify and validate the milling simulator and controllers using component tests, limiting cases, held-out lab data, solver convergence, uncertainty sweeps, Monte Carlo, controller stress tests, and traceable acceptance metrics. Use before claiming a plant model or controller is credible.
---

# Validation and Experimentation

## Separate verification from validation

**Verification:** Did we implement the equations/architecture correctly?

**Validation:** Do the model outputs agree with physical behavior and lab data well enough for the intended control-design use?

## Component verification

### Pump/hydraulics

Check:

- zero-speed/no-flow limiting behavior;
- pressure response to blocked/moving actuator;
- affinity/map consistency;
- mass continuity;
- leakage-driven relaxation;
- physical pressure bounds.

### Pusher mechanics

Check:

- force balance;
- breakaway/stiction;
- no negative unilateral contact force;
- end stops;
- energy dissipation from friction.

### Machining

Check:

- zero force with zero engagement;
- force scales sensibly with chip load/engagement;
- torque sign and units;
- material removal does not occur without cutting;
- averaged model agrees with the mean of tooth-resolved model under matched conditions.

### PMSM/sensing

Check:

- Iq-to-torque scaling;
- speed-loop step response;
- saturation;
- sensor lag/noise/quantization.

## Solver verification

Run step-size/tolerance convergence studies.

Stiff hydraulic/contact configurations may require stiff solvers.

Do not accept a controller result that changes materially when the integration tolerance is tightened.

## Data validation

Use held-out experiments for:

- pressure transient;
- rod motion;
- spindle torque;
- engagement/load;
- material removal/completion.

Validate both transient shape and key scalar metrics.

## Uncertainty

Assign ranges/distributions to uncertain parameters such as:

- bulk modulus;
- leakage;
- friction;
- cutting coefficients;
- shaft stiffness;
- sensor delay;
- motor torque constant.

Use Monte Carlo or structured worst-case sweeps.

## Controller challenge scenarios

At minimum:

- first contact at different initial pressures;
- abrupt engagement increase;
- hard/soft material segment;
- elevated friction;
- low leakage / slow pressure relaxation;
- torque-sensor bias;
- resolver/signal delay;
- near stall;
- recovery after overload.

## Acceptance metrics

Plant:

- normalized RMSE or fit on held-out channels;
- peak/transient error;
- phase/time-delay error;
- correct mode-transition sequence;
- physically correct limiting behavior.

Controller:

- peak/RMS torque;
- pressure overshoot;
- completion time/material removal;
- stall/overload incidence;
- control effort;
- recovery time;
- robustness margin across uncertainty cases.

## Model hierarchy test

Controller designed on the control-oriented model must be tested on the higher-fidelity plant and uncertain parameter sets.

See `references/REFERENCE.md`.
