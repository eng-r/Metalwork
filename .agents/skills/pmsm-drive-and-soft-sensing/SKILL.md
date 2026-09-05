---
name: pmsm-drive-and-soft-sensing
description: Model PMSM spindle and pump drives with resolver feedback, FOC torque/current relationships, speed-loop dynamics, actuator saturation, q-axis-current torque estimation, sensor filtering/delay/noise, and WOB/torque soft sensing. Use when defining what the plant exposes to the controller or calibrating motor/sensor channels.
---

# PMSM Drive and Soft Sensing

## Scope

The simulator usually does not need switching-level inverter simulation. Model the drive only to the fidelity needed by the milling supervisory-control bandwidth.

## PMSM torque

Use the standard dq torque relationship:

- permanent-magnet torque proportional to q-axis current;
- include reluctance torque for salient PMSMs when `Ld != Lq`;
- for surface PMSM with approximately `Ld = Lq`, torque is approximately proportional to `Iq`.

Use the actual motor pole pairs, PM flux linkage and current scaling.

## Drive hierarchy

Represent separately:

1. current/FOC dynamics if relevant;
2. motor electromagnetic torque;
3. rotor/spindle inertia and damping;
4. speed-control loop dynamics;
5. mechanical shaft compliance/load.

If the commercial/embedded FOC speed loop is much faster than the milling outer loop, a calibrated closed-loop actuator model is acceptable.

## Resolver

Model:

- rotor angle/speed measurement;
- sample rate;
- quantization/resolution if material;
- filtering/differentiation delay;
- bias/noise or occasional artifacts if observed.

Do not give the controller the exact simulated angular speed if the real system only has a filtered resolver estimate.

## Torque from Iq

Expose both:

- true mechanical/electromagnetic torque for diagnostics;
- controller-visible torque estimate derived from measured `Iq`, motor parameters and filtering.

Include bias from loss/friction and dynamic lag if laboratory comparison shows it.

## Pump motor

The pump PMSM should likewise have finite speed-loop dynamics and saturation. Pump RPM setpoint must not be converted directly into hydraulic pressure.

## WOB soft sensor

A useful soft sensor may fuse:

- line pressure;
- pusher geometry/effective area;
- rod acceleration/velocity;
- friction estimate;
- contact state.

Start physics-based. Add estimator correction only when justified by data.

## Observers

Potential candidates:

- low-pass / band-limited torque estimator;
- Kalman/EKF/UKF for coupled pressure-force states;
- disturbance observer / ESO for load disturbance;
- moving-horizon estimator if the nonlinear grey-box model is strong enough.

All observer tuning must reflect real sample rates and delays.

See `references/REFERENCE.md`.
