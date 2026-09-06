# Metalwork holistic plant/control rebaseline

Base reviewed: `eng-r/Metalwork` master after commit `329aa42450b8f48405c5463d2074ef793d71742f` (`fixing step-down response`).

This package intentionally replaces the incremental symptom-fix approach with one causal plant and one fair controller-comparison architecture.

## Physical causal chain

The nominal control-oriented plant is now:

`pump RPM -> hydraulic pressure -> pusher force -> engagement/WOB -> ToB + ROP -> removed surface -> engagement`

The key equations implemented are conceptually:

- `F_wob = k_contact(delta) * delta + c_contact * positive(delta_dot)`
- `T_bit = c_TF * F_wob + U * MRR / omega + T_edge + disturbances`
- `ROP = ROP_ref * (WOB/WOB_ref)^m * (RPM/RPM_ref)^n / hardness^q * chip_efficiency`
- `P_dot = beta_eff/V * (Q_pump - Q_bypass - Q_leak - A_p*x_dot)`

The old dominant `rubbing_torque_coeff * contact_area` behavior is removed as the mean ToB source. Once contact area saturated it gave almost no pressure->ToB control authority; this was the main reason upward ToB steps could not respond.

## Controller architecture

Both PID and LADRC now share exactly the same two feedforward layers:

1. desired ToB -> nominal target WOB -> nominal pressure feedforward;
2. pressure reference -> inverse centrifugal-pump shutoff-head map -> nominal pump RPM.

The outer ToB governor adds a PI pressure trim. It no longer relies on a deliberately slow 5 bar/s integrator/ramp to discover the correct pressure.

- PID: pressure PID trim around pump feedforward.
- LADRC: first-order pressure ESO/LADRC trim around the same pump feedforward.

The safety FSM no longer changes overload thresholds when the operator changes ToB. It is a supervisor, not a second load controller.

## Asymmetric hydraulics

Downward load commands remain physically slower because the pump cannot create negative flow. A lower pressure reference reduces the required pump head, but trapped pressure unloads through:

- continued material removal reducing engagement;
- forward pusher motion increasing chamber volume;
- small calibrated bypass/leakage.

The pump may still rotate at the lower feedforward speed while chamber pressure is above its shutoff head; the check-valve/non-reverse pump map then produces zero forward flow. This is physically different from repeatedly forcing motor RPM to zero.

## Numerical coupling

The previous RK4 implementation calculated cutting force once and held it fixed through all four pressure/position RK stages. This is inconsistent after adding stiff contact mechanics.

The new plant evaluates axial contact reaction at every RK stage using stage-specific `x` and `v`, while slow material/chip states are frozen during that 1 ms integration interval. The material state advances once after the mechanical RK step.

## WOB observer

Static seal friction is set-valued and cannot be uniquely inferred from pressure + displacement alone. The soft WOB observer now uses a documented calibrated fraction of the pressure-dependent breakaway bound while the rod is stuck, and the dynamic Stribeck/Coulomb model while moving.

## Disturbances

Disturbances no longer define the nominal waveform.

- spatial hard spots remain deterministic for a seed;
- chip load has a bounded continuous state;
- chip jams are stochastic with a refractory interval rather than threshold-driven periodic relaxation oscillations;
- vibration amplitude is reduced to a perturbation around nominal load.

## UI / telemetry

The WebSocket stream still runs at ~30 Hz, but actual waveforms are now averaged over the most recent 33 ms of 1 kHz physics history before streaming. This prevents a raw ~58 Hz structural mode from aliasing into fake low-frequency chart motion.

Setpoints and references remain latest-sample values so a ToB step stays visually sharp.

The monitor now exposes:

- pressure reference and actual;
- ToB SP, Iq estimate, truth;
- nominal maximum achievable ToB when hydraulic authority is insufficient;
- pump feedforward, pump command, pump actual;
- target WOB estimate;
- explicit `SETPOINT LIMITED` warning.

## Acceptance tests added

The package adds end-to-end tests that were missing before:

- ToB and WOB monotonic with engagement;
- ROP monotonic with WOB and zero without load/contact;
- harder material raises ToB and reduces ROP;
- ToB setpoint step creates immediate physics feedforward movement;
- impossible ToB is explicitly reported as limited;
- pump head feedforward is monotonic;
- LADRC 3.0-ish -> 4.5 ft·lbf class step has a clear response within 0.5 s on the deterministic plant;
- PID and LADRC share the same nominal authority path.

Local validation of the packaged Python modules: **15 tests passed** in the reconstructed current repo environment before packaging.

## Important calibration note

The following are engineering priors, not claimed identified values:

- `contact_stiffness_n_m = 5.5e6`
- `torque_force_coeff_m = 1.05e-3` (`mu_eff * r_eff` interpretation)
- `nominal_physical_rop_mm_min = 0.08`
- feedforward seal force `650 N`
- LADRC local pressure gain `b0 = 75 bar/s per normalized pump-speed-squared deviation`

They make the simulator internally causal and controllable. They should later be identified from pump dead-head/flow tests, pusher force tests, and controlled Inconel cuts rather than tuned from desired demo appearance.
