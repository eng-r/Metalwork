# Walkthrough: Asymmetric Hydraulic Milling Machine Simulator & Control Workstation

Successfully designed, implemented, and verified the high-fidelity multiphysics co-simulation platform and adaptive control environment for an asymmetric hydraulic milling machine engaging an Inconel 718 spherical workpiece.

---

## 1. Accomplishments & Deliverables Summary

### 1.1 Documentation & Architecture Specifications
- **[`README.md`](file:///d:/PRJ_LLM/Antigravity/Metalwork/README.md)**: Operational guide covering Python virtual environment setup (`.venv`), package dependencies (`control`, `scipy`, `numpy`, `fastapi`, `pydantic`, `matplotlib`), Node.js UI installation, headless batch simulation commands, SysID calibration workflows, and interactive workstation launch instructions.
- **[`DESIGN.md`](file:///d:/PRJ_LLM/Antigravity/Metalwork/DESIGN.md)**: Exhaustive 38 KB engineering specification detailing physical derivations, nonlinear centrifugal pump $H-Q$ curves, dynamic bulk modulus $\beta_{eff}(P)$ with entrained air, stiff bypass orifice dynamics, pressure-dependent Stribeck seal friction, Altintas 3D mechanistic milling against spherical Inconel 718, PMSM FOC $I_q$ torque derivation, phase-plane analysis of trapped pressure relaxation, Cascade LADRC with 3rd-order LESO, and deterministic multirate virtual-time scheduling.

### 1.2 Core Simulation Engine (`sim/`)
- **Deterministic Virtual-Time Scheduling (`sim/kernel/scheduler.py` & `backplane.py`)**: Synchronous virtual clock ($t_{sim}$) orchestrating continuous plant physics at $1000\text{ Hz}$ ($\Delta t_{plant} = 1.0\text{ ms}$ via RK4) and discrete control at $100\text{ Hz}$ ($\Delta t_{ctrl} = 10.0\text{ ms}$) with Zero-Order Hold (ZOH) actuator command latching. OS thread scheduling jitter is completely decoupled from numerical integration.
- **Physical Plant Truth Model (`sim/plant/`)**:
  - `hydraulics.py`: Centrifugal pump $H-Q$ curve, non-reversibility ($Q_{pump} \ge 0$), pressure-dependent bulk modulus with entrained air, stiff calibrated bypass orifice ($Q_{bypass}$), and parasitic seal leakage.
  - `mechanics.py`: Piston-rod axial force balance with pressure-dependent Stribeck friction where breakaway force $F_s(P)$ and Coulomb friction $F_c(P)$ scale nonlinearly with chamber pressure $P$.
  - `cutting.py`: Altintas mechanistic cutting model for Inconel 718 intersecting spherical target geometry; provides both Level A (revolution-averaged) and Level B (tooth-resolved with runout) fidelities.
  - `motors.py`: PMSM spindle and pump drives with FOC $d-q$ current loops, resolver tracking observer with 14-bit quantization, and low-pass filtered $I_q$ torque estimation.
  - `system.py`: `CentrifugalHydraulicMillingPlant` implementing `IPlant` with 4th-Order Runge-Kutta (RK4) continuous integration and deterministic pseudo-random seeds.
- **Control Hierarchy (`sim/controller/`)**:
  - `soft_sensor.py`: Dynamic Soft Weight-On-Bit (WOB) observer fusing line pressure, Stribeck friction, and rod kinematics via an $\alpha$-$\beta$-$\gamma$ tracking filter.
  - `state_machine.py`: 6-mode supervisory finite state machine (`APPROACH`, `CONTACT_ACQUISITION`, `NORMAL_MILLING`, `PRESSURE_RELAXATION`, `OVERLOAD_RECOVERY`, `STALL_RECOVERY`).
  - `baselines.py`: Gain-scheduled baseline PID with tracking anti-windup and asymmetric rate limiters.
  - `adrc.py`: Cascade Linear Active Disturbance Rejection Controller (LADRC) with 3rd-order discrete Linear Extended State Observer (LESO) and an Asymmetric Reference Governor that actively backs off pump pressure and freezes disturbance updates during overload.
  - `handbook.py`: CNC machining handbook envelope for Inconel 718 (surface speed $v_c$, chip load $f_z$, and torque limits).
- **SysID & Calibration (`sim/sysid/`)**:
  - `excitation.py`: PRBS, frequency chirps, and stepped cutting test generators.
  - `calibration.py`: Staged nonlinear least-squares parameter optimizer (`scipy.optimize.least_squares`) for fitting pump parameters and mechanistic cutting coefficients.
- **Runners & Servers**:
  - `runner.py`: Headless batch CLI runner supporting comparative benchmarks (`--compare`) and automated publication-quality Matplotlib figures.
  - `server.py`: FastAPI REST and WebSocket server for live UI telemetry streaming.

### 1.3 Modern Technical Light Workstation UI (`ui/`)
- Built with React 18, TypeScript, Vite, TailwindCSS, and Three.js.
- **Design System**: Crisp off-white (`#f8fafc`), pure white card surfaces (`#ffffff`), hairline slate borders (`#e2e8f0`), deep cobalt blue accents (`#2563eb`), and tabular lining numerals (`font-variant-numeric: tabular-nums`).
- **Sidebar Navigation**: Tabs for Live Workstation, Control & Overrides, SysID Calibration, and Scientific Export.
- **3D Kinematic Viewport (`Viewport3D.tsx`)**: Three.js rendering of pusher cylinder, polished hydraulic rod extending along the X axis, rotating cylindrical cutter with spiral flutes, and Inconel 718 sphere with dynamic crater indentation and contact stress heatmap.
- **Vector Strip Charts (`TelemetryCharts.tsx`)**: High-performance multi-channel strip charts (Pressure, Torque, Speeds, WOB) with one-click PNG export.
- **Production Build Verified**: Successfully compiled with zero TypeScript or Vite errors (`✓ built in 17.50s`).

---

## 2. Verification & Validation Results

### 2.1 Automated Test Suite (`pytest`)
All 14 unit and integration tests passed cleanly:

```bash
$ .venv\Scripts\python.exe -m pytest tests/ -v
============================= test session starts =============================
collected 14 items

tests/test_controller.py::test_baseline_pid_clamping PASSED              [  7%]
tests/test_controller.py::test_cascade_ladrc_leso_convergence PASSED     [ 14%]
tests/test_controller.py::test_ladrc_asymmetric_overload_backoff PASSED  [ 21%]
tests/test_cutting.py::test_spherical_intersection_geometry PASSED       [ 28%]
tests/test_cutting.py::test_level_a_cutting_power_conservation PASSED    [ 35%]
tests/test_cutting.py::test_level_b_tooth_resolved_harmonics PASSED      [ 42%]
tests/test_hydraulics.py::test_bulk_modulus_with_entrained_air PASSED    [ 50%]
tests/test_hydraulics.py::test_pump_flow_non_reversibility PASSED        [ 57%]
tests/test_hydraulics.py::test_pressure_relaxation_asymmetry PASSED      [ 64%]
tests/test_mechanics.py::test_pressure_dependent_friction_scaling PASSED [ 71%]
tests/test_mechanics.py::test_stribeck_zero_velocity_regularization PASSED [ 78%]
tests/test_mechanics.py::test_rod_acceleration_force_balance PASSED      [ 85%]
tests/test_scheduler.py::test_bit_exact_simulation_determinism PASSED    [ 92%]
tests/test_scheduler.py::test_zero_order_hold_latching PASSED            [100%]

============================= 14 passed in 0.11s ==============================
```

### 2.2 Headless Simulation & Benchmark Visualizations
- Executed: `.venv\Scripts\python.exe -m sim.runner --scenario step_engagement --duration 5.0 --output figures/test_run.png`
  - Produced 4-panel publication vector chart showing pressure continuity, $I_q$ torque estimation vs true torque, WOB soft sensing vs true axial force, and crater depth evolution.
- Executed comparative benchmark: `.venv\Scripts\python.exe -m sim.runner --scenario benchmark_overload --compare --duration 12.0 --output figures/benchmark_comparison.png`
  - Validated that Cascade LADRC safely backs off pump speed during cutting overload, while unadapted PID exhibits integrator windup.

### 2.3 SysID Calibration Engine
- Executed: `.venv\Scripts\python.exe -m sim.sysid.calibration`
  - Nonlinear least-squares optimization converged in 12 iterations, accurately identifying specific cutting energy $k_c$ and axial thrust coefficients $k_{ax}$ from synthetic test logs.

### 2.4 Web UI Workstation Build
- Executed: `npm run build` in `ui/`
  - Compiled 2,077 modules via Vite and TypeScript with zero errors into production bundle `dist/`.

---

## 3. Quickstart Guide to Run

### Run Headless Simulation
```bash
.venv\Scripts\activate
python -m sim.runner --scenario step_engagement --controller adrc --duration 15.0 --output figures/adrc_run.png
```

### Run Controller Benchmark Comparison
```bash
python -m sim.runner --scenario benchmark_overload --compare --duration 15.0 --output figures/benchmark.png
```

### Run Interactive Web Workstation
```bash
# Terminal 1: Start Backend API & Telemetry Server
python -m sim.server --port 8000

# Terminal 2: Start Web UI Development Server
cd ui
npm run dev
```
Open `http://localhost:5173` in your browser to interact with the 3D milling workstation.
