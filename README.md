# Asymmetric Hydraulic Milling Machine Simulator & Adaptive Control Workstation

A high-fidelity, multiphysics co-simulation platform and adaptive control design environment for a hydraulically pushed milling machine engaging an Inconel 718 spherical workpiece.

The platform couples a variable-speed centrifugal pump, compressible hydraulic line, pressure-dependent seal friction, and PMSM drives with a mechanistic 3D milling model. A deterministic virtual-time scheduler coordinates multi-rate plant and controller execution with Zero-Order Holds (ZOH), ensuring bit-exact reproducibility for System Identification (SysID) and control benchmarking. The system includes both headless batch execution engines and a modern technical light web workstation UI.

---

## Key Highlights

- **Deterministic Virtual-Time Kernel**: Numerical integration ($\Delta t_{plant} = 1.0\text{ ms}$) and discrete control ($\Delta t_{ctrl} = 10.0\text{ ms}$) run on a synchronized virtual clock with explicit sample holds. Operating-system thread scheduling jitter is completely decoupled from physics simulation.
- **Physical Hydraulic Operating Point**: Replaces crude RPM-to-pressure transfer functions with a speed-dependent centrifugal head-flow ($H-Q$) curve, dynamic bulk modulus $\beta_{eff}(P)$ with entrained air, piston displacement flow $A_p \dot{x}$, a stiff calibrated bypass orifice ($Q_{bypass}$), and parasitic seal leakage.
- **Decoupled Pressure & Contact Force (WOB)**: Pressure $\times$ Area is not treated as instantaneous Weight-On-Bit. Rod inertia and nonlinear, pressure-dependent Stribeck seal friction are explicitly resolved, supported by a dynamic Soft WOB Observer.
- **Mechanistic Cutting of Inconel 718**: Altintas mechanistic cutting model with differential tangential, radial, and axial cutting forces ($dF_t, dF_r, dF_a$) based on 3D cylindrical-cutter / spherical-workpiece intersection geometry and instantaneous chip thickness $h(\theta)$. Features Level A (revolution-averaged) and Level B (tooth-resolved with runout) fidelities.
- **Advanced Control Hierarchy**:
  - *Baseline*: Gain-Scheduled PID with asymmetric rate limiters and supervisory state machine.
  - *Primary*: Cascade Linear Active Disturbance Rejection Control (LADRC) with a 3rd-order Linear Extended State Observer (LESO) and Asymmetric Reference Governor.
  - *Comparators*: Evaluated against Constrained Model Predictive Control (MPC) and Model Reference Adaptive Control (MRAC).
- **Modern Technical Light UI**: Crisp academic/laboratory workstation aesthetic (`#f8fafc`, `#ffffff`, border `#e2e8f0`, cobalt blue `#2563eb`) featuring a left sidebar, real-time hairline metric cards, interactive Three.js 3D milling kinematics with dynamic crater rendering, and publication-ready vector chart exports.

---

## Directory Layout

```
Metalwork/
├── README.md                      # Operational and setup manual (this document)
├── DESIGN.md                      # Exhaustive engineering & mathematical specification
├── requirements.txt               # Python package dependencies
├── pyproject.toml                 # Modern Python packaging configuration
├── sim/                           # Python simulation kernel & controllers
│   ├── common/                    # Abstract base classes and data contracts
│   │   ├── interfaces.py          # IPlant, IController, ISoftSensor
│   │   └── contracts.py           # ProcessVariables, ControlCommands, TruthDiagnostics
│   ├── kernel/                    # Deterministic execution engine
│   │   ├── scheduler.py           # Virtual-time multirate scheduler
│   │   └── backplane.py           # Thread-safe double-buffered state backplane
│   ├── plant/                     # Physical truth models (Level A & Level B)
│   │   ├── hydraulics.py          # Centrifugal pump H-Q, bulk modulus, bypass orifice
│   │   ├── mechanics.py           # Rod inertia, pressure-dependent Stribeck friction
│   │   ├── cutting.py             # Inconel 718 mechanistic milling & sphere intersection
│   │   ├── motors.py              # PMSM FOC dq-currents & resolver tracking
│   │   └── system.py              # Integrated plant with RK4 numerical integrator
│   ├── controller/                # Control architectures
│   │   ├── state_machine.py       # Supervisory operating mode transitions
│   │   ├── soft_sensor.py         # Dynamic Soft WOB / force estimator
│   │   ├── baselines.py           # Gain-scheduled PID with asymmetric anti-windup
│   │   ├── adrc.py                # Cascade LADRC with 3rd-order LESO & Reference Governor
│   │   └── handbook.py            # Machining parameters & optimal surface speed envelope
│   ├── sysid/                     # Calibration and Design of Experiments (DoE)
│   │   ├── excitation.py          # PRBS, chirp, and stepped cutting test generators
│   │   └── calibration.py         # Staged grey-box nonlinear least-squares optimizer
│   ├── runner.py                  # Headless batch CLI runner & matplotlib generator
│   └── server.py                  # FastAPI REST & WebSocket streaming server
├── tests/                         # Automated test suite (pytest)
│   ├── test_hydraulics.py         # Pressure continuity, non-reversibility, bypass test
│   ├── test_mechanics.py          # Pressure-dependent friction & stick-slip dynamics
│   ├── test_cutting.py            # Mechanistic milling forces & geometric intersection
│   ├── test_scheduler.py          # Virtual-time determinism & bit-exact reproducibility
│   └── test_controller.py         # ADRC vs PID overload and pressure relaxation benchmark
└── ui/                            # Web workstation frontend
    ├── package.json               # Node.js dependencies
    ├── vite.config.ts             # Vite configuration
    ├── tsconfig.json              # TypeScript configuration
    ├── tailwind.config.js         # TailwindCSS styling tokens
    ├── index.html                 # HTML shell
    └── src/                       # React / TypeScript source code
        ├── App.tsx                # Workstation container & layout
        ├── components/
        │   ├── Sidebar.tsx        # Navigation & scenario selector
        │   ├── MetricCard.tsx     # Hairline laboratory KPI indicators
        │   ├── Viewport3D.tsx     # Three.js 3D mechanical milling rig
        │   ├── TelemetryCharts.tsx# High-resolution vector strip charts & SVG export
        │   └── ControlPanel.tsx   # Manual overrides & parameter tuning drawer
        └── services/
            └── socket.ts          # WebSocket client for real-time telemetry streaming
```

---

## Prerequisites

Ensure your system meets the following prerequisites:

1. **Python**: Version `3.11` or higher (`3.11.x` - `3.12.x` recommended).
2. **Node.js**: Version `18.x` or higher (LTS recommended) and `npm` (`v9.x` or higher).
3. **C/C++ Build Tools** (optional, recommended for accelerated native ODE solvers):
   - Windows: Microsoft Visual C++ Build Tools or MinGW.
   - Linux: `build-essential`.
   - macOS: Xcode Command Line Tools.

---

## Setup & Installation Instructions

### 1. Python Environment Setup

Open a terminal (PowerShell or Bash) in the repository root directory (`Metalwork/`):

```bash
# 1. Create a dedicated virtual environment named .venv
python -m venv .venv

# 2. Activate the virtual environment
# On Windows (PowerShell):
.venv\Scripts\Activate.ps1
# On Windows (Command Prompt):
.venv\Scripts\activate.bat
# On Linux / macOS:
source .venv/bin/activate

# 3. Upgrade core packaging tools
python -m pip install --upgrade pip setuptools wheel
```

### 2. Install Python Dependencies

Install the core simulation, control, sysid, and API packages:

```bash
pip install -r requirements.txt
```

#### Key Libraries Included:
- **`python-control`**: Foundational control systems library (transfer functions, state-space representations, ZOH discretization, Bode/Nyquist stability margins).
- **`adrc`** / **`pyadrc`**: Standard Active Disturbance Rejection Control utilities for benchmarking.
- **`scipy` & `numpy`**: Vectorized numerical operations and ODE integration (Runge-Kutta 4, Levenberg-Marquardt optimization).
- **`fastapi` & `uvicorn[standard]`**: Asynchronous web server exposing REST endpoints and high-speed WebSockets for UI telemetry streaming.
- **`pydantic`**: Strict data validation and schema definitions for plant and controller configuration.
- **`matplotlib`**: Publication-quality scientific vector chart rendering for headless experiments.
- **`pandas` & `pyarrow`**: Fast telemetry recording to CSV and Apache Parquet formats.
- **`pytest`**: Unit testing and physical assertion verification framework.

### 3. Web UI Workstation Setup

Navigate to the `ui/` directory and install the Node.js packages:

```bash
cd ui
npm install
cd ..
```

#### Frontend Stack:
- **React 18** with **TypeScript** for strict type-safe UI components.
- **Vite** for sub-second hot-module replacement and optimized production builds.
- **Three.js** / **@react-three/fiber** for hardware-accelerated 3D kinematic visualization of the pusher, spinning bit, and spherical workpiece.
- **TailwindCSS** configured with the Modern Technical Light aesthetic palette.
- **Lucide React** for clean, minimalist engineering icons.

---

## Running the Simulator

The platform operates in two primary modes: **Headless Execution** (ideal for batch sweeps, SysID, Monte Carlo runs, and CI/CD) and **Interactive Workstation Mode** (with full 3D rendering and real-time strip charts).

### Mode 1: Headless Simulation (Offline / Batch Execution)

Run a complete simulation without launching the web server or UI. The deterministic virtual-time kernel integrates the equations, executes the controller, logs telemetry, and exports publication-ready vector figures:

```bash
# Run standard step-engagement scenario using Cascade LADRC
python -m sim.runner --scenario step_engagement --controller adrc --duration 15.0 --output figures/adrc_milling.png

# Run comparative benchmark: Baseline PID vs Cascade LADRC
python -m sim.runner --scenario benchmark_overload --compare --duration 20.0 --output figures/benchmark_comparison.png

# Export raw simulation telemetry to Apache Parquet and CSV
python -m sim.runner --scenario step_engagement --export-parquet data/telemetry.parquet --export-csv data/telemetry.csv
```

#### Supported Scenarios:
- `step_engagement`: Pusher advances rapidly in free space, contacts the Inconel 718 sphere, and transitions into regulated steady-state milling.
- `benchmark_overload`: Sudden 50% step increase in target hardness / contact curvature designed to trigger bit overload; demonstrates pressure relaxation dynamics and anti-windup performance.
- `stall_recovery`: Artificially drops spindle speed to trigger the safety state machine, showing automated pump back-off and re-engagement.
- `relaxation_test`: Commands pump RPM to minimum idle while in deep cut to measure the relaxation time constant through material clearance versus the stiff bypass orifice.

---

### Mode 2: System Identification (SysID) & Calibration Pipeline

Calibrate the hybrid plant model against laboratory data using the staged DoE pipeline:

```bash
# Generate synthetic excitation test logs (PRBS pump commands and stepped cutting)
python -m sim.sysid.excitation --generate-suite --out-dir lab_data/

# Run staged nonlinear least-squares parameter estimation
python -m sim.sysid.calibration --data-dir lab_data/ --validate-heldout
```

The SysID tool outputs calibrated parameters with 95% confidence intervals:
- Pump head coefficients $(a_0, a_1, a_2)$
- Effective fluid bulk modulus $\beta_{eff}$ and bypass orifice conductance $C_{bypass}$
- Pressure-dependent Stribeck friction parameters $(F_{c0}, F_{s0}, \alpha_{cp}, \alpha_{sp}, v_s)$
- Mechanistic cutting force coefficients for Inconel 718 $(K_{tc}, K_{te}, K_{rc}, K_{re}, K_{ac}, K_{ae})$

---

### Mode 3: Interactive Workstation Mode (Backend + Web UI)

Launch the simulation backend and web UI for real-time monitoring and parameter tuning.

#### Step 1: Start the Backend Simulation Server
In your activated Python environment:

```bash
python -m sim.server --port 8000 --host 127.0.0.1
```
The FastAPI backend boots up, instantiates the process backplane, and listens on `http://127.0.0.1:8000` with WebSocket telemetry at `ws://127.0.0.1:8000/ws/telemetry`.

#### Step 2: Start the Web UI Dev Server
In a second terminal:

```bash
cd ui
npm run dev
```
Open your browser at `http://localhost:5173`. You will see the **Modern Technical Light Milling Workstation**.

---

## Modern Technical Light UI Guide

The web workstation is built to meet the visual standards of scientific research laboratories and academic publications:

- **Sidebar Navigation**: Switch seamlessly between:
  - **Live Workstation**: Real-time 3D kinematic view and live operational strip charts.
  - **Controller Comparison**: Side-by-side performance overlays (ADRC vs PID vs MRAC).
  - **SysID & Calibration**: Residual plots, parameter covariance ellipses, and excitation signal configurators.
  - **Export Center**: High-resolution chart exports (SVG, PNG) and raw dataset downloads.
- **Top Command Bar**: Displays deterministic simulation time ($t_{sim}$), system operating mode (e.g., `NORMAL_MILLING`, `PRESSURE_RELAXATION`), Play/Pause, Step Tick, and emergency Stop (E-Stop).
- **Hairline Metric Cards**: Live indicators for:
  - Hydraulic Line Pressure $P_{hyd}$ (bar)
  - Estimated Weight-On-Bit $\hat{F}_{wob}$ (N)
  - Bit Spindle Torque $T_{bit}$ derived from FOC $I_q$ (N·m)
  - Spindle Speed $\omega_{bit}$ & Pump Speed $\omega_{pump}$ (RPM)
  - Axial Penetration Depth $d$ (mm) & Material Removal Rate MRR ($\text{mm}^3/\text{s}$)
  - LESO Estimated Total Disturbance $f(t)$
- **Interactive 3D Viewport**:
  - Precision mechanical model showing the pusher cylinder, extending hydraulic rod, rotating cylindrical cutter, and Inconel 718 sphere.
  - Dynamically cuts a spherical crater into the target as material is removed.
  - Color-coded thermal/stress contact ring indicating instantaneous cutter load.
- **Vector Strip Charts**:
  - Sub-millisecond vector rendering with zoom, pan, cursor inspection, and one-click SVG export for LaTeX/reports.

---

## Automated Verification & Tests

Run the full suite of physical conservation checks, numerical convergence tests, and controller assertions:

```bash
pytest tests/ -v
```

### Key Test Coverage:
1. `test_hydraulics.py`: Asserts non-reversibility ($dP/dt \le 0$ when $\omega_p=0$ and $\dot{x}=0$), bulk modulus compression curves, and stiff orifice flow.
2. `test_mechanics.py`: Asserts pressure-dependent breakaway force scaling $F_s(P)$ and dynamic stick-slip damping.
3. `test_cutting.py`: Validates cylindrical-spherical intersection geometry, chip thickness calculation, and mechanistic force balance.
4. `test_scheduler.py`: Asserts bit-exact identical trajectories across multiple runs under varying thread timing.
5. `test_controller.py`: Verifies that Cascade LADRC safely relaxes pressure during cutter overload while unadapted PID suffers integrator windup and tool stall.

---

## Technical Documentation

For complete mathematical derivations, ODE formulations, control-theoretic proofs, state-space equations, and software design contracts, see **[`DESIGN.md`](DESIGN.md)**.
