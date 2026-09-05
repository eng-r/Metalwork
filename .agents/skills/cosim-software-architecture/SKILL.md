---
name: cosim-software-architecture
description: Design the Python OOP simulator, deterministic virtual-time backplane, plant/controller adapter interfaces, multi-rate scheduling, headless execution, optional threaded real-time emulation, logging, FastAPI/WebSocket backend, and React/TypeScript UI boundary. Use for software decomposition and timing. Do not let UI timing drive numerical integration.
---

# Co-Simulation Software Architecture

## Prime directive

The simulation kernel must be deterministic and runnable without the UI.

## Suggested component boundaries

- `PlantModel`
- `Controller`
- `Estimator`
- `SimulationClock`
- `Scheduler`
- `Backplane`
- `SensorAdapter`
- `ActuatorAdapter`
- `Scenario`
- `ExperimentRunner`
- `Recorder`
- `Metrics`
- `ApiServer`
- web UI

Define interfaces so plant and controller implementations can be swapped independently.

## Virtual-time backplane

The backplane should contain named operational signals with:

- timestamp / simulation tick;
- value;
- units;
- validity/status;
- optional sample age.

Explicitly define sample rates for:

- plant integration;
- controller;
- drive loops or reduced actuator models;
- sensor updates;
- telemetry/UI updates;
- logging.

Use zero-order hold or an explicitly documented interpolation rule between rates.

## Threading recommendation

### Offline/headless simulation

Prefer one deterministic scheduler driving plant and controller in virtual time.

Threads are not required for numerical correctness.

### Real-time-emulation mode

Separate plant/controller worker threads may be used if synchronized by:

- common monotonic tick;
- barrier/event;
- bounded queues;
- explicit deadline handling.

Never use race timing as a model feature.

### UI/logging

Safe candidates for separate threads/tasks:

- WebSocket publication;
- disk logging;
- plot-data decimation;
- command reception.

## Python backend

Use OOP and dependency injection/configuration so multiple models can implement the same contracts.

Candidate libraries:

- NumPy;
- SciPy;
- python-control;
- Pydantic for configuration/contracts;
- pandas/Polars and Parquet for results as needed;
- CasADi only for optimization/MPC paths;
- SysIdentPy/PySINDy only for identification paths.

Avoid making optional research packages mandatory for the basic simulator.

## API/UI boundary

Use REST for:

- configurations;
- experiment definitions;
- start/stop/reset;
- result metadata.

Use WebSockets for live telemetry/plot streams.

The UI should never directly mutate plant state. It sends commands/configurations through the backend contract.

## Web UI

React + TypeScript/TSX is appropriate.

Focus UI on engineering UX:

- scenario/config panel;
- controller selection;
- start/pause/reset;
- time traces;
- phase/status indicators;
- target/contact visualization;
- parameter/metric panels;
- export controls.

Do not spend effort on 3D visualization before the model is validated.

## Headless mode

Required use cases:

- command-line batch runs;
- DoE sweeps;
- Monte Carlo;
- controller comparison;
- regression testing;
- calibration optimization.

The exact same simulation kernel must serve headless and UI modes.

## Reproducibility

Every run should record:

- software/config version;
- plant/controller class and parameters;
- random seed;
- solver settings;
- scenario;
- initial state;
- result metrics.

See `references/REFERENCE.md`.
