# Milling Simulator Agent Skill Pack

This pack is intended to guide an AI coding/research agent through the **planning, modeling, identification, control-design, software-architecture, validation, and UI design** of a hybrid physics/data-driven simulator for a hydraulically pushed milling tool.

It is deliberately split into focused skills rather than one giant `SKILL.md`. The Agent Skills specification recommends progressive disclosure: keep the triggering/workflow instructions in `SKILL.md`, and load detailed technical material from `references/` only when needed.

## Recommended activation order

1. `milling-sim-orchestrator`
2. `multiphysics-plant-modeling`
3. `hydraulic-pusher-modeling`
4. `machining-cutting-mechanics`
5. `pmsm-drive-and-soft-sensing`
6. `sysid-doe-calibration`
7. `hybrid-greybox-modeling`
8. `adaptive-control-design`
9. `cosim-software-architecture`
10. `validation-and-experimentation`

The orchestrator should invoke only the relevant skills for a given task. Do not load all reference files into context at once.

## Key architectural position captured by this pack

The core simulator should use a **deterministic virtual-time scheduler/backplane**, not independently free-running plant and controller threads. Plant and controller may be wrapped in separate threads for real-time emulation, but numerical simulation must preserve explicit sample times, zero-order holds, deterministic ordering, and reproducibility. UI, logging, and telemetry streaming may run asynchronously.

## Expected project outputs later

When the user asks to proceed from planning to implementation, the agent should generate at least:

- project `README.md` with environment setup, Python virtual environment, package installation, UI setup, launch modes, and headless mode;
- detailed `design.md` covering architecture, interfaces, physics, states, parameters, sensors, controller hierarchy, calibration workflow, validation strategy, and data flow;
- configuration schemas for plant, controller, experiment, and simulation scenarios;
- a headless simulator first;
- a web UI only after the headless simulation kernel is validated.

## Reference format

Each skill contains:

- `SKILL.md` — task workflow and rules;
- `references/REFERENCE.md` — technical references and modeling notes.

## Agent Skills standard

https://agentskills.io/specification

OpenAI skill-creator reference:
https://github.com/openai/skills/blob/main/skills/.system/skill-creator/SKILL.md

Anthropic skill-creator reference:
https://github.com/anthropics/skills/blob/main/skills/skill-creator/SKILL.md
