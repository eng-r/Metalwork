# Orchestrator References

## Agent Skills structure

Agent Skills specification:
https://agentskills.io/specification

OpenAI skill creator:
https://github.com/openai/skills/blob/main/skills/.system/skill-creator/SKILL.md

Anthropic public skills:
https://github.com/anthropics/skills

## Why grey-box / control-oriented modeling is appropriate

MathWorks System Identification overview:
https://www.mathworks.com/help/ident/gs/about-system-identification.html

Grey-box model estimation:
https://www.mathworks.com/help/ident/grey-box-model-estimation.html

Control-oriented system identification review (Annual Reviews in Control, 2026):
https://doi.org/10.1016/j.arcontrol.2026.101067

## Architectural principle

The project is a coupled hybrid dynamical system with continuous states, discrete contact/machining modes, actuator limits, sample/hold effects, and uncertain parameters. The simulator architecture should make all of these explicit rather than bury them in a monolithic "step()" function.
