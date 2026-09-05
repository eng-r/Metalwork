# Adaptive / Robust Control References

Model Reference Adaptive Control overview:
https://www.mathworks.com/help/slcontrol/ug/model-reference-adaptive-control.html

Adaptive control design overview:
https://www.mathworks.com/help/slcontrol/adaptive-control-design.html

L1 Adaptive Control tutorials — Naira Hovakimyan:
https://naira-hovakimyan.mechse.illinois.edu/l1-adaptive-control-tutorials/

J. Han / ADRC perspective — From PID to Active Disturbance Rejection Control:
https://doi.org/10.1109/TIE.2008.2011621

Gernot Herbst — practical LADRC introduction:
https://arxiv.org/abs/1908.04596

Disturbance-observer-based control overview:
https://doi.org/10.1109/TIE.2015.2478397

Constrained MPC force control of an electrohydraulic actuator:
https://doi.org/10.1016/j.conengprac.2010.09.002

Robust MPC + PI force control with nonlinear hydraulic plant and grey-box identification:
https://doi.org/10.1016/j.conengprac.2018.07.009

Model Predictive Control for Force Control in Milling:
https://doi.org/10.1016/j.ifacol.2017.08.2336

Adaptive force control in end milling — controller comparison:
https://doi.org/10.1016/0278-6125(91)90043-2

Monitoring and control of cutting forces in machining — review:
https://www.fujipress.jp/ijate/au/ijate000300040445

## Working recommendation for this project

Do not commit to one advanced controller at the beginning. A strong study sequence is:

1. gain-scheduled PI/PID + state machine;
2. LADRC/ADRC comparator;
3. constrained MPC/NMPC comparator using the calibrated grey-box model;
4. MRAC only if the reference-model structure is justified.

The pusher's one-sided pressure-release dynamics are a particularly strong reason to evaluate constrained predictive control.
