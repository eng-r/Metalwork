# Hybrid / Grey-Box References

MathWorks grey-box model estimation:
https://www.mathworks.com/help/ident/grey-box-model-estimation.html

Nonlinear grey-box models:
https://www.mathworks.com/help/ident/nonlinear-grey-box-models.html

Physics-informed online learning of grey-box models using moving-horizon estimation:
https://doi.org/10.1016/j.ejcon.2023.100861

Control-oriented system identification review:
https://doi.org/10.1016/j.arcontrol.2026.101067

PySINDy:
https://pysindy.readthedocs.io/en/stable/

SysIdentPy:
https://sysidentpy.org/

CasADi for nonlinear ODE/DAE optimization and optimal control:
https://web.casadi.org/docs/

## Recommended role of learned models

Treat data-driven components as corrections to known physics and as explicit uncertainty models. This improves interpretability and makes it easier to prevent a learned model from violating contact, force, or actuator constraints.
