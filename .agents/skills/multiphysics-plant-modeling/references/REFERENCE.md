# Multiphysics Plant Modeling References

SciPy ODE integration:
https://docs.scipy.org/doc/scipy/reference/integrate.html

`solve_ivp`:
https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html

MathWorks nonlinear grey-box modeling concepts:
https://www.mathworks.com/help/ident/nonlinear-grey-box-models.html

Control-oriented system identification review:
https://doi.org/10.1016/j.arcontrol.2026.101067

Hydraulic actuator pressure dynamics and bulk modulus example:
https://pmc.ncbi.nlm.nih.gov/articles/PMC7919780/

Hydraulic cylinder model with pressure states and dynamic friction:
https://doi.org/10.1016/j.compstruc.2014.02.006

## Modeling note

The contact/cutting process introduces discrete mode changes while hydraulics, drivetrain and structural states are continuous. Treat the plant as a hybrid dynamical system. The model should remain differentiable within a mode when possible, while state transitions handle unilateral contact and other discontinuities.
