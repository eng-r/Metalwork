# Co-Simulation / Software References

Python thread synchronization and Barrier:
https://docs.python.org/3/library/threading.html

SciPy ODE integration:
https://docs.scipy.org/doc/scipy/reference/integrate.html

Python Control Systems Library:
https://python-control.readthedocs.io/

CasADi:
https://web.casadi.org/docs/

FastAPI WebSockets:
https://fastapi.tiangolo.com/advanced/websockets/

React with TypeScript:
https://react.dev/learn/typescript

Vite:
https://vite.dev/guide/

Pydantic:
https://docs.pydantic.dev/latest/

SysIdentPy:
https://sysidentpy.org/

PySINDy:
https://pysindy.readthedocs.io/en/stable/

## Timing note

Python threads can be useful for emulating distributed execution and keeping UI/logging responsive, but an offline simulator should not depend on OS scheduling to decide when control or plant updates occur. Use virtual time and explicit scheduling for reproducible control experiments.
