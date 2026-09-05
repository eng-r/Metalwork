# Validation References

MathWorks system-identification workflow and model comparison:
https://www.mathworks.com/help/ident/gs/system-identification-workflow.html

MathWorks System Identification overview:
https://www.mathworks.com/help/ident/gs/about-system-identification.html

NIST/SEMATECH Engineering Statistics Handbook:
https://www.nist.gov/programs-projects/nistsematech-engineering-statistics-handbook

SciPy integration / solver options:
https://docs.scipy.org/doc/scipy/reference/integrate.html

Adaptive force-control comparison in milling using a validated process model:
https://doi.org/10.1016/0278-6125(91)90043-2

## Core rule

Never validate a controller only against the exact model and nominal parameters used for tuning. Use a richer plant, held-out experimental data, and uncertainty cases.
