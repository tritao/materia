# Linear stage

A single-axis lead-screw stage: a NEMA 23 stepper, a 10 mm trapezoidal screw (2 mm lead), a coupling, two flange bearings and a carriage on
an MGN12C profile rail, with 200 mm of travel. It is `machinekit.assembly.LinearAxis` unchanged, so the joint (`carriage-slide`), its limits
and the parts are the library's. The "with pendant" variant adds the jog pendant, whose first position jogs the carriage.
