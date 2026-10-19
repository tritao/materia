# Downloaded Leadshine torque-speed graphs

Retrieved 2026-10-19 from manufacturer product pages. Original images are kept
unchanged; source URLs, SHA-256 hashes and revision issues are recorded in
[manifest.json](manifest.json). These are raster curves, not a numerical table.
All three graphs are now digitized in [digitized-curves.json](../../digitized-curves.json).
The original PDF entries retain their ratings and assumed actuator interpolation.
An additional, explicitly named `ELM2L-1500LA100FT-H MS1AE` model matches the
1.5 kW page variant and exposes its graph through a conditional study API.

| Local catalog designation | Downloaded graph | Source and reconciliation issue |
| --- | --- | --- |
| ELM2L-1500LA100FT-H | [MS1AE graph](ELM2L-1500LA100FT-H-MS1AE.png) | [Manufacturer page](https://www.leadshine.com/product-detail/ELM2L-1500LA100FT-H%20MS1AE.html) identifies MS1AE. Voltage/current differ from our older PDF row. |
| ELM2M-2900LD180FT-H | [Graph](ELM2M-2900LD180FT-H.png) | [Manufacturer page](https://www.leadshine.com/product-detail/ELM2M-2900LD180FT-H.html) has conflicting graph/table peak torque, plus different inertia/mass from our older PDF. |
| ELM2M-7500LD180FT-H | [Graph](ELM2M-7500LD180FT-H.png) | [Manufacturer page](https://www.leadshine.com/product-detail/ELM2M-7500LD180FT-H.html) agrees on 48/119 Nm torque labels but differs in voltage, rotor inertia and mass. |

All three graphs distinguish **continuous duty** from **intermittent duty** and
show falling continuous torque above the base-speed knee. That is information
our rated/peak numbers alone do not provide. The graph images do not establish
the matching drive, DC bus, ambient temperature, heat-sink/cooling arrangement
or permitted intermittent duration. Their absence must remain explicit.

For the 2.9 kW row, the graph's intermittent plateau is labeled 46.5 Nm, the
same product page's table says peak torque 30 Nm, and the older PDF used by
our model says 55.8 Nm. These cannot be silently combined into one rating.
For the 1.5 kW model, MS1AE and MS10A are distinct page variants; a curve for
one must not be assigned to the other solely because their base designations match.

## Digitization and model assignment

[digitize_curves.py](../../digitize_curves.py) records manually inspected pixel
coordinates and tick anchors, checks original-image hashes, and generates the
JSON plus typed native Haxe data. The 1.5 kW curve has a 4.9 Nm continuous plateau
to 3000 rpm and approximately 2.19 Nm at 5000 rpm. The reading allowance is
0.15 Nm vertically and 60 rpm horizontally. The native/Python study interpolators
subtract the vertical allowance plus horizontal allowance times the maximum
captured segment slope. At 5000 rpm the resulting study ordinate is about
1.96 Nm. This allowance covers reading the drawing, not unknown operating conditions.

The larger graphs have nonuniform tick spacing and are treated as schematic.
Their data uses piecewise tick calibration; stated reading allowances are 1 Nm /
30 rpm for 2.9 kW and 2.5 Nm / 30 rpm for 7.5 kW. Those allowances cannot bound
unidentified revision or axis-drawing errors. Their numeric data remains blocked
from automatic model assignment, including the apparently plausible continuous
segments. Neither graph was used to overwrite the older catalog values.

The MS1AE page's [mechanical drawing](ELM2L-1500LA100FT-H-MS1AE-mechanical.png)
agrees with the inherited nominal main dimensions. The new model explicitly
uses the page's 400 VAC, 4.5 Arms rated and 13.5 Arms peak ratings; the old 380 VAC
PDF entry is preserved. Geometry remains a simplified nominal envelope.

`LeadshineServo.torqueCurveEvidence()` exposes the review records. Only the exact
MS1AE designation gets `studyContinuousTorque(rpm)`; out-of-range speeds return
null rather than extrapolated torque. `LeadshineTorqueCurves.qualifiedContinuousTorque`
still returns null because operating conditions are unverified. Intermittent
curves remain evidence: no permitted duration is known and no runtime overload
capability is derived from them. Current actuator curves remain explicitly assumed.

To qualify these curves, obtain the matching drive/bus/cooling and ambient
conditions, reconcile the conflicting large-motor sources, and establish the
permitted intermittent duration. That information cannot be recovered from the
graph pixels. The present import is numerically usable conditional evidence,
not a new manufacturer guarantee.
