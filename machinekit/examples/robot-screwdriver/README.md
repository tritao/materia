# Robot screwdriver equipment

Open `materia.project.json` from the start page's **Robot screwdriver equipment**
entry. The cell carries a spindle on a sensed compliant feed, a replaceable hex
socket bit with a vacuum sleeve, a tightening controller and a hand-loaded screw
presenter. Four screws seat the flange bearing housing on its tapped plate.

The preview checks the CAD fastenings, bit fit, torque capacity, on-axis reach
and approach clearance. Equipment dimensions and spindle capacities are explicit
custom design assumptions; tightening guidance comes from the published SD0
catalog. The scene declares real tool joints and four control channels.

This is the SD4 equipment preview. Tightening behavior and the process sensor are
SD5; an automatic screw-driving mission is SD6. The model and controller slots
carry no fabricated tightening program or angle limits.

The separate `materia.threads.project.json` and `materia.fastenings.project.json`
retain the thread rendering and CAD workpiece previews. See [PLAN.md](PLAN.md).
