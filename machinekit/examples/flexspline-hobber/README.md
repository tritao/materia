# Flexspline hobber

A vertical GE-class moving-column hobber: fixed direct-drive work table, radial
column infeed, vertical head feed, direct swivel and twin-rail hob shift.

See [MACHINE_DESIGN.md](MACHINE_DESIGN.md) for datums, travel, support and
acceptance scope, and [PLAN.md](PLAN.md) for engineering evidence and open gates.
The project entry point is `materia.project.json`; run it through Materia's
project-source loader. `check-quick.sh app` runs the coarse app/cycle smoke check.
Full reference metrology and error-budget cuts are separate, expensive checks.

Open `inspection.scene.json` for the mechanical inspection view with the electrical
rack hidden. Its drives and power wiring remain in the underlying assembly.

The head has a front-mounted compact swivel and vertical twin-rail Y stage.
[LAYOUT_SECTION.svg](LAYOUT_SECTION.svg) shows the dimensioned radial stack.
