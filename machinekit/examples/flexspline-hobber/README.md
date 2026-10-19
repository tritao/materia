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

The inspection assembly now selects full supplier HGH15CA/HGH25CA blocks and a
matching HGR25R X rail profile. Before generating it on a fresh checkout, run
`python3 machinekit/examples/linear-rail-reference/prepare.py`. Asset revisions
are pinned and saved in component recipes; no runtime downloads occur. The X
screw is centred between the bed rails in a recessed channel, with its motor at
the rear. Channel covers remain a design provision.

### Watch a coarse cut

Open `cutting.scene.json` in Materia. It selects the cutting project, hides the
separate electrical rack, and displays the hob's relieved teeth. Press **F5** to
play/pause (or use the toolbar), **Shift+F5** to reset, and **Home** to fit the machine. Frame or zoom
into the hob and blank to see the teeth emerge from the stock. This is the
module-1, 40-tooth demonstration; it is not a reference-part grading run.

The electrical rack, power supply and six drive modules are hidden by default in
all complete-machine entrypoints (`hobber`, `detailed`, `cutting`), including direct
generated-artifact opens. Visibility is authored per occurrence and survives
saving/reopening; the components, BOM and electrical connections remain present.
They can be shown again from the scene hierarchy. Saved inspection/cutting view
overrides retain the same preference.
