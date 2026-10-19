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
all complete-machine entrypoints (`hobber`, `detailed`, `cutting`, `loading`), including direct
generated-artifact opens. Visibility is authored per occurrence and survives
saving/reopening; the components, BOM and electrical connections remain present.
They can be shown again from the scene hierarchy. Saved inspection/cutting view
overrides retain the same preference.

The hob spindle now includes a hollow shaft, nose, opposed bearings near the
tool, bearing retention and a drawbar-retained taper-and-face arbor with drive
pins. It is an assumed custom interface, not a qualified HSK interface; see
[MACHINE_DESIGN.md](MACHINE_DESIGN.md#hob-spindle-cartridge-2026-10-19).


### Cup loading demonstration

Open [loading.scene.json](loading.scene.json) and press **Play**. The head starts
at X=110 mm, with Y=Z=0. A one-piece cup (open end/toothed rim down,
closed end up) approaches from the front, transfers 250 mm with 35 mm clearance,
then lowers over the collapsed sleeve onto the axial stop, clamps, runs the actual
coarse tooth-band cut, retracts the head, releases and lifts out. **Reset → Play**
replays it. The original quick [cutting.scene.json](cutting.scene.json) remains
available without the handling sequence.

The Machine panel provides **Blank**, **Clamped**, **Finished** and **Clamped
section** inspection views. Inspection is available before cutting or after the
sequence completes. **Continue sequence** returns from a paused inspection view.
Sectioning is view-only; close it with **Close section**. Inspection does not
reset or replace the actual cut stock.

The earlier blank was only an annular tooth band. It is now a single CAD cup
with a 40 mm overall length, 2 mm closed end and 0.6 mm wall below the tooth
roots (assumed dimensions). Material removal and metrology still operate on the
toothed band; the unmachined cup wall/end remain visible in the runtime mesh.
This demo uses module 1, 40 teeth and a 4 mm band, not the precision reference.

Loading is a guided kinematic illustration, without a robot or loose-body
contact dynamics. The separate sleeve closes the nominal 25 µm radial fit gap
at true display scale; its expansion is explicitly illustrative, not a solved
hydraulic strain or pressure prediction. Cutting starts after seating/clamping;
unloading starts only after the spindle stops and X/Y/Z return to the loading pose.

The HB-D11 foundation is a 930 × 800 × 800 mm monolithic plinth on three rigid
levelling feet, with the tooth band 1000 mm above the floor. The default bought DD table has a Ø200 mm face, 210 mm height and Ø40 mm bore;
unpublished accuracy/stiffness and mounting details remain assumed. The built-up
table remains selectable through the same interface. See PURCHASED_TABLE_REPORT.json.
See [MACHINE_DESIGN.md](MACHINE_DESIGN.md#heavy-plinth-acceptance-hb-d11) for mass,
support and preliminary stiffness evidence. The app's machining time excludes
the illustrative loading/unloading sequence.

### Post-plinth reference budget

The current HB11 run is in `build/flexspline-reference-budget-plinth`.
`review.md` / `review.json` summarize completed inspections; `progress.json`
lists missing cases. The background units are `flexspline-hb11-plinth-budget`
and `flexspline-hb11-plinth-review` (`journalctl --user -u UNIT -f`).

To resume after the unit has stopped, use the same frozen evidence and provider:

```sh
python3 machinekit/examples/flexspline-hobber/run-plinth-budget.py \
  build/flexspline-reference-budget-plinth \
  --previous build/flexspline-reference-budget-parallel \
  --native build/flexspline-shared-sweep/native-pair
```

The runner refuses changed binary/input/settings fingerprints. Do not replace
its frozen binary during a run. HB4 convergence and HB12 reference acceptance
remain open; a completed numerical budget does not establish ISO certification.
