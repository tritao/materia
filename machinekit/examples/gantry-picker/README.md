# Gantry picker

Open `materia.project.json` from Materia’s Start page and press Play. A 1500 × 1000 × 500 mm belt gantry carries six cartons from its infeed to a two-row pallet pattern with a sensed suction tool.

The two Y motors follow one translational joint. Geometry, transmissions and drive limits come from the physical assembly. The tool reuses the catalog suction assembly used by the arm. Tables are 200 mm high to leave room under the Z column. Layout, material choice and racking tolerance are stated assumptions.

Play first homes the switched axes sequentially, retracting Z before horizontal motion. Only Z moves initially; on the current example the whole homing sequence takes about 80 simulation seconds before carton handling begins. The transport status names the active homing axis, and the status bar explains its phase.

The focused MuJoCo regression checks all six cartons, placement within 2 mm and 2 degrees, unintended contacts, and drive-checked plans. Run it with `PROJECT_SOURCE_ONLY=gantry ./haxeon/scripts/haxeon run --project app/haxeon.project-source.json`. Use `PROJECT_SOURCE_ONLY=gantry-realtime` for the paced Play-path check (it runs at wall-clock pace).

The carriage plate places the flange 60 mm ahead of the Z guide so a lifted carton clears the column. The Z switch brackets sit alongside the column, outside that lift path. These dimensions are authored layout assumptions.
