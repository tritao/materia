# Gantry picker

Open `materia.project.json` from Materia’s Start page and press Play. A 1500 × 1000 × 500 mm belt gantry carries six cartons from its infeed to a two-row pallet pattern with a sensed suction tool.

The two Y motors follow one translational joint. Geometry, transmissions and drive limits come from the physical assembly. The tool reuses the catalog suction assembly used by the arm. Tables are 200 mm high to leave room under the Z column. Layout, material choice and racking tolerance are stated assumptions.

Implementation is uncompiled and runtime validation is deferred at the user’s request. Cycle time, final placement error, allocation budget and clearance results have not been measured.

The carriage plate places the flange 60 mm ahead of the Z guide so a lifted carton clears the column. The Z switch brackets sit alongside the column, outside that lift path. These dimensions are authored layout assumptions.
