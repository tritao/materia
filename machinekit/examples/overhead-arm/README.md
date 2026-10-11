# Overhead gantry arm

An inverted Reach900 industrial arm hangs from a physical mounting adapter on
the Z carriage of a 3 m × 1.5 m XYZ gantry. X and Y use rack drives; Z uses a
lead screw. The table extends 100 mm beyond the X and Y travel on every side.

Open `materia.project.json` in Materia. The six-step mission picks a workpiece
near one corner and places it at the opposite corner. The stations lower Z by
150 mm, leaving travel in both directions. The arm's inverted ready pose turns
the suction tool toward the tabletop, within the same physical joint limits as
the floor-mounted example.

`OverheadArmTests` checks tabletop reach analytically at fifteen stations and
runs positioning and a coordinated diagonal FollowPath with all three external
axes on MuJoCo, matching G17's physical handling acceptance. The deterministic
backend does not support the compound-body contacts needed for this table.
The coordinated check uses a stated 2 mm planning clearance; fitted gantry
hardware has a nominal 3 mm gap, leaving room for the continuous motion bound.
Dimensions and mounting offsets are
authored layout assumptions; transmission ratios and drive limits come from
the installed components.
