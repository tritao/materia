# Detailed hob geometry

`HobDesign` defines the generating rack and the cutting tool used by `HobbingSweep`.
`HobSolid` now makes a detailed CAD solid from those same rake-face polygons.
There is no independent decorative thread or substituted involute profile.

The part has two CAD representations and an optional display mesh:

- `Envelope`, or a recipe with `detailed=false`: the bored cylindrical blank for
  fast conservative machine-clearance, runtime and error-source studies.
- `Preview` with `detailed=true`: helical tooth lands, straight gashes with the
  requested rake, backing relief, through bore, optional keyway and flat end faces.
  It is a single fused B-rep, usable for STEP export and CAD-derived mass/inertia.

`renderTeeth=true` displays a tessellation of that same fused B-rep. It has no
independent tooth, core or end-cap construction. Vertices at sharp face boundaries
remain separate for their normals, but their positions form shared mesh boundaries.
With `detailed=false`, physical properties still use the conservative CAD envelope;
select `detailed=true` when detailed mass/inertia or CAD export is needed.

Open `hob.project.json` for the isolated fine reference display, or
`detailed.project.json` for the coarse machine with displayed teeth. Use
`hob-cad.project.json` for the actual detailed B-rep and its physical properties.
The normal `materia.project.json` remains the fast cutting demonstration. Detailed
views now generate the fused solid rather than an overlapping display approximation;
initial generation takes longer. Generated scene artifacts are cached by Materia.
The isolated keyed example uses the same `HobDesign` as the reference machine.
The installed machine uses a plain bore; a matching physical key is not added.

## Construction

Each land starts at the exact gash rake plane. Its outer comb profile contains all
teeth along the tool, with consecutive starts separated by `axialPitch`. Following
sections retreat around the tool and follow the same helical lead. Radial relief
increases linearly with retreat at the tip and tapers to zero at the tooth root.
This preserves the front cutting profile. Each section remains planar, including
nonzero rake: its circle/plane intersection is recomputed at the relieved radius.

A smooth loft connects those sections. The canonical land is repeated by a screw
transform: gash `g` rotates by `2πg/gashes` and translates by `lead*g/gashes`.
Intersecting each land with the finite blank clips end teeth and prevents material
outside the conservative cylinder. A cylindrical core joins the lands. Gashes
continue below the thread roots; the bore and optional keyway are then removed.

## Explicit assumptions and accuracy

These are designed backing surfaces, not a recovered supplier grinding specification.
The supplied design assumptions are:

| Input | Default |
|---|---|
| Land fraction of gash pitch | 0.6 |
| Tip relief angle | 3 degrees |
| Additional flute depth below tooth root | 0.5 mm |
| CAD cutting-profile chord tolerance | 0.0005 mm (0.5 µm) |
| Backing-surface angular sampling target | 0.01 mm |
| Bore | 16 mm |
| Keyway | Disabled by default; isolated example assumes width 5 mm, radial depth 2.3 mm |

The 0.5 µm bound applies to simplification of the recovered polygon, not to the
manufacturing accuracy of a hob. The smooth backing loft is interpolated through
sampled sections; its sampling target is not a certified global surface-error bound.
The cutting simulation keeps the original rake polygons and its own tolerances.
Adding the body does not change force coefficients, sharpening allowance, metrology
limits or the HB11 error budget. Grinding kinematics, resharpening, end chamfers,
coating and vendor-specific flute/relief geometry remain unspecified.

Recipes save these inputs, along with generating target, starts, gashes and rake.
The old recipe ID loads with `detailed=false`, preserving the previous envelope
and its performance. The designation now reflects actual OD, gashes and starts.

## Checks

Run `./check-hob.sh quick` from this directory (or use its full repository path).
This exercises the complete parts/persistence suite, closed-mesh topology checks,
and a short keyed, two-start, nonzero-rake CAD tool. Checks include one connected solid, finite mounting ends,
actual bore/keyway clearance and backing relief, and distances from simulated
cutting-edge points to actual CAD edges. The native-edge comparison includes a
small numerical guard beyond the configured profile tolerance. Mesh checks weld
coincident face-seam positions at 1 nm resolution for topology only, then require
two oppositely directed incidences per edge, no duplicate/collapsed triangles, one
connected shell, genus one for the bore, and positive signed volume. Face normals
remain separate at sharp edges in the rendered mesh.

`./check-hob.sh reference` builds the full 40 mm, 12-gash fine tool separately.
Set `HOB_DETAIL_EVIDENCE` to an existing output directory to export STEP and a
result report. Set `HOB_DETAIL_EXPORT_MESH=1` for the additional inspection scene.
`HOB_DETAIL_INPUT_STEP` permits verification
and scene generation from a previously exported solid without rebuilding it.
Reports are saved before optional mesh/export work so completed geometry checks
are not lost if that later work is interrupted. These are tool-geometry checks;
they do not claim a new cut, a supplier accuracy class or structural acceptance.

To check a saved isolated hob scene without rebuilding CAD, run the parts-test
bytecode with `hob-mesh scene /path/to/hob.mtrg`. `hob-mesh reference` checks a
short 12-gash reference solid; `hob-mesh` covers zero and both signs of rake on
short, keyed, two-start tools. These checks retain 0.01 mm mesh deflection and
0.0005 mm cutting-profile simplification.
