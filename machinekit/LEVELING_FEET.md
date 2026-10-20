# Levelling-foot hardware

`machinekit.structural.LevelingFoot` is the shared component for the mill,
flexspline hobber, sheet router and T-slot tending tables. Its saved `part`
uses `LevelingFootSpec` schema 1. Previous machine-specific foot and seat
recipes are rejected; there is no old-format reader.

The closed variants are:

- `Foot`: a complete nominal purchased foot, including pad, stem and an explicit
  round collar or `NoCollar`. It is one BOM item.
- `Pad` and `Screw`: separate components when supplied separately.
- `Plate`: a removable rectangular bearing plate with a clearance hole.
- `Nut`: a separate round or hex nut with an explicit metric thread.

Screws use an explicit nominal-diameter or thread-root CAD envelope. Their
metric diameter and pitch are saved, and CAD carries the matching semantic
thread; nuts carry its internal mate. These nominal shapes do not qualify
thread engagement, bearing strength, fatigue or supplier conformity.

Every part starts at local Z=0. On a complete foot, `floor` is Z=0,
`mount` and `frame` are the declared mounting height, and `top` is the
physical stem top. On separate hardware, `mount` is Z=0 and `frame`/`top`
are the upper face. Assembly placements determine installation height.

The mill's prior foot is preserved by placing its floor 50 mm below the
casting datum; its mounting height is 90 mm. The hobber keeps its removable
110 × 110 × 15 mm plate, two separate M24 × 3 nuts and complete foot.
The roof/register dimensions belong to `FoundationLayout`, rather than the
hardware library. The router retains its 100 mm pad, 20 mm stem and frame
datum. T-slot tables retain their thread-root stems and existing locknuts.

Native migration checks compare the old and new solids by symmetric
difference. Assembly codecs preserve the same shared recipe and distinct
physical BOM items. The hobber's gravity and roof-contact checks remain
active independently of the unsupported non-box beam model.

All members are explicitly assumed nominal hardware. A saved geometry is
not a vendor load rating. The hobber's 15 kN per-foot value remains an
assumed screening target in its load setup; supplier ratings, roof bending,
creep, fatigue, dynamic stability and physical qualification remain pending.
