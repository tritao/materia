# SR11 sheet CAM and retained-part ordering

Run `machinekit/examples/sheet-router/sheet-operations/check.sh` from the repository
root. It reuses the SR9 compact core and existing CAD/FFI providers; no OCCT rebuild
is needed. The current focused core passes **46 assertions**, and the existing
panel CAM regression passes **293 assertions**. The both-bed cabinet run and its
holding margins are recorded in `validation.json`.

`CamJob.sheetProfile` adds the following to the existing profile API, using metres
and metres/second throughout:

- Exact compensated line/arc paths, with climb or conventional direction per
  operation. For a clockwise spindle, outside climb is clockwise and inside climb
  counterclockwise; conventional reverses both.
- Manual tab centres, or deterministic automatic tab placement, away from entry
  and corners. Straight-span tabs lift over requested length plus cutter diameter,
  so the rounded cutting caps leave the requested minimum web length. Circular
  profiles additionally account for angular cutter reach and the smallest radius
  across the curved kerf. Requested count, length and retained thickness are
  explicit; impossible counts or overlapping/manual corner tabs fail.
- Onion skin of a declared thickness, with an optional final skin pass. Tabs remain
  during the skin pass. Native checks compare every Z ray between rough/final jobs:
  each changed ray loses exactly 0.5 mm, while the four 3 mm webs remain. Native
  samples also check the retained length around each tab centre.
- Tangent quarter-circle lead-in/out on the waste side and a profile ramp of a
  declared length. Curved ramps use authored XY helices. A full finishing lap
  follows a ramp, so the starting section is finished at depth too. Leads are
  geometric entries; their sweep against neighbouring parts/fixtures still needs
  the full stock/clearance acceptance check.

The tab length is a minimum retained bridge width, not a web-strength rating.
Manual centres are distances along the compensated, oriented loop after its entry
has been moved to the middle of its longest straight span (or a circular half-span).
Use `CamSheetProfile.plan` to inspect this path and the resulting tab intervals.
Tab centres need enough space within one straight span, or a continuous circular
span; this prevents automatic webs across polygon corners.

`CamJob.withSpindleSpeed` sets a modal operation RPM. A speed change on the same
loaded tool emits a spindle command without another tool change; new tools use
the selected operation speed. `drill` already expands pecks with feed plunges and
full retracts; the new regression checks three pecks over a 6 mm depth.
`surface` emits a bidirectional raster with bounded step-over and depth levels.
Native surfacing is compared with an independent box target: zero gouge/leftover,
and removed volume equals initial stock minus target volume.

`CamSheetProfiles.fromDxf` maps SR10 part boundaries from mm to SI at an explicit
surface Z. It preserves exact line/arc boundaries alongside the declared chordal
preview and splits full circles into exact semicircles. Circular compensation is
analytic, preserving arcs and allowing curved webs. Other exact boundaries use
the existing native compensation with the contour's stated chord tolerance.

`CamSheetOrder.order` enforces prerequisites, inside-before-outside on each part,
all non-release operations before that part's release, and small-before-large
where operations are otherwise ready. Ready internal operations precede outside
profiles. The optional admission callback performs a tentative native stock cut
and the SR8 holding check. It must roll the state back on refusal and retain it on
acceptance. If every remaining ready operation is unsafe, planning fails instead
of returning a plan that releases a slipping part. The library does not duplicate
pump, friction or cutting-force assumptions.

## Cabinet evidence and limits

| Bed | Accepted operations | Minimum retained holding margin | Refused free-part margin |
|---|---:|---:|---:|
| 2600 mm | 8 | 24.224 | 0.0151 |
| 3150 mm | 10 | 25.703 | 0.0133 |

Both beds pass all ten cabinet assertions.

The router fixture uses each complete 1600×2600/3150×18 mm sheet at **1 mm Z-ray
spacing** for component/pressure scanning, with six/eight 700 mm-wide panels plus
a 50 mm part and its blind hole. It applies actual generated CAM and SR8's default
300 N per part, safety factor 2. A through-cut small part is refused and restored.
The retained sequence completes the small hole, the small tabbed profile, then
larger tabbed profiles. Four 8 mm webs per profile retain a nominal **3 mm above
the support plane** (bottom −1 mm plus the requested 4 mm tab thickness).

Every accepted checkpoint remains one connected stock component and passes the
SR8 lateral holding screen. This does **not** qualify the strength or fatigue of
wood/plastic tabs: SR8 deliberately flags retained webs for force/material
qualification. The 300 N allowance remains provisional until SR13. The −1 mm
fixture overcut proves separation in native stock; it is not an approved physical
Z reach, spoilboard skim or tool-gauge allowance. The existing SR8/SR13 tool-reach
gate stays in force. Final tab removal/breakout is a separate supported/manual
operation; the planner does not claim a freed small part will hold.

The core fixture uses all three dexel grids at 0.5 mm. The full-sheet cabinet run
uses Z rays for connectivity/holding rather than claiming another full tri-dexel
structural or renderer qualification. SR9 covers both tri-dexel sheet backends.
SR13/SR16 remain held for MotionKit OM-R1 and cutting/commissioning acceptance.

Direction convention follows the existing CamKit convention and [Autodesk's
climb/conventional reference](https://help.autodesk.com/cloudhelp/ENU/Fusion-CAM/files/GUID3DCDDEC0-95C8-4444-8DC4-74E1B20F9413.htm).

Retained packet: `build/sheet-router-evidence/sr11-sheet-operations.tar.gz`,
SHA-256 `36df6f76160ac5e97e710ee3029d69b606596f6ac8b85b5fee8fa8f31a8029e4`; regular-file member hashes verified.
