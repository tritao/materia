# HZMotion mount suitability for the VMC variants

Updated installation, 2026-10-20. This is an interface/architecture
screen, not a completed collision, fastener, bearing-load or drive-sizing release.

## Installed selections

| Configuration / axis | Installed motor | Mount |
| --- | --- | --- |
| Small X | Leadshine ELM2H-0400LA60F, 400 W / 220 V | Catalog-pattern fabricated plate; folded 1:1 belt retained |
| Small Y | Leadshine ELM2H-0400LA60F, 400 W / 220 V | HZMotion M60D-BK12-C supplier STEP, BF12 support |
| Both Z axes | Leadshine ELM2H-0750LA80E, 750 W / 220 V, internal holding brake | HZMotion M80E-BK15-C integrated housing, flat T adapter, SFU2005 screw and BF15 support; motor internal brake |
| Both spindle drives | Leadshine ELM2L-1500LA100FT-H MS1AE, 1.5 kW / 380 V | Fabricated adjustable carrier, 60T motor / 20T spindle HTD5M pulleys |
| Compact X/Y | No installed motors or screw drives | Guided packaging study; drive sizing/integration remains pending |

The spindle ratio is 3:1 speed increase: 8,000 spindle rpm requires about 2,667
motor rpm. This replaces an assumed 2.2 kW spindle envelope with a real **1.5 kW**
motor, not a claim of equivalent power. Belt capacity, matched drive selection,
thermal duty and the motor's continuous operating envelope still need qualification.

The X motor face is now at X = -216 mm, outward of its old -195 mm position,
with connectors facing forward and a rectangular pocket in the saddle. This
preserves carriage-bolt key access. The belt plane is X = -234 mm; the screw
engages 11 mm of the 15 mm pulley bore, exceeding the explicit packaging check
of 0.75 times its 12 mm journal diameter. Retention/key and torque capacity are
separate qualifications. The widened removable hood remains inside the base.

Both Z drives now use the purchased integrated motor/fixed-bearing housing.
One flat 20 mm T adapter replaces the old crossplate, stand and separate coupling
guard. Its lower stem fits between the rails; the crossbar keeps the existing
column-strip anchors. Motor face is 40 mm above the column; the bearing centre
sits 93 mm below that face. Travel and guide positions are unchanged.

The 20 mm / 5 mm lead screw has the explicit HZMotion BK15 end requirement:
20 mm coupling journal, 44 mm shoulder-to-coupling length, 15 mm bearing journal,
and M15×1 lock thread over 12 mm (root envelope only). The SFU2005 nut has its
published 51 mm barrel and offset flange, with a matching shelf bore and M6 pattern.
This is a machining requirement to order, not a claim that a standard Vallder end
fits unchanged. A generic 19/12 mm, 48 mm coupling provides 18/20 mm shaft
engagement; torque, keys/clamping and fatigue remain unqualified.

The former assumed external 4 N·m brake envelope interferes with the supplier
housing and is removed. The real motor's internal holding brake is retained.
The sizing report publishes required holding torque and explicitly marks holding
capacity unqualified: no invented 4 N·m rating or application delay is assigned
to the Leadshine brake. VMC scenes are kinematic previews; this does not qualify a
power-loss stop or create a brake/control safety contract.

The spindle motor connectors face sideways to clear the stationary Z drive. Its
carrier and guard support whole-tooth belt fitting and the full adjustment range.
The separate legacy Bench Mill and its electrical/control model are unchanged.

## Supplier mount limits

## Why one bracket cannot serve every axis

The installed Small Y component is a coaxial integrated motor/bearing bracket.
The Small X plate returns the motor into the saddle instead of projecting it
outside the base. Its motor and screw axes are separated vertically and connected
by a belt. Replacing this plate with a coaxial bracket changes that architecture.

A first-order Small X direct-drive screen, retaining the fixed bearing centre
at X = −192.5 mm and the Y installation's 41.5 mm bearing-centre-to-motor-face
offset, puts the motor face at X = −234 mm. An ELM2H-0400LA60F body extends another
88.8 mm to X = −322.8 mm, **62.8 mm beyond the 520 mm base's left edge**.
The current generic 130 mm motor body would reach X = −364 mm. These estimates
exclude connector/cable access, guards and a new collision sweep. They explain
why the return belt exists; changing to direct drive is possible only with a
revised machine envelope and supports.

The generic flange patterns are explicitly assumed and differ from the real
motor interfaces. The generic 400 W uses a 50 mm square; the 60 mm Leadshine/HZ
interface uses a 70 mm bolt circle, equivalent to a 49.497 mm square. The generic
750 W uses a 70 mm square; the real 80 mm motor and HZ interface use a 90 mm
bolt circle, equivalent to a 63.640 mm square. A matching nominal frame width
does not establish hole alignment.

For Z, the M80E-BK15-C fits the selected 80 mm Leadshine pilot and bolt circle.
Its 15 mm fixed journal and BF15 floating support motivated the SFU2005 upgrade.
The pinned STEP has a width of 88 mm (drawing 86) and rear mounting holes at
X=±27 mm (drawing ±29). The STEP component and T adapter use measured holes;
existing drawing recipes retain the drawing revision. Motor thread depth remains
unpublished, so M6×16 motor bolts are a packaging proposal, not a joint release.

The smaller KK86-FK10L with 80EP adapter retains a 12 mm bearing bore and has a
nominal 70 mm motor pilot / 90 mm bolt circle. However, the adapter's pilot recess
depth is unpublished, its M5 motor fasteners need qualification in the motor's
6.5 mm holes, and adapter-to-body fastening is not established. The selected braked ELM2H-0750LA80E still needs verified holding data and matched drive wiring.

## Remaining work

Qualify the selected motors with matched drives, winding/voltage and production
loads. Complete Compact X/Y screw/motor sizing and integration independently.
Supplier components remain shared on suitable direct-drive axes; the folded X
plate, flat Z adapters, spindle carrier, seats and guards remain machine-specific.

The original interface screen below explains the rejected direct substitutions;
its generic motor dimensions describe the previous installations.

## Evidence and validation

- `tools/suppliers/hzmotion-reference.json`: reviewed dimensions, 135-entry family
  coverage matrix, bearing arrangements and unresolved interfaces.
- `tools/suppliers/hzmotion-cad.json`: enabled STEP references are M60D-BK12-C,
  M80E-BK15-C, BF12 and BF15; other downloads are not automatically validated catalog geometry.
- `MillXYStageLayout`, `MillXYStage`, `MillZAxisLayout`, `MillZAxis`,
  `MillCapacityLayout` and `LeadshineServo`: installed architecture and datums.
- `python3 -m unittest discover -s tools/suppliers/tests -p 'test_hzmotion*.py'`:
  ten catalog/provenance regression tests pass. Mechanical installation checks
  additionally cover pilots, fasteners, carriage key access, pulley engagement,
  full-stroke envelopes, spindle withdrawal, belt adjustment and scene persistence.
  These do not qualify real cutting loads or electrical safety.

## Current load screen

The Small XYZ and Compact Z motors pass the existing preliminary duty and
sustained-cut torque screens. The 20 mm screw improves the critical-speed screen;
see the regenerated `build/axis-sizing/axis-sizing.json` for current mass and speed
results. Bearing capacity, production duty and structural stiffness remain subject
to engineering release. The current kinematic mission remains 20 mm/s.

The spindle selection does not meet the historical 2.2 kW tool-output target.
Increasing its power will require a separately qualified motor/head installation.

The Z audit checks physical motor/housing contact, complete travel against the
castings and mounts, screw handedness, guide reserve and recipe persistence.
The Compact study additionally checks full spindle payload/removal clearance.
Native CadKit modeling and naming smoke tests cover the rigid-placement change;
the supplier CAD audit checks two successive placements of the actual M80 STEP
without topology or volume changes. Use rigid locations rather than rebuilding
trimmed geometry when placing purchased parts.

The BF15 lower plate uses 15 mm blind M6 taps and M6×45 screws (about 13 mm
nominal engagement). No nuts protrude toward the moving saddle. The full XYZ
sweep checks this interface; thread strength and joint loads remain unqualified.
