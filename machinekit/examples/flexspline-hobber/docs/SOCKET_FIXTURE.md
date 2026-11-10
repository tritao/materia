# Two-socket casting fixture

This is a separate manufacturing coupon, not the complete hobber base mould.
It uses the existing panel plans, standard fasteners, portable component recipes
and continuous straight-translation removal checks.

The sample casting is 400 × 240 × 70 mm. Two M6 × 40 blind sockets are 160 mm
apart, their mouths flush against a common mould face. Two 500 × 340 × 18 mm
plates form the bed and removable locating plate. A round primary hole and
slotted secondary hole locate the upper plate on two nominal 6 mm pins; two
M6 closure bolts clamp the plates together. Each socket is pulled against the
upper plate by an M6 × 30 screw and ISO 7089 washer fitted from below. Larger
access holes through the bed allow the screw/washer stack and driver to pass.

The mouth faces down during casting. The socket body and closed crimped tail
point upward into the mix. A qualified mouth-face/bolt-path seal is required;
its material remains unselected. No fictitious sealing-film thickness is added
to the datum stack, and no resin-side plug is invented for the closed tail.

With the nominal 18 mm plate and 1.6 mm washer, screw insertion is 10.4 mm.
This spans the reference 6 mm thread dimension while remaining inside the
25 mm insertion cavity. The planner rejects screws that are too short or
would bottom out. Supplier OD/crimp geometry is still nominal as described in
[the enclosure notes](ENCLOSURE.md#cast-in-socket-model).

`materia.tooling.casting.SocketFixturePlanner` supplies an explicit blind-socket
contract, panel operations, sealing records and qualification states. It does
not reinterpret sockets as the old `CastingToolingInsert` through-bored boxes.
`machinekit.tooling.SocketFixtureCad` adapts that data to CAD using `PanelKit`,
`ToolingPanel`, `ToolingPin` and `ToolingHardwareCad`. `SocketCoupon` subtracts
the actual full socket exterior from the casting, including the space occupied
by the protected blind cavity; no rectangular pocket surrounds the tail.
The full-base `FoundationToolingAdapter` is unchanged and does not yet consume
this new contract.

Build with `haxeon.socket-fixture.json`, then run the resulting HashLink module
with the usual MachineKit native libraries. Example build from the worktree root:

```sh
./haxeon/scripts/haxeon build --compiler-only \
  --project=machinekit/examples/flexspline-hobber/haxeon.socket-fixture.json \
  --output=/tmp/hobber-socket-fixture.hl
```

Outputs in `build/socket-fixture/`:

- `socket-fixture-assembled.mtrg` for Materia, with views and an Explode slider.
- `fixture-plan.json`, portable `fixture.assembly.json`, and `review.json`.
- `demolding-steps.json`, with screw release before plate withdrawal.
- DXF, SVG and panel-operation JSON for both locating/bed plates.
- `bom.json`, separating two permanent sample sockets and nominal mix mass
  from the reusable plates, pins, screws, washers, nuts and walls. Seal compound
  and wall closure hardware remain listed as unselected requirements.

Geometry acceptance covers all initial near-pair interferences, socket mouth
positions, protected cavity volume, driver approach, released positioning
hardware sweeps, plate withdrawal against the cured casting and sockets, wall
withdrawal against casting, and component/scene round trips. A negative control
proves the screw heads block plate removal when left fitted. Screw unscrewing
is semantic; these are translation checks after thread release.

The demoulding procedure is to cure, remove positioning screws/washers, release
plate and wall closure hardware, withdraw walls, separate the supporting bed
and locating pins, then withdraw the locating plate. Support the casting while
changing tooling; actual restraint and handling need qualification.

This is a **geometry review**, not a ready-to-pour mould. OD/tail dimensions,
stock flatness, locator fit and accumulated hole-position error need physical
confirmation. A 0.15 mm nominal diametral pin clearance is not a claim of
0.15 mm final socket accuracy. Wall clamping/pressure, seam and mouth sealing,
release, compaction, cure, retention and handling remain pending in the normal
tooling readiness report. In particular the wall components show their mould
faces, not qualified closure hardware. Review this coupon before extending
the method to the ten tray sockets or flipping the full hobber base.
