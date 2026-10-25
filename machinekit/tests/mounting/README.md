# Shared linear-mount source checks

Run from the repository root through the heavy-run gate, reusing prebuilt OCCT:

```sh
CADKIT_OCCT_DIR=/home/joao/dev/materia-cache/occt-f1dc4efb-release \
  /home/joao/dev/materia-cache/tools/heavy-run --mem 4G --need 4G --margin 2G \
  --label linear-mount -- ./haxeon/scripts/haxeon run \
  --project=machinekit/tests/mounting/haxeon.json
```

`MountedElement` adapts the rail or catalogue rack's own connectors, counterbores,
thread and recipe identity. An authored scale recipe can supply that contract;
today's generic tape scale has no drilled mounting data and is rejected. No holes
are invented for it.

`MountHoleDerivation` contains the formerly cast-in-only derivation. Both the
existing `InsertMountPattern` and a future welded-host receiver use it. Receiver
local +Z is the finished steel face normal; the component pose can put that face
anywhere in the machine. The rack's bolt-face plane is independent of its back
plane. Holes retain actual connector identity, source fingerprint, full clearance
budget and permitted transfer-drilling proposal. Null allocations stay pending.
The persisted legacy drilling-route tokens still use casting terminology; they
are not evidence that a welded receiver is a cast insert.

`MountServiceScrews` is the existing rail service-screw selection extracted into
one shared function. `MillRailFasteners` delegates to it. It checks the actual
counterbore, matching thread, grip, engagement, tip gap, back wall and shank.
It supports the existing **blind C45 receiver** route; it does not qualify a
through-bolt/captive-nut route or weaker welded steel by applying C45 defaults.

`LinearMountDatum` is a preflight contract for separate retained-insert and
relieved-welded-pad hosts, replicated plane, steel receiver, adjustment bar,
push screws and locking fasteners. It computes a feasible nominal layer interval
from explicit compound limits and fixture/cure bounds. It counts real added
members once when a receiver also serves as the side bar. It checks identities
and roles, not physical contact, measured alignment, anchorage capacity or cure.
Those still need the physical mount factory and its native checks. The test
compound ranges, plastic layer stand-in and fixture dimensions are **not**
production geometry or supplier limits.

Validation on 2026-10-22: `LinearMountTests` **55 assertions**; existing
`RailSeatTests` **116 assertions / 36 service screws**; `GearRackChecks` passes
all four catalogue recipes, transformed geometry, ratings and reload. Logs and
source hashes are retained by the router's shared-mount work record. No frozen
router geometry or FE mode has changed. RM2 remains in progress, not complete.

The separate `haxeon.contact.json` target runs a review-only native side-mount
fixture. It uses the same hole and fastener derivation, a 20×28 mm C45 receiver,
one real 500 mm rack and ISO 7089 M6 washers. **26 assertions** pass, including
contact on both sides of each washer and **1.10 mm** minimum clearance to the
conservative pinion envelope at nine stations. Set `LINEAR_MOUNT_EXPORT` to an
output directory to retain STEP witnesses and `contact.json`. This is not a
production component recipe, full-axis mounting model, stiffness or fatigue pass.

`LinearMountSteel` is the first rebuildable physical RM2 member. Both rails and
racks use its C45 receiver stock and the same neutral drilling; a welded host's
receiver is not a cast insert. Its recipe retains source identities/budgets,
locks the C45 material used by the thread policy, rejects stale holes after stock
shrinking, and is registered/classified for saved assemblies. Native checks
verify assembly reload plus computed mass/inertia exported exactly once by the
existing `AssemblyFemExport`. This verifies mass export, **not** a complete joint
compliance law or full mounting-factory acceptance.

## Cable-chain display checks (2026-10-25)

`haxeon.chain.json` compares `PackedCableChainMesh` with native
`CableChain.geometryAt` at seven travel positions: bounds, hollow volume within
1.5% chordal-bend discretization, closed/outward winding, finite transformed
coordinates and reusable vertex/normal storage. It also checks the routed-chain
recipe reload (23,407 assertions).

`haxeon.chain-bindings.json` checks both router beds at nine X/Y poses each,
including scene-motion wire reload and the moving X-chain datum carried by Y.
The display cap corners stay at the fixed/moving bracket connectors (540
assertions, 0.001 mm float32 comparison tolerance). It does not claim whole-router
save/reload: unrelated code-only components still prevent that. The app passes a
compiler-only build; no interactive editor visual check was performed. Compact
[source/log evidence](../../examples/sheet-router/mounting/cable-chain-validation.json)
is retained without executable or native build outputs.
