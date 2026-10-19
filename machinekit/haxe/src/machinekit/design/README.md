# Manufacturing and assembly design checks

`DesignInterfaces` is an optional `MachineComponent` facet. Components declare
holes, registers/datums, lubrication ports and required material regions in
local coordinates. `DesignInterfaces.of(part)` retrieves it. Existing components
without this facet keep their existing behavior; absence does not mean that
manufacturability has been verified.

A `HoleInterface` describes the drilling face, inward +Z axis, bore radius,
complete drilling depth, minimum surrounding material and optional counterbore,
blind thread and machining approach. Blind threads carry a typed `ThreadSpec`, selected tap-drill size, usable
thread, cylindrical drilling and drill-point allowance. `drillingTools()` builds owned
CAD cutting tools from the same data that the checker evaluates. Recipes
regenerate the facet; no live CAD objects are saved in metadata.

Assemblies add portable requirements using `addDesignRequirement`:

- `FastenerJoint` binds an actual screw to a tapped interface and an ordered-by-
  position set of clearance interfaces. The checker derives grip from the
  installed poses, checks nominal thread profile/diameter/pitch/hand, axes/direction, thread engagement,
  bottoming, unthreaded shank, clearance and counterbore bearing. The clamped
  interfaces must cover the grip without gaps or overlaps.
  Screws implement the existing `ScrewFastener` contract, including a bearing
  frame, head clearance and bearing radii. Socket-head screws and hex bolts use
  the same evaluator.
- `AccessPath` anchors clearance envelopes to a component interface, with named
  joint coordinates and either `WholeAssembly` or `InstalledMembers` obstacles.
  Use explicit installed members for a bench/assembly phase. Use the whole
  assembly for service access so guards and surrounding structure added by a
  parent machine are checked too.

Call `AssemblyDesignChecks.check(machine).throwIfErrors()` at the desired review
or export boundary. It evaluates actual cut solids, rather than just declared
nominal edge distances. The generic evaluator contains no machine dimensions,
component IDs, world-axis assumptions or mill dependencies. Tool paths are
conservative **continuous clearance envelopes supplied by the assembly designer**;
the evaluator does not infer a motion planner or an assembly order from final
geometry. Provide approach, withdrawal, hose bend and handling envelopes.

Requirements survive snapshots and recipe encoding as an optional field of the
assembly description. Nesting remaps member references, installed-member lists,
requirement IDs and service joint names. Envelope frames remain local to their
anchor, including when the whole machine rotates. The getter returns a deep
copy. Descriptions predating this optional field have no assembly requirements.

The mill's `MillYNutDesignChecks.declare` states assembly intent and its wrapper
supplies the surrounding frame/head. Its adapter, saddle and ballnut carry the
component interfaces. Another machine can use the same evaluator without that
builder. These are geometric/process requirements, not strength, fatigue or
supplier qualification; dimensions and tool envelopes must match the selected
hardware and process.

Run the independent, non-mill pump fixture tests with:

```sh
./haxeon/scripts/haxeon run --project=machinekit/tests/haxeon.design-interfaces.json
```

They exercise typed recipe rebuilding, nesting/rotation, parent guards, bench
phases, saved service poses and negative manufacturing/assembly cases. The
bench mill's existing XY suite retains its geometry and access regressions.
