# End effector (EOAT) follow-up tasks

Possible expansions for the end-effector layer described in
`END_EFFECTOR_PLAN.md`. Effort: **S** about one PR, **M** a few PRs, **L** a
project of its own.

## Real parts

- [ ] **Vendor catalogs (M per vendor).** Eins, Schunk, Zimmer, SMC or Piab via
  `Catalog<T>`: coupling key, ports, declared mass and centre, envelope
  geometry and provenance, using the same API as the generic parts.
- [ ] **Vendor STEP files as envelopes (M).** Load a vendor model for
  collision and appearance, then declare connectors and ports by hand. Vendor
  files usually allow internal use but not redistribution, so keep them outside
  the repository.
- [ ] **Fittings and tubing as components (S–M).** Push-in fittings, reducers,
  tube lengths and bulkheads, so a mismatched-interface warning can suggest the
  adapter that resolves it.

## Authoring and review

- [ ] **Document and editor persistence (L).** Store ports, connections, end
  effectors and changer sets in `cadkit.parametric` documents, show them in the
  editor, and pick working frames visually. This is the largest structural gap:
  end effectors are code-only today.
- [ ] **Pneumatic schematic view (M).** Generate a service diagram from the
  port graph (robot air → changer → manifold → ejector → cup).
- [ ] **Design report (S).** Per configuration: BOM, mass and centre of mass
  against the robot payload envelope, working frames, service chains and
  validation warnings.

## Engineering checks

- [ ] **Payload with workpiece (S).** Add the grasped part's mass and centre of
  mass per pick and check the robot load chart across the motion, not only at
  a static pose.
- [ ] **Suction capacity (M).** Cup area × vacuum level against part mass,
  acceleration and a safety factor. Needs vacuum level and cup area as port or
  component properties.
- [ ] **Air consumption and cycle time (M).** Compressed-air use per cycle from
  actuator volumes and ejector flow.
- [ ] **Tool-change feasibility (M).** Tool stand poses, approach and retract
  paths, and clearance between parked tools and the robot's reach.

## Simulation fidelity

- [ ] **State-dependent collision (M).** Open and closed gripper hulls selected
  by the runtime gripper state, instead of one full-stroke envelope.
- [ ] **Contact-based grasping (M–L).** Hold parts through suction or friction
  contacts rather than attachment, so parts can slip or drop under
  acceleration; feeds the suction capacity check.
- [ ] **Hose and cable dress packs (M).** A swept volume or simple cable model
  for hose loops, a common real-world collision.

## Planning and runtime

- [x] **Planner uses tool collision (M).** Check the tool's convex pieces
  against the cell during motion planning, with the proximity margin as a
  planning clearance; `FinishSurface` and its Paint/Sand variants accept the
  mounted tool and cell obstacles.
- [ ] **Tool-aware task skills (M).** Choose a tool configuration and working
  frame per part in AutomationKit `Pick`/`Place` tasks, and schedule tool
  changes.
- [ ] **Runtime bindings from the design (S–M).** Generate valve channels,
  vacuum sensors and changer-lock bindings from the design's ports instead of
  hand-written control bindings.

## Beyond end effectors

- [ ] **Machine utilities (M).** Electrical power and signal ports on
  `NemaStepper`, `LinearAxis` and the picking station, giving any MachineKit
  assembly wiring and I/O lists.
- [ ] **Rotary unions and slip rings (S).** Model them as bridges for rotating
  stages.
- [ ] **Cable and I/O list export (M).** Export for electrical CAD or PLC
  configuration.

## Suggested priorities

1. Suction capacity and payload-with-workpiece checks: they answer the
   questions people ask about a tool and build on existing mass and port data.
2. One vendor catalog slice: makes the layer usable with real parts and tests
   the abstractions against a real vendor.
3. Document and editor persistence: needed once someone must author tools
   interactively.
