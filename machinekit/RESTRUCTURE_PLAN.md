# MachineKit restructuring (MK0–MK9)

Planned 2026-10-04 from an outside architecture review, checked against main 712019dbc. The domain
model and the dependency direction (ProjectKit ← CadKit ← MachineKit ← adapters) stay as they are.
This plan simplifies the implementation around them.

## Verified starting point

- `MachineAssembly.hx` is 1,656 lines. It holds components, joints, couplings, subassemblies, ports,
  service connections, bridges, conversions, transmissions, belt paths, motors, encoders, BOM extras,
  mass, validation, the saved form and the CadKit export.
- `include()` copies a child flat into the parent with prefixed IDs. Saving then rebuilds a nested
  ProjectKit definition (`nestedMechanical`, `syncNestedConnectors`, `exposeNested` with generated
  `__machinekit_N` connectors), and loading flattens it again and rebuilds the child
  (`rebuildIncluded`).
- `MachineAssemblyDescription.hx` copies ProjectKit's assembly schema as `Frozen*` typedefs, and
  `FrozenAssemblyDefinitions` converts between them by encoding to JSON and decoding back.
- `AssemblySideRecord` has optional `endEffector`, `changer` and `tools` fields;
  `EndEffector`/`EndEffectorSet` override `describe()`.
- `ComponentCapability` imports `machinekit.welding` (`ArcTorch`, `WeldingSupply`, `WireFeed`,
  `WorkReturn`), and also has `PlanarScanner`.
- `RuntimePortIntent` is used only by `MachineComponent` and one test.
- `AssemblyPreview` builds `SceneArtifactRobotTool`/`SceneArtifactRobotSensor` and imports welding.
- `addMate`/`addConstraint` take `kind:String` and `cast` it to `AssemblyJointType`.
- `SteppedShaft` (sections, keyways, grooves, detail) and the routed-hose recipe store JSON inside
  `Token` values.
- `MachineKitComponents` is a static list plus a global `extensions` array; one caller outside
  MachineKit (`PickingStationRecipes`) registers into it.
- `machinekit.units` (`Millimetres`, `Metres`, `Kilograms`, `KgMm2`) is half adopted; public APIs use
  `Float` with unit-suffixed names.
- Haxeon wire schemas cannot be recursive (E1024), so a nested saved form needs a table of levels
  rather than a self-referencing record.
- Already done, contrary to the review: the persisted drive record is the typed `@:wire enum
  Transmission`, and screw supports are the typed `ScrewSupport`. What is left is that their member
  references are plain strings.

## Decisions

- **No compatibility layers.** The saved schema is bumped once (MK2/MK3), with no migration reader.
  Example documents and fixtures are regenerated.
- **`MachineAssembly` keeps its API because it is a good API**, not to spare callers. Methods change
  where a typed parameter replaces a string.
- **Authoring is hierarchical; exports are flat.** A child assembly stays a child. CadKit, RobotKit,
  mass and validation work on one flattened view, built by one function and cached.
- **The saved form keeps the nested ProjectKit definition**, because the CadKit document layer
  stores it (`AssemblyDocuments.fromDefinition`) and the editor shows its hierarchy. It is written
  straight from the hierarchy, with each subassembly a table entry, and beside it the MachineKit facts
  of each level (`MachineLevelRecord`, keyed by subassembly path). A joint that reaches into a
  subassembly goes through a connector the subassembly exposes, named `~path/connector`. The flat
  ProjectKit definition, with derived couplings and belt networks, is what `definition()` returns.
- **`EndEffector` stays a subclass in memory**: an end effector is an assembly with a mount, frames and
  exclusions. Only its persistence moves out of the generic record.
- **Welding process code goes to ProcessKit**, together with RobotKit R3. Welding hardware stays.
- **Units:** delete `machinekit.units`. Unit-suffixed names are the convention; real quantity types,
  if added, belong in ProjectKit at cross-domain boundaries.

## Phases

### MK0 — small typed cleanups (no schema change)
- `addMate`, `addMateOnAxis`, `addConstraint` and `addConstraintOnAxis` take `AssemblyJointType`.
- Delete `RuntimePortIntent` and `MachineComponent.runtimePortIntents()`.
- Delete `machinekit.units`; callers use `Float` with unit-suffixed names.

### MK1 — `ComponentRegistry`
- A `ComponentRegistry` instance with `register`, `byId` and `all`.
- Built-ins register by module (`StandardComponents`, `DriveComponents`, `TransmissionComponents`,
  `PneumaticComponents`, `RoboticsComponents`, `StructuralComponents`, `WeldingComponents`), replacing
  `MachineKitAdditionalRecipes`.
- `MachineKitComponents.defaultRegistry()` is the shared registry. Decoding takes a registry, with
  the default as the default argument.

### MK2 + MK3 — hierarchy and saved form (one schema bump)
- `include()` keeps the child `MachineAssembly` (a snapshot) with its id and pose. Records authored on
  the parent may name nested members by path (`arm/link1`).
- A single recursive flatten builds the view used by the CadKit export, transmissions, motors,
  services, mass and validation. It is cached and dropped on every edit.
- `definition()` returns the flat ProjectKit `AssemblyDefinition`. `MachineKitRobotCompiler` uses it
  instead of thawing `describe().mechanical`.
- Delete `nestedEntries`, `nestedMechanical`, `syncNestedConnectors`, `exposeNested` and
  `rebuildIncluded`, the `Frozen*` typedefs, `FrozenAssemblyDefinitions` and its compile-fail probes.
- New saved form (schema 10): `MachineAssemblyDescription {schemaVersion, mechanical, machine,
  subassemblies}`, where `mechanical` is the nested `AssemblyDefinition` itself and `subassemblies`
  holds `{path, definition, machine}` per subassembly. Derived data (belt networks, actuators) is not
  saved.
- `EndEffectorDescription {assembly, endEffector}` and `EndEffectorSetDescription {base, changer,
  tools}` live in `machinekit.robotics`; `EndEffector`/`EndEffectorSet` have their own `encode`,
  `decode` and `fromDescription`. Documents store a `MachineDocumentKind` enum (`Assembly`,
  `Effector`, `EffectorSet`), and port connections carry the level they belong to.

### MK4 — split the internals
- `MachineAssembly` becomes a façade over `MechanicalAssembly` (members, subassemblies, connectors,
  joints, couplings), `ServiceNetwork` (ports, connections, bridges, conversions, exposures, tracing),
  `DriveSystem` (transmissions, belt paths, motors, encoders) and `AssemblyInventory` (BOM extras, mass).
- Checks are split the same way and gathered by `check()`.

### MK5 — typed component facets
- Replace the `ComponentCapability` enum with the `ComponentFacet` interface (`check`, `describe`)
  and one class per kind in its domain's package, each with a static `of(component)`:
  `CouplingFacet` (component), `SuctionFacet`, `VacuumSourceFacet`, `VacuumActuatorFacet`,
  `VacuumValveFacet`, `VacuumPressureSensorFacet` (pneumatic), `GripFacet`, `ChangerLockFacet`
  (robotics), `ArcTorchFacet`, `WeldingSupplyFacet`, `WireFeedFacet`, `WorkReturnFacet` (welding),
  `PlanarScannerFacet` (new `machinekit.sensing`). The component core no longer imports welding.

### MK6 — structured recipe values (schema bump)
- `ComponentValue` gains `Vector`, `List` and `Record`. `SteppedShaft` and the routed hose stop
  storing JSON in tokens. The Inspector edits the new values.

### MK7 — runtime projections out of MachineKit
- `AssemblyPreview` keeps geometry, definition and state. Tool and sensor records (`RobotScene`) and
  runtime channels (`EndEffectorControls`) move to a new package, `machinekit/robot`
  (`machinekit-robot`, Haxe package `machinekit.robot`). It depends only on ProjectKit, CadKit and
  MachineKit, so the examples that need it do not pull in RobotKit's native builds; cadbridge
  depends on it.

### MK8 — welding split (with RobotKit R3)
- Torch, power source, feeder, cylinder and clamp stay. `Weldment`, `WeldSeam(s)` and
  `WeldingRecipe` move to ProcessKit.

### MK9 — renames (last, one mechanical commit)
- `machinekit.motion` → `machinekit.drive`; `Transmission`, `Sense` and `AllowanceEdit` →
  `machinekit.transmission`.
- Optional later: `FrameAssembly.asComponent()` (a weldment as one rigid component with a cut list).

## Coordination

- `machine-tending` has unmerged MachineKit changes (`MachineAssembly` +29 lines, `AssemblyPreview`,
  `ComponentCapability`, `MachineComponent`, `MachineKitComponents`, new motion/pneumatic/milling
  parts). Whichever lands second rebases; MK5 must add facets for its new capabilities.
- MK7 and MK8 go with RobotKit R3 (ProcessKit). MK9 waits until no branch has open MachineKit edits.

## Porting a branch onto the restructure

| Before | After |
|---|---|
| `component.capabilities()` + `switch` | `XFacet.of(component)` (e.g. `SuctionFacet.of`), or `component.facets()` with `Std.isOfType` |
| `addCapability(Suction(...))` | `addFacet(new SuctionFacet(...))`; a new kind is a new class implementing `ComponentFacet` in its domain's package |
| `component.coupling()` as `{key, connector}` | `component.coupling()` returns `CouplingFacet` |
| `FrozenAssemblyDefinitions.thaw(a.describe().mechanical)` | `a.definition()` for the flat, derived definition; `a.describe().mechanical` is the nested one |
| `describe().mechanical.elasticNetworks` / derived couplings | `definition()` (the saved form holds no derived data) |
| `describe().machine.tools/changer/endEffector` | `EndEffector.describeEndEffector()`, `EndEffectorSet.describeSet()` |
| `description.machine.<records>` of included members | `description.subassemblies[i].machine` (per level, local paths) |
| `MachineKitComponents.byId/all/register` | `MachineKitComponents.defaultRegistry().byId/all/register` |
| `MachineKitAdditionalRecipes` | the part class's own `recipeType()`, registered in its package's `*Components` |
| `AssemblyPreview.robotTools/robotSensors` | `machinekit.robot.RobotScene.*` (package `machinekit-robot`, add it to haxeon.json) |
| `machinekit.robotics.EndEffectorControls` | `machinekit.robot.EndEffectorControls` |
| `machinekit.units.*` | plain `Float` with unit-suffixed names |
| editing `MachineAssembly` internals | the level parts: `MechanicalAssembly`, `ServiceNetwork`, `DriveSystem`, `AssemblyInventory`; derived data in `DriveSystem.derive` |

## Gate (each phase)

The MachineKit suite (`machinekit/scripts/test-haxeon`, which builds every example), the CadKit
nesting smoke tests, the MotionKit tests that compile MachineKit assemblies, the app compile, and
regenerated example documents.
