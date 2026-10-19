# SceneKit

SceneKit provides the retained scene database, render planning and GPU
execution, spatial queries and picking, and interaction state for the Materia
workspace. It consumes sibling NativeKit for platform and GPU services and
sibling Haxeon for binding validation.

The source is split into three dependency-ordered libraries:

- `scene`: retained data, transactions, snapshots, and stable identities;
- `scene_render`: render plans, spatial indexing, picking, and GPU execution;
- `scene_interaction`: selection, hover, and interaction presentation.

## Scene identity

An entity is a shared logical source. A node identifies one placement in the
scene hierarchy, and a `SceneNode` is the immutable state of that placement in
a `SceneSnapshot`:

```text
EntityId (shared logical source)
  ├── NodeId (front-left placement)
  ├── NodeId (front-right placement)
  ├── NodeId (rear-left placement)
  └── NodeId (rear-right placement)

SceneSnapshot.findNode(NodeId) → SceneNode
```

For example, a vehicle can share one wheel entity across four nodes. Each node
has its own parent and transform while referring to the same geometry and
material resources. In CAD terms, a node represents one occurrence or placement
of an entity.

```haxe
var tx = scene.beginTransaction();
var vehicle = tx.createNode();
var frontLeft = tx.createNode();
tx.setParent(frontLeft, vehicle);
tx.setSourceEntity(frontLeft, wheelEntity);
tx.setGeometry(frontLeft, wheelGeometry);
tx.commit();

var snapshot = scene.snapshot();
var state:SceneNode = snapshot.findNode(frontLeft);
```

Configure, build, and test from this directory:

```sh
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build -L scene --output-on-failure
```

Override sibling checkout locations with `NKSCENE_NATIVEKIT_DIR` at CMake
configure time, or `NATIVEKIT_DIR` and `HAXEON_DIR` when running binding tools.
The Haxeon end-to-end smoke test uses the SceneKit build directory:

```sh
NATIVEKIT_BUILD="$PWD/build" scene_render/tools/test-haxeon.sh
```

## Interaction appearance

Selection and hover use the highlight material's RGB as a display tint. They
retain each primitive's source material, including opacity, base alpha, alpha
mode/cutoff, textures, roughness, and sidedness. Highlight alpha is ignored.
Tints inherit through subtrees; the nearest override wins within a layer, and
hover takes precedence over selection. Base node/source material overrides still
replace materials before tinting. Face-hover overlays must use the source
surface material and enter the hover layer rather than the base material layer.
Tint-only updates change instance data without rebuilding material batches.

## Reusable translation gizmos

`scene_interaction/haxe` is a pure Haxe package (`scenekit-gizmo`, namespace
`nativekit.scene.gizmo`) for perspective translation handles. It has no NativeKit
FFI, UI, scene-entity, document or undo dependencies. The root SceneKit manifest
also exposes its source root.

```haxe
var eye = camera.eyePosition();
var view = new GizmoCamera(new GizmoVec3(eye[0], eye[1], eye[2]),
  new GizmoVec3(camera.targetX, camera.targetY, camera.targetZ),
  new GizmoVec3(0, 0, 1), viewportWidth, viewportHeight,
  PerspectiveCamera.FOV_Y, camera.clipNear, camera.clipFar);
var layout = new TranslationGizmo(view, worldPivot);
var handle = layout.pick(pointerX, pointerY);
if (handle != null) {
  var drag = layout.begin(handle, pointerX, pointerY, snapIncrement);
  var proposed = drag.update(nextPointerX, nextPointerY);
  // Proposed is absolute World displacement from pointer-down, or null.
  // On release, pass drag.finish() to the consumer's commit operation.
  // On cancellation, drag.cancel(); discard the consumer's transient preview.
}
```

First scope is world X/Y/Z axes and XY/XZ/YZ pads, one pivot. `shapes()` returns
immutable labelled segments or convex quads in logical viewport pixels; drawing
and picking use the same geometry. Two-point primitives describe an axis shaft
and tip; the UI may decorate that tip with an arrow and label. Plane pads require
an inside-polygon hit. Handle picking precedes normal scene selection in the
consumer. Picking ties resolve by screen distance, depth, then stable handle
order. Arrays are detached copies and vectors/points are immutable.

Default projected size is approximately 90 logical pixels, hit radius 8 pixels.
The world size derives from pivot eye-depth and perspective FOV. View-aligned
axes (absolute eye dot above 0.99), edge-on planes (normal eye dot below 0.2),
axis segments below 12 pixels and plane pads below 36 square pixels are absent
from both drawing and picking. Pivot/vertices outside the camera depth range are
rejected; an offscreen pivot has no handles. The UI clips overlay geometry to
the viewport. Handle geometry can extend beyond viewport boundaries, but picking
is restricted to the viewport. No GPU picking/readback is required.

A drag freezes camera, pivot, handle, constraint plane, starting intersection and
snap increment. Axis planes face the eye while containing the axis; plane pads
use their World plane. Proposals derive from current minus initial plane hits,
then project into the constrained axis/plane basis. Snap rounds displacement
coefficients, not the pivot's absolute grid position. Parallel rays (absolute
normal dot below 1e-6), behind-eye intersections and nonfinite inputs return null
and retain `lastDelta`. The next valid update can recover. Finish/cancel close
the solver; late updates throw. `translatedShapes(delta)` follows an accepted
proposal using the captured size/visible handles instead of changing the active
constraint as the pivot moves. Perspective projection is the only supported
camera mode in this checkpoint.

Consumers own pointer capture, hover/active colours and labels, camera locking,
selection/document/asset/viewport revision guards and cancellation. Documents
converts World displacement to local position through its captured parent and
validates its detached World. The solver does not decide whether a proposal can
be admitted or commit any scene/source/history state. When validation rejects a
proposal, use the last admitted displacement for the displayed overlay, rather
than the solver's latest unadmitted proposal. Resize, focus loss, camera movement
or canonical-state changes cancel the consumer gesture.

Tests (use an available Haxeon executable, tested with Beartooth's pinned 73e448aa):

```sh
haxeon run --project scenekit/scene_interaction/tests/gizmo/haxeon.json
python3 scenekit/scene_interaction/tests/camera-gizmo/check-beartooth.py \
  --haxeon /path/to/beartooth/haxeon/scripts/haxeon \
  --preview-project /path/to/beartooth/libs/document-world/preview/haxeon.json
```

The second check creates a temporary project around Beartooth's accepted render
boundary and compares frozen rays/projections with its actual PerspectiveCamera.
It also feeds solver displacement into the Documents transient preview under a
rotated/nonuniformly scaled parent and verifies single commit/exact-source undo.
It does not edit dependency pins. Pure tests cover six constraints, geometry/pick
agreement, immutable data, zoom/viewport sizing, snapping and off-grid grab,
parallel/behind-eye recovery, clip rejection and lifecycle. Desktop integration,
local-orientation handles, rotation/scale and final usability tuning remain later
gates. C++/C ABI contracts are unchanged.

Reference techniques were reviewed at these revisions; no third-party runtime
or implementation code was imported:

- [Godot viewport, 232e6b14](https://github.com/godotengine/godot/blob/232e6b14abf6e590b41d60f11704e42de22bff72/editor/scene/3d/node_3d_editor_viewport.cpp)
- [Three.js TransformControls, a505186b](https://github.com/mrdoob/three.js/blob/a505186b5185088bd800de092568d0643e96bd75/examples/jsm/controls/TransformControls.js)
- [ImGuizmo, 105e4fde](https://github.com/CedricGuillemet/ImGuizmo/blob/105e4fde45e4e74aae8973d98750820ef7aca7e2/src/ImGuizmo.cpp)

Translation gizmos accept an optional immutable `GizmoBasis`; the default remains World axes. `GizmoBasis.fromQuaternion` produces rotation-only local axes from the consumer's composed World orientation. Scale and shear do not define movement axes. Drawing, picking, frozen drag constraints and snap coefficients share this basis. Snapping measures World distance along each chosen axis, relative to pointer-down.

`RotationGizmo` and `RotationDrag` provide projected axis rings and signed degrees about a frozen World axis. Picking uses the drawn polyline, hides near-edge-on/near-clipped rings, and rejects unstable ray/plane updates while retaining the last angle. Successive samples unwrap the ±180° boundary; snapping uses movement increments in degrees. Consumers own TRS admission, authored Euler branches and atomic publication. Ring constraints and orientation remain frozen during capture.

`ScaleGizmo` provides local X/Y/Z shafts with square tips and a uniform center pad. Drawing and picking share `GizmoShapeHit` with translation; the center pad takes priority where it covers a shaft. `ScaleDrag` freezes projected axis sensitivity at pointer-down and emits an absolute positive factor relative to 1. Uniform drag follows a fixed logical-pixel diagonal; axis drag follows the captured projected local axis. Factor snapping applies to movement increments about 1; invalid, zero, negative or excessively large factors retain the last valid proposal. Scale factors are dimensionless and independent of the entity's initial scale. Consumers own local TRS materialization and whole-subtree admission.

The optional `scene_interaction/haxe/uikit/src` adapter extracts transform overlay presentation and pointer-capture validity from the Documents preview. It borrows existing UIKit services; the default pure math package does not include these sources. See `scene_interaction/haxe/uikit/README.md` for the consumer contract and independent fixture.
