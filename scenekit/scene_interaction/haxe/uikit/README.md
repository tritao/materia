# Optional UIKit gizmo adapter

Opt in by adding this directory's `src` to your application source roots, alongside a dependency on the pure `scenekit-gizmo` package and your existing `haxeon-ui` package. The pure package's default manifest includes only its `src`, so math-only consumers do not acquire UIKit or native UI dependencies.

`TransformGizmoOverlay` borrows a `FontCollection` and owns projected layout, picking, hover presentation, drawing and cached glyph layouts. Its layout call takes an opaque consumer generation key, `GizmoTool`, a frozen camera/pivot, movement basis and local scale basis. Generation changes invalidate stale geometry; size changes above half a logical pixel clear it. Drawing translation during a gesture takes the consumer's admitted absolute World displacement. Rotation and scale constraint layouts stay frozen. Dispose the overlay before its borrowed font service.

`GizmoPointerCapture` starts from a routed UIKit PointerDown event and captures that pointer. It stores the consumer generation and logical viewport dimensions; `matches` rejects generation changes, nonfinite dimensions and significant resize. `clear` clears only adapter state. The consumer must end native/UI capture through routed release/cancel events, including keyboard cancellation.

The generation key must cover selection, source revision, asset/camera generations, tool and orientation. Consumers own opaque model tokens, admission, last-valid rollback, renderer placement, commit/cancel and history. The adapter imports no Documents, World, ProceduralKit or asset code and defines no authoring format.

The independent fixture under `../../tests/uikit-gizmo` consumes UIKit and gizmos without document/asset dependencies. Run its `check-beartooth.py` against the accepted preview manifest to verify compatibility without changing any pins.
