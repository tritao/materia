# Imported reference arms

Official UR and FAIRINO robot descriptions converted to Materia's existing articulated
scene format. The source URDF is the authority for transforms, axes, limits and declared
inertias. There is no ROS runtime or hardware driver dependency.

Run from the repository root:

```sh
python3 -m venv machinekit/examples/reference-arms/.venv
machinekit/examples/reference-arms/.venv/bin/pip install -r machinekit/examples/reference-arms/requirements.txt
machinekit/examples/reference-arms/.venv/bin/python machinekit/examples/reference-arms/prepare.py
./haxeon/scripts/haxeon run --project=machinekit/examples/reference-arms/haxeon.json
```

The initial models are `ur5e` and `fairino5_v6`. To prepare all catalogued descriptions,
use `prepare.py --all`. It attempts every model, checkpoints `build/preparation-report.json`,
and returns nonzero if any source is unavailable or invalid. Pass model IDs after `--`
to the Haxe example to export/check other prepared descriptions.

Each model directory below `build/` contains the expanded `robot.urdf`, hashed binary
visual/collision mesh streams, a readable `manifest.json`, its generated typed wire
representation, a `<model>.scene`, and `validation.json` after successful checking.
The typed wire representation is derived from the Haxe consumer schema, not separately
maintained field IDs. Scene files use metres and import through the same assembly bridge
as our authored CAD robots. Massless fixed URDF frames are retained as transforms on the
visible links rather than invented physical bodies.

The checker round-trips the saved scene, compares all source link poses against an
independent matrix FK oracle at zero, six single-axis jogs and 32 sampled configurations,
then checks IK candidates against compiled FK and limits. Multiple returned candidates
can be different physical turns of the same geometric branch. A missing EAIK solution
is checked with the existing numerical solver from a perturbed seed and counted explicitly.
This initial checker does not yet establish trajectory clearance or dynamic performance.

## Source and engineering boundaries

`sources.json` pins official repository revisions; the UR archive also has a pinned
SHA-256. `source-lock.json` pins the individual source files of prepared models.
Prepared manifests record input hashes and binary stream hashes; the consumer
rejects changed mesh streams or URDF. Generated vendor assets are ignored by Git.
UR's source notices distinguish BSD-licensed models from newer models with Graphical
Documentation terms. FAIRINO's description package currently declares `TODO: License
declaration`; redistribution permission remains unresolved and its assets are not bundled.

Visual meshes and source collision meshes are prepared separately. Imported collision
pieces currently use enclosing 26-DOPs of source collision meshes. These are conservative
convex approximations, not exact B-rep clearance evidence. Render colors currently use
the URDF material or a neutral fallback; embedded DAE textures are not reproduced.

Source mass/inertia is preserved explicitly, including inertial-frame rotation. Exterior
mesh volume is retained only as geometric metadata; its placeholder density must not be
used for material or load sizing. The imported URDF effort values are not verified drive
torque curves, and there are no invented motors or actuator capabilities.

UR uses the vendor `tool0` frame. Standard FAIRINO descriptions end at the vendor wrist3
origin; its physical flange/TCP relationship still needs checking against matching CAD.
Source rounded angles are preserved without snapping them to perfect right angles.

For source continuous joints, the checker uses an explicit +/-pi planning window for
finite IK turn enumeration; the saved scene retains the continuous source joint.
This window is not a mechanical limit. FR3-MT uses the existing numerical backend
because its source geometry has no EAIK decomposition. Numerical fallback does not
promise complete branches or persistent geometric branch labels.

Initial range verification exported 22 scenes: 14 UR models, seven standard FAIRINO
models and FR5-WML. All passed 39 source-pose checks each (858 total). FR3-WML and
FR3-WMS source URDFs reference an undeclared `tool_Link` and are rejected explicitly.
Five importer tests cover source hash rejection, coordinate conventions, finite input,
safe mesh paths and generated wire-field mapping. Run them with the preparation venv:

```sh
python -m unittest discover -s machinekit/examples/reference-arms -p 'test_*.py'
```

Optional static preview (requires NumPy and Matplotlib):

```sh
python3 machinekit/examples/reference-arms/render-preview.py
```

This produces `build/reference-preview.png`; it compares scale in illustrative
configurations, not identical TCP poses or task performance.

The remaining work and completion gates are tracked in [PLAN.md](PLAN.md).
