# Cobot arm

The selected hardware reference for the first Reach850 wrist bench prototype is
**ZeroErr eRob80H100I V6**, with holding brake, EtherCAT and a requested HM
high-precision multiturn output encoder. The complete supplier ordering code,
quote and availability still need confirmation. This selection is not yet
integrated into `CobotArm`; the geometry below remains the generic module model.
The comparison, source revisions and downloaded-file hashes are recorded in
[`actuator-selection.json`](actuator-selection.json).

The [V6 supplier drawing](https://www.zeroerr.cn/static/download/eRobI/eRob80I/V6/eRob80HxxI-XX-18EX_V6_MC2E.EX.XX.zip)
specifies 31 Nm rated, 70 Nm start/stop peak, and 30 rpm at rated torque and 48 V.
It lists an 80 mm diameter, 64.2 mm nominal length, 67.5 mm axial extent on the
drawing, 18 mm through-bore and 1.09 kg with brake. Connector and cable clearance
must be added. The current wrist assumption is 28 Nm rated at 24 rpm, 56 Nm peak
and 30 rpm maximum, with a 96 x 40 mm module envelope. These are initial sizing
assumptions, not validated hardware requirements.

The brake-equipped MYACTUATOR RH-17 alternative has a 104 mm length and 54 Nm
peak rating. CubeMars AKH70-48 offers more torque but lists 12 arcmin planetary
backlash; its power-off holding brake was not verified. Encoder resolution and
encoder positioning accuracy must not be compared directly with mechanical
backlash or whole-arm repeatability. Selection is based on packaging, interface
documentation and published capability; thermal duty, bearing loads, brake
performance and loaded accuracy still need joint-specific validation. Mounts and
cable routing must be redesigned from the V6 drawing before replacing the
generic joint; the vendor module does not use our assumed six-M4 interface.

The [manufacturer arm reference packages](https://www.zeroerr.cn/download/eRob.html)
provide a stronger whole-arm starting point. Their assembly drawings identify:

| Reference payload label | J1 / J2 | J3 | J4 / J5 / J6 |
| --- | --- | --- | --- |
| 3 kg | eRob80T | eRob80T | eRob70T |
| 5 kg | eRob110T | eRob110T | eRob70T |
| 10 kg | eRob142T | eRob110T | eRob80T |

Use **eRobot 5 as the mechanical baseline for the 5 kg arm**. Its T-shaped
modules supply perpendicular mounting faces, allowing straight hollow links
instead of our separate routed elbows. The drawings include thickened locating
ends, radial fasteners, third-axis housing/output adapters and cable parts. Its
upper-arm axis spacing is 425 mm but the physical upper-arm link is 305 mm long;
the forearm axis spacing is 401 mm and wrist offsets are 86 / 86 mm. The upper-arm
link drawing specifies 7075-T6 with natural anodizing. These details must not be
replaced with nominal tube lengths or our existing fastener patterns.

The 80H100I V6 selection above remains a **single-joint bench reference**. The
5 kg reference's smaller eRob70T wrists warrant a fresh load and packaging study
before freezing the whole-arm actuator sizes. Assembly labels do not resolve
ratios, brake/encoder ordering codes or compatibility with current revisions.
The packages include assembly/component STEP, PDF and SolidWorks drawings;
their payload labels alone do not validate our tool inertia or dynamic duty.
Reviewed drawing hashes and observations are recorded in `actuator-selection.json`.
The articulated supplier-based variant is implemented in
[`zeroerr-arm`](../zeroerr-arm/README.md), using the pinned T modules and straight
links. Its six joints remain passive until the hardware configuration is resolved.

The four size classes retain their reference DH axes and flange transforms. Wrist
links 4 and 5 use `CobotWristLink`: broad locating hubs joined by a hollow elbow,
with a saddle relieved around the next module. Their section is sized from the
module diameters, rather than shrinking a tube to fit the axial gap. An offset
that cannot accommodate the hub, shell walls and clearance is rejected.

The assumed construction uses a 2.5 mm aluminium shell, 5 mm hubs, 1.5 mm housing
clearance, 2 mm annular locating shoulders, and six M4 screws at each interface.
The module stators have matching pilot seats and nominal threaded-hole envelopes.
The three rotating output flanges are separate `CobotWristRotor` members carried by
the driven links; their holes and screws follow the joint rotation. Their annular
skirts clear the existing gearheads. Motor and gearbox dimensions are unchanged.
`CobotToolLink` encloses the final output in a smooth hollow taper, with a 5 mm
input hub, 9 mm terminal mounting face and matching M6 receivers. Its M4 access
pattern is clocked 15 degrees from the four-hole tool-flange pattern; construction
rejects a layout with less than 1 mm of web between them. The tool flange retains
its reference position and orientation. The rotor recess represents bearing running clearance; rolling elements, bearing
selection, preload, sealing and fatigue life remain engineering assumptions.

The nominal elbow shell diameters and checked arm masses (including wrist
flanges and fasteners, excluding the tool and offboard drives) are:

| Class | Shell diameter | Arm mass |
| --- | --- | --- |
| Reach500 | 63 mm | 15.026 kg |
| Reach850 | 80.64 mm | 21.470 kg |
| Reach900 | 92.4 mm | 36.892 kg |
| Reach1300 | 92.4 mm | 37.229 kg |

## Validation

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/cobot-arm/haxeon.json
LD_LIBRARY_PATH="$PWD/haxeon/.tools/hashlink:$PWD/haxeon/out:$PWD/machinekit/examples/cobot-arm/build/host/native/cadkit:$PWD/machinekit/examples/cobot-arm/build/host/native/collisionkit-native" \
  ./haxeon/.tools/hashlink/hl machinekit/examples/cobot-arm/build/host/main.hl
```

Eleven sampled poses include both wrist bend directions and the preview motion loop. The checks compare the flange against independent DH arithmetic, check CAD solid
validity and interference including the wrist screws, verify mounting-face
contact, module recipe round trips, undersized-layout rejection, mass and static
holding torque at the reference payload. CAD slices through each elbow measure
minimum principal second moments of area at three sections to guard against reintroducing a thin
neck. This is a section-stiffness regression, not a whole-elbow stress, deflection,
fastener-load or fatigue analysis. Sampled poses do not certify the entire joint
range. Existing reach/collision certificates for a cell need regeneration when
its arm geometry changes.

## FAIRINO and UR reference comparison

Reference files were retrieved on 2026-10-15. The FAIRINO FR5 V6.0 STEP and FR5
mechanical drawings come from the [official download centre](https://www.fairino.com/DOWNLOAD2).
The UR STEP download requires login; the comparison instead uses the manufacturer's
[UR5e visual meshes and kinematic configuration](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).
Exact source revisions, download URLs, measurement methods and file hashes are
recorded in `reference-sources.json`. Downloaded vendor assets stay outside the
repository.

| Measurement | FAIRINO FR5 | UR5e | Materia Reach850 |
| --- | --- | --- | --- |
| Wrist casing width | 80 mm cylindrical faces in STEP | about 78.7 mm mesh X extent | 96 mm module diameter |
| Upper-arm reference length | 425 mm, supplied drawing | 425 mm, kinematic configuration | 425 mm |
| Forearm reference length | 395 mm, supplied drawing | 392.2 mm, kinematic configuration | 392.2 mm |
| Wrist offsets | 102 / 100 mm, supplied drawing | 99.7 / 99.6 mm | 99.7 / 99.6 mm |

The drawing and STEP are separately versioned manufacturer downloads; drawing
dimensions and mesh extents are labelled rather than treated as calibrated
measurements of the same assembly. Whole-arm images use the supplied extended
reference poses; wrist crops share a physical scale. Mesh measurements describe
external surfaces and give no evidence about motor, gearbox or bearing internals.

The comparison motivates fuller elbow shells close to the housing diameter,
shorter external bend routing, 1 mm turned hub edge breaks and a smooth terminal housing
that tapers to the tool flange. Materia retains its current motor and gearbox
sizes, torque assumptions and reference joint axes. In particular, the 96 mm
Reach850 wrist module is not reduced to an 80 mm vendor casing: matching that
external size would require an independently justified internal package.
