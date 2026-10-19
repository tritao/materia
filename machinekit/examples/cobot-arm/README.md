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

The four size classes retain their reference DH axes, motor/gearbox envelopes and
flange transforms. The shoulder and wrist links 1, 4 and 5 use `CobotWristLink`:
broad locating hubs joined by a hollow elbow,
with a saddle relieved around the next module. Their section is sized from the
module diameters, rather than shrinking a tube to fit the axial gap. An offset
that cannot accommodate the hub, shell walls and clearance is rejected.

The assumed construction uses a 2.5 mm aluminium shell, 5 mm hubs, 1.5 mm housing
clearance, 2 mm annular locating shoulders, and six M4 screws at each interface.
The module stators have matching pilot seats and nominal threaded-hole envelopes.
All six rotating output flanges are separate `CobotWristRotor` members carried by
the driven links; their holes and screws follow the joint rotation. Their annular
skirts clear the existing gearheads. Motor and gearbox dimensions are unchanged.
`CobotToolLink` encloses the final output in a smooth hollow taper, with a 5 mm
input hub, 9 mm terminal mounting face and matching M6 receivers. Its M4 access
pattern is clocked 15 degrees from the four-hole tool-flange pattern; construction
rejects a layout with less than 1 mm of web between them. The tool flange retains
its reference position and orientation. The rotor recess represents bearing running clearance; rolling elements, bearing
selection, preload, sealing and fatigue life remain engineering assumptions.

The upper arm and forearm use `CobotArmLink`: tapered hollow bridges with 2.5 mm
walls, flat service lands, locating pilots and six M4 screws at both module
interfaces. End hubs reach the actual housing annuli rather than ending inside
the next module's gearhead pocket. The upper arm has a broader section; the
forearm retains its previous proximal radius and tapers toward the wrist to
preserve clearance when the shortest arm folds back on itself. Motor and gearbox
sizes, ratios and drive ratings are unchanged.

Each long link has a separate 2 mm aluminium service lid, a nominal 0.25 mm NBR
gasket, four M3 screws and internal screw bosses. Screw-access bores reach the
module fasteners. Two 9 mm cable-entry ports open into a continuous bridge duct;
an 8.4 mm gauge checks the duct and both entries. Connector frames identify the
entry and exit locations. The ports do not assume a through-bore in the existing
gearboxes. External joint loops, glands, strain relief and the tool harness are
not yet modeled. The hollow housings assume cored/assembled shell construction;
casting tooling, manufacturing splits, fits and gasket compression remain to be
specified. The gasket geometry establishes assembly space, not an ingress rating.

The nominal wrist-elbow shell diameters and checked arm masses (including wrist
flanges and fasteners, excluding the tool and offboard drives) are:

| Class | Shell diameter | Arm mass |
| --- | --- | --- |
| Reach500 | 63 mm | 14.472 kg |
| Reach850 | 80.64 mm | 22.130 kg |
| Reach900 | 92.4 mm | 36.343 kg |
| Reach1300 | 92.4 mm | 36.790 kg |

Mass includes the shoulder/arm output flanges, service lids, gaskets and all
modeled mounting screws. Different housing/cavity distributions mean some
classes become lighter while another becomes heavier; these are CAD-derived
assumed masses, not manufacturer measurements.

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

The long-link checks also require one valid connected solid, containment within
the collision envelope, a continuous duct, open cable entries, an unobstructed
service window and mounting contact at both drive interfaces and across each
gasket/lid stack. Material probes also check the outer wall through each hub's
90-degree rounding: overlapping hollow cylinders can form a connected solid
while leaving that quadrant open, so both outer and inner spherical transitions
are modeled at all four route bends. Fixed-connected member pairs are checked once per class;
all pairs across moving joints are checked at every sampled pose. No collision
pair is exempted merely because it is adjacent or is a fastener.

Use `--class=Reach850` to run one size. Pass `--export --class=Reach850` to write
`build/reach850.scene` and posed `build/reach850.mesh.json` for review. The scene
includes the preview motion loop; the mesh export omits offboard drives and
internal gearheads for a clear exterior view. Generated artifacts stay out of
project history. The current [whole-arm comparison](build/arm-before-after.png)
shows the old and refined models at the same pose and scale. The
[hub-corner comparison](build/corner-before-after.png) shows the missing outer
rounding corrected at the tube-to-joint junction.

The additional hub rounding changes the tending collision envelope. Its previous
reach certificate cannot be reused: the saved `infeed-1-2-approach` pose for
gripper A collides with the current cell. Regenerate the certificate and run its
independent check before using these housings in the tending cycle.

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

The custom cobot's optional collar socket is a provisional service module, not an
OEM UR/FAIRINO interface. The imported commercial robots now have separate
[source-backed wrist electrical profiles](../reference-arms/README.md#manufacturer-wrist-electrical-interfaces),
including revision-specific pinouts and documented power limits. Their socket
mating positions/key orientations remain explicitly unqualified.

Long-link collision geometry follows the housing construction: 17 convex pieces
per link, including cover screw bosses, with 0.03 mm tessellation and a 0.05 mm
outward support reserve. The long oblique taper is enclosed in its own
longitudinal frame before transforming its hull back to the link frame; a
world-axis envelope can otherwise add tens of millimetres of false volume.
Internal bores remain conservatively filled. The
machine-tending arm audit independently checks coverage of the finer 0.01 mm
surface triangles and compares previously flagged poses against exact CAD.
