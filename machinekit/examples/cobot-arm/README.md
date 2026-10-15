# Cobot arm

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
