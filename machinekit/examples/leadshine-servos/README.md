# Leadshine ELM1 / ELM2 servos

Open `materia.project.json` in Materia to view small ELM1 motors and larger ELM2
motors up to 7.5 kW, each with a row of brake and non-brake variants. The small
ELM1/ELM2 frames share dimensions, with magnetic and optical feedback respectively.

```haxe
var motor = machinekit.motion.LeadshineServo.model("ELM1H-0400MA60F");
var braked = machinekit.motion.ServoMotor.model("ELM2H-0750LA80E");
var powerful = machinekit.motion.LeadshineServo.model("ELM2M-7500LD180ET-H");
var housing = motor.geometry();
var shaft = motor.shaft(); // separate keyed rotating part
var plateTool = motor.mountingCutout(10);
```

The catalog contains 54 variants, with F without brake or E with brake:

| Family | Power | Supply | Drawing face LC |
| --- | --- | --- | --- |
| ELM1H MA / ELM2H LA | 50–1000 W | 220 VAC | 40, 60, 80 mm |
| ELM2H LD130 -HD | 850, 1300, 1800 W | 220 VAC | 130 mm |
| ELM2M LB130 -H | 1000, 1500, 2000 W | 220 VAC | 131 mm |
| ELM2L LA100 T-H | 1000, 1500 W | 380 VAC | 100 mm |
| ELM2H LD130 T-H | 850, 1300, 1800 W | 380 VAC | 130 mm |
| ELM2M LD180 T-H | 2900, 4400, 5500, 7500 W | 380 VAC | 180 mm |

Voltage, rated/max rpm and key height are explicit per model. The 220 VAC 1.8 kW
high-inertia row has a 2000 rpm maximum; its 380 VAC counterpart reaches 3000 rpm.
The 7.5 kW row has 48 Nm rated torque and 119 Nm peak, a 42 mm shaft and a
10–12 mm key width across the 180 mm family (key height is 8 mm).
The catalog does not include the 380 VAC 80 mm winding variants or models absent
from the cited datasheet.

`ServoMotor.ratings()` includes the new designations, and both factories retain
the vendor recipe on serialization. Saved generic motors remain generic.

Source: [Leadshine AC servo datasheet, pages 3–16](https://www.leadshine.com/upfiles/downloads/006a40c553d0c6fc60a1221d6a764222_1675822261873.pdf).
The model follows LC, LL, LR, LA, LZ, LG, LB, LE and the keyway LK/KH/KW/T dimensions.
LA is a bolt-circle diameter; hole coordinates are ±LA/(2√2). The mounting face
is z=0, body extends toward -Z, pilot projects by LE and shaft extends toward +Z.
Connectors are `mountFace`, `shaftAxis`, `shaftEnd`, `pilot`, and `bolt1`–`bolt4`.
`mountingCutout` uses the drawing hole diameters and configurable diametral
pilot clearance. Shaft keyways have square ends and estimated axial placement;
shaft-end tapped holes and
manufacturing tolerances are not modeled. Keys are not installed in the gallery.

Housing ribs, cover proportions and connector positions/shapes are approximations.
Connectors reach the drawing's maximum LH. `Envelope` encloses the preview and
fills bolt holes. Neither housing representation includes the separate shaft.
Brake variants use their published longer bodies, mass and rotor inertia; they
do not automatically install a simulated brake or control wiring.

Torque, speed, current, rotor inertia and encoder resolution use catalog values.
Torque-speed interpolation remains explicitly assumed. `electricalRating()`
returns null because verified winding constants are unavailable; the inherited
`rating.electrical` compatibility field contains an estimate, not manufacturer data.
Published total mass is split between the housing and exposed shaft to avoid
double counting. The total centre is estimated at the body midpoint; housing
inertia is unknown. The ELM2 50 W braked mass is truncated in the source;
its 0.44 kg value uses the mechanically identical ELM1 row.

Run checks or export the catalog as separate housing and shaft STEP files:

```sh
./haxeon/scripts/haxeon run --project machinekit/tests/haxeon.leadshine.json
LEADSHINE_STEP_DIR=/tmp/leadshine-step ./haxeon/scripts/haxeon run --project machinekit/examples/leadshine-servos/haxeon.json
```
