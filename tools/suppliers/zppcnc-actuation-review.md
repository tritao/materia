# AR155-I actuation evidence

Reviewed 2026-10-20. Status: **hardware integrated; dynamic actuation blocked on
supplier operating data**. This review does not qualify workholding safety.

## Sources and confidence

The [product specification](https://zppcncvise.com/products/zpp-125mm-self-centering-vise-zero-point-workholding)
and the pinned 2025/2026 catalogs specify 8–10 kN at 6–7 bar and a
130 × 145 mm clamping envelope. Neither establishes automatic jaw stroke,
port functions, valve schematic, or a force-versus-travel law. The published
145 mm capacity must not become the pneumatic stroke.

The [supplier article](https://zppcncvise.com/blogs/news/ar155-pneumatic-self-centering-vise)
claims approximately 1.5 seconds per cycle and spring closing with air opening.
These are marketing statements, without an operating schematic or test conditions.
They are leads for supplier confirmation, not parameters for the runtime. In
particular, a pressure switch on an air-opening chamber cannot establish grip.
Loss of electrical power, loss of upstream air, and trapped downstream pressure
must be specified separately.

The [download page](https://zppcncvise.com/pages/service) supplies catalogs; the
product page supplies the already pinned STEP. Public English and Chinese searches
for AR155 stroke/operating instructions did not locate an operating manual.
The STEP resolves external bore locations but not port function or internal motion.

## Runtime implications

`machinekit/assembly/DriveSystem.hx` currently compiles pneumatic cylinders with
two A/B hoses, piston and rod areas, and a stroke matching the driven slide.
`CylinderSpec` and `CylinderControlSpec.PressureEffort` describe that cylinder
model. Binding the AR155 to an ISO cylinder or a 5/2 valve would assume both an
internal mechanism and an air circuit that have not been established.

If spring-close/air-open operation is confirmed, implement a reusable
single-acting spring-return actuator/control model first. Use confirmed mechanism
or measured jaw force/travel data; do not infer a piston bore from rated jaw force.
Preserve the two coupled jaws and their existing geometric seating datums.

Clamp confirmation must combine workpiece presence, the valid jaw-position window,
and a confirmed lock/grip signal appropriate to the actual mechanism. Supply
pressure is a separate permissive, not proof of clamping. Sensor hysteresis,
timeouts, and thresholds require operating data or explicitly identified simulation
settings. A release request during cutting must stop machining before actuation.
Missing confirmation must inhibit start; lost confirmation must stop execution and
require an explicit reset. No automatic restart after pressure recovery.

The current 2 mm per-jaw release is a packaging assumption. Existing CNC checks
verify the preset jaw position only; they are not pneumatic confirmation.

## Supplier technical request (draft, not sent)

Subject: AR155-I operating manual and pneumatic integration data

We are integrating an AR155-I single-station vise on a manual TS96-155 plate.
Please provide the operating/maintenance manual and pneumatic schematic for the
current AR155-I revision, including:

1. Automatic stroke per jaw and total opening change, distinct from coarse jaw
   adjustment; permitted preset opening and workpiece engagement.
2. Port identification, threads, required fittings, exhaust requirements, and
   whether each visible bore is an operating port or an alternate connection.
3. Whether it is spring-close/air-open or double acting; required valve circuit
   and default valve state. Please distinguish electrical power loss, upstream air
   loss, and residual pressure behavior.
4. Jaw force versus pressure and travel, including spring/lock retention force
   at zero supply pressure, and whether rated force is per jaw or total.
5. Required air quality, operating pressure limits, flow/consumption, and opening
   and closing times with their measurement conditions.
6. Available clamped/released, workpiece-presence, and mechanical-lock feedback;
   sensor mounting, signal definitions, and recommended CNC confirmation logic.
7. Permitted workpiece lift/overhang, grip depth, cutting loads, and any limits on
   unattended operation or air-loss retention.

The downloaded STEP is `AR155_I.stp`; please confirm its revision matches the
provided instructions. The 130 × 145 mm capacity alone does not identify the
automatic stroke needed for our simulation.
