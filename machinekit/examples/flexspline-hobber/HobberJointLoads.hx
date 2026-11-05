import machinekit.assembly.JointSlip.JointLoad;
import machinekit.assembly.JointSlip.LoadCase;
import machinekit.motion.BallNut;
import machinekit.motion.ErobActuatorData;
import machinekit.motion.ErobActuatorPart;
import machinekit.motion.ServoMotor;

/**
 * The loads the hobber's bolted joints carry, declared for the slip check of `JointFasteners` (BOM_PLAN F1, `fasteners.slip`). Every figure comes from the
 * machine's own parts and ratings; where a figure is assumed it says so. Joints not listed carry no load the machine knows: the catalogue rails and blocks
 * (their rated loads are the maker's, `HobberFrameChecks.zOffsetMoments`), nut flanges (axial thrust is tension in their screws, not slip), motor flanges
 * (the belt pull is checked in `HobberFrameChecks.spindleDrive`) and the casting's and enclosure's mounts (static weight).
 */
class HobberJointLoads {
  /** Ball screw lead of X, Y and Z, m (the drive sizing's). */
  public static inline final LEAD = 0.005;
  /** The largest hob the spindle takes (the reference hob), radius m, for a jam. */
  public static inline final HOB_RADIUS = 0.016;
  /** The cutting force bound the spindle drive is sized for (`HobberFrameChecks.spindleDrive`), N: ASSUMED upper bound. */
  public static inline final CUTTING_FORCE = 100.0;

  /** The member of `machine` with id `id`. */
  static function member(machine:FlexsplineHobber, id:String):Dynamic {
    for (entry in machine.components()) if (entry.id == id) return entry.component;
    throw 'The hobber has no member "$id"';
  }

  /** Mass of the members whose id is `id` or starts with `id` + "/", kg. */
  static function massOf(machine:FlexsplineHobber, ids:Array<String>):Float {
    var total = 0.0;
    for (entry in machine.components()) for (id in ids) if (entry.id == id || StringTools.startsWith(entry.id, id + "/")) total += entry.component.massProperties().mass;
    return total;
  }

  /** Mass of everything that rides on the guide block `first`: its members and all their descendants in the joint graph, kg. */
  static function movingMass(machine:FlexsplineHobber, first:String):Float {
    var moving = new Map<String, Bool>();
    moving.set(first, true);
    var changed = true;
    while (changed) {
      changed = false;
      for (j in machine.definition().joints) if (moving.exists(j.parent) && !moving.exists(j.child)) {
        moving.set(j.child, true);
        changed = true;
      }
    }
    var total = 0.0;
    for (entry in machine.components()) if (moving.exists(entry.id)) total += entry.component.massProperties().mass;
    return total;
  }

  public static function of(machine:FlexsplineHobber):Array<JointLoad> {
    var loads:Array<JointLoad> = [], g = 9.80665;
    // The A axis: the eRob's output torque passes the output flange, the adapter ring and the swivel plate, and its reaction the housing and the Z saddle.
    // Design peak and momentary (collision or emergency stop) torque are the maker's for the ratio; the flange M4 are tightened to the maker-table torque
    // of 12.9 M4 (4.95 N m, 6790 N, `FastenerTightening`: 80 % of proof) because 70 % leaves only 1.12 x on the momentary torque.
    var row = ErobActuatorData.row(machine.swivel.housing.ratio);
    var torque:LoadCase = {peak: row.peakTorque, momentary: row.momentaryTorque};
    loads.push({label: "A axis: housing reaction on the Z saddle", between: ["Zplate", "swivel/housing"], torque: torque});
    loads.push({label: "A axis: output flange to the adapter ring", between: ["swivelAdapter", "swivel/output"], torque: torque, utilisation: 0.8});
    loads.push({label: "A axis: adapter ring to the swivel plate", between: ["swivelPlate", "swivelAdapter"], torque: torque});
    // The drives' thrust. The design peak is the drive sizing's duty (`HobberFrameChecks.sizing`: 0.5 m/s2, 100 N process and 100 N friction, gravity on Z and
    // along the tilted Y, the rotor's inertia and drag) on the axis's moving mass: the sizing's peak torque through the nut's efficiency and the lead. The
    // momentary load of the fixed supports is the motor's peak torque the same way (a jam against the stops). It passes the fixed support's feet into their
    // pads and brackets, and the nut block's base into its carriage (the nut's flange is perpendicular to the screw: tension, not slip); a jam is sized into the
    // supports' dowels but not into the nut blocks' two screws (open: JointSlip reports x0.95 for X and Z, less for Y).
    for (axis in [
      {id: "X", first: "blockXLeft", pad: "XdrivePad", foot: "Xfixed", carriage: "Xbridge", block: "XnutBlock", seat: null},
      {id: "Y", first: "blockYLeft", pad: "YdriveFixed", foot: "Yfixed", carriage: "Yplate", block: "YnutBlock", seat: "YnutSeat"},
      {id: "Z", first: "blockZLeft", pad: "ZdrivePad", foot: "Zfixed", carriage: "Zplate", block: "ZnutBlock", seat: null}
    ]) {
      var motor:ServoMotor = member(machine, axis.id + "motor"), nut:BallNut = member(machine, axis.id + "nut");
      var q = LEAD / (2 * Math.PI), moving = movingMass(machine, axis.first);
      var duty = machinekit.sizing.LinearDriveSizing.demand({
        massKg: moving, motors: 1, metresPerRadian: q, efficiency: nut.efficiency(), rotaryInertiaKgM2: 0.4e-4, acceleration: 0.5, velocity: 0.2,
        strokeM: axis.id == "Y" ? 0.03 : axis.id == "X" ? 0.095 : 0.2, cuttingForceN: CUTTING_FORCE, dragForceN: 100,
        gravityForceN: axis.id == "Z" ? moving * g : axis.id == "Y" ? moving * g * Math.sin(Math.PI / 12) : 0, rotaryDragNm: 0.1, reserve: 1.5
      }, motor.rating);
      var perNm = nut.efficiency() / q, design = duty.peakNm * perNm;
      loads.push({
        label: axis.id + " drive: screw thrust on the fixed support",
        between: [axis.pad, axis.foot],
        shear: {peak: design, momentary: motor.rating.peakTorque * perNm}
      });
      var base = axis.seat == null ? axis.carriage : axis.seat;
      loads.push({label: axis.id + " drive: screw thrust through the nut block", between: [base, axis.block], shear: {peak: design}});
      if (axis.seat != null) loads.push({label: axis.id + " drive: screw thrust from the nut seat into the saddle", between: [axis.seat, axis.carriage], shear: {peak: design}});
    }
    // The spindle seat on the Y saddle's plate carries the weight of the seat, cradle and spindle (with the Y axis's 0.5 m/s2) and the cutting force; a jammed
    // hob loads it with the tangential force the servo's peak torque makes at the hob's radius.
    var mass = massOf(machine, ["spindleSeat", "motorCradle", "spindle"]);
    var jam = machine.spindle.jointTorque(machine.spindle.motor.rating.peakTorque) / HOB_RADIUS;
    var weight = mass * (g + 0.5);
    loads.push({label: "Spindle seat on the Y saddle plate", between: ["Yplate", "spindleSeat"], shear: {peak: weight + CUTTING_FORCE, momentary: weight + jam}});
    // The cradle on the seat carries the belt's pull on the servo at its peak torque (twice the pretension, `HobberFrameChecks.spindleDrive`) and the servo group's weight.
    var pull = 2 * machinekit.hobbing.HobSpindle.pretension(machine.spindle.motor.rating.peakTorque, machine.spindle.motorPulley.pitchDiameter);
    loads.push({
      label: "Motor cradle on the spindle seat",
      between: ["motorCradle", "spindleSeat"],
      shear: {peak: pull + massOf(machine, ["motorCradle", "spindle/motor"]) * g}
    });
    return loads;
  }
}
