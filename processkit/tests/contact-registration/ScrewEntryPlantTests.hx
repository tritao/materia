import motionkit.robot.CartesianAdmittance;
import processkit.screwdriving.ScrewEntryController;
import robotkit.spatial.Vec3;
import robotkit.spatial.Wrench3;

/** A hole in a plate, in hole-plane coordinates (metres); `depth` below the plate top is its seat, or none. */
private typedef PlantHole = {u:Float, v:Float, radius:Float, chamfer:Float, ?depth:Float};
private typedef PlantPlate = {top:Float, thickness:Float, holes:Array<PlantHole>};
private typedef PlantRun = {phase:ScrewEntryPhase, failure:Null<String>, peakForceN:Float, seconds:Float,
  searches:Int, depth:Float, u:Float, v:Float, contactDepth:Null<Float>, peakAt:String};

/**
 * Screw entry against a small plant, fast enough to iterate on: the screw tip is a mass on a stiff position servo
 * that follows the commanded twist, the force/torque sample arrives one control period late, and the work is a stack
 * of plates with chamfered holes, penalty contact and friction. The hole plane is turned against the root frame so
 * the controller's axis handling is exercised.
 */
class ScrewEntryPlantTests {
  static var checks = 0;
  static inline final CONTROL_SECONDS = 0.01;
  static inline final SUBSTEPS = 20;
  static inline final MAXIMUM_FORCE_N = 25.0;
  static final axisU:Vec3 = new Vec3(0, 1, 0);
  static final axisV:Vec3 = new Vec3(1, 0, 0);
  // u x v: the advance axis points down, into the work.
  static final axisN:Vec3 = axisU.cross(axisV);

  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }

  public static function run():Void {
    var springs:Array<Null<Float>> = [null, 2000.0];
    for (spring in springs) scenarios(spring);
    rejections();
  }

  /** Every scenario for one tool: rigid along the axis (`spring` null) or on a feed spring of `spring` N/m. */
  static function scenarios(spring:Null<Float>):Void {
    check(axisN.z == -1, "the test's hole plane advances downwards");
    // A flange housing over a tapped plate: an M6 clearance bore with 0.3 mm radial play and a chamfer, then the
    // tapped hole whose lead-in lets the screw tip sink 0.5 mm before the thread stops it.
    function cell(offsetU:Float, offsetV:Float, tapped:Null<PlantHole>, ?flangeBore:PlantHole):Array<PlantPlate> {
      var bore = flangeBore != null ? flangeBore : {u: offsetU, v: offsetV, radius: 0.0003, chamfer: 0.0004};
      var plate:PlantPlate = {top: 0.040, thickness: 0.016, holes: tapped == null ? [] : [tapped]};
      return [{top: 0.023, thickness: 0.017, holes: [bore]}, plate];
    }
    function tappedAt(u:Float, v:Float):PlantHole return {u: u, v: v, radius: 0.0003, chamfer: 0.0004, depth: 0.0005};

    var nominal = simulate(cell(0, 0, tappedAt(0, 0)), spring);
    check(nominal.phase == Seated, 'nominal screw seats in the tapped hole: ${nominal.failure} (${nominal.searches} searches, ' +
      'last contact at ${nominal.contactDepth}, peak ${nominal.peakForceN} N, at ${nominal.u}, ${nominal.v})');
    check(nominal.searches == 0, 'nominal screw passes the clearance bore without searching (${nominal.searches})');
    check(nominal.peakForceN < MAXIMUM_FORCE_N, 'nominal seating stays below the force bound: ${nominal.peakForceN} N');
    check(nominal.seconds < 30, 'nominal entry takes seconds, not minutes: ${nominal.seconds} s');

    var tool = spring == null ? 'stiff tool' : 'feed spring';
    var summary = ['$tool: nominal ${round(nominal.seconds)} s ${round(nominal.peakForceN)} N'];
    for (offset in [[0.001, 0.0], [-0.0007, 0.0009], [0.0, -0.0013]]) {
      var shifted = simulate(cell(offset[0], offset[1], tappedAt(offset[0], offset[1])), spring);
      summary.push('offset ${round(Math.sqrt(offset[0] * offset[0] + offset[1] * offset[1]) * 1000)} mm ' +
        '${round(shifted.seconds)} s ${round(shifted.peakForceN)} N (${shifted.peakAt})');
      check(shifted.phase == Seated, 'a ${offset} m registration error is found on the flange face: ${shifted.failure}');
      check(shifted.searches >= 1, "an offset bore is found by searching the face the screw lands on");
      check(Math.abs(shifted.u - offset[0]) < 0.0004 && Math.abs(shifted.v - offset[1]) < 0.0004,
        'the seated screw is on the hole axis: ${shifted.u}, ${shifted.v} for ${offset}');
      check(shifted.peakForceN < MAXIMUM_FORCE_N, 'searching the flange face stays below the force bound: ${shifted.peakForceN} N');
      check(shifted.seconds < 120, 'an offset entry takes well under two minutes: ${shifted.seconds} s');
    }

    var noTappedHole = simulate(cell(0, 0, null), spring);
    check(noTappedHole.phase == Failed && noTappedHole.peakForceN < MAXIMUM_FORCE_N,
      'a plate without its tapped hole fails the search inside the bore: ${noTappedHole.failure}, ' +
      'peak ${noTappedHole.peakForceN} N (${noTappedHole.peakAt})');
    var noBore = simulate([{top: 0.023, thickness: 0.017, holes: []}], spring);
    check(noBore.phase == Failed && noBore.failure.indexOf("exhausted") >= 0 && noBore.peakForceN < MAXIMUM_FORCE_N,
      'a flange without its bore exhausts the spiral: ${noBore.failure}');
    var blocked = simulate(cell(0, 0, tappedAt(0, 0), {u: 0, v: 0, radius: 0.0003, chamfer: 0.0004, depth: 0.002}), spring);
    check(blocked.phase == Failed && blocked.depth < 0.030 && blocked.peakForceN < MAXIMUM_FORCE_N,
      'debris in the bore stops the screw above the CAD plane and fails: ${blocked.failure} at ${blocked.depth}');
    // A tapped hole modelled as a plain bore (as the CAD cell's MuJoCo geometry is) still hands over at the plane:
    // whether a thread is there to engage is the tightening cycle's to measure.
    var plainBore = simulate(cell(0, 0, {u: 0, v: 0, radius: 0.0003, chamfer: 0.0004, depth: null}), spring);
    check(plainBore.phase == Seated && plainBore.searches == 0,
      'a bore without a modelled thread hands over at the CAD plane: ${plainBore.failure}');
    Sys.println('Screw entry plant, ${summary.join(", ")}');
  }

  static function rejections():Void {
    var limits = limitsFor();
    var rejected = false;
    try new ScrewEntryController(limits, axisU, axisV, 0.04, limits.entryAdvance, admittance(limits, 4000.0))
      catch (_:Dynamic) rejected = true;
    check(rejected, "a depth tolerance as large as the entry advance would accept a flat surface as a hole");
    Sys.println('Screw entry plant: $checks checks passed');
  }

  static function round(value:Float):Float return Math.round(value * 100) / 100;

  static function limitsFor():ScrewEntryLimits
    return new ScrewEntryLimits(5.0, 2.0, 2, 0.002, 0.004, 0.0016, 0.0004, 0.002, 10.0, 0.00015, 0.0003, 5.0);

  /**
   * Search-axis admittance with a 20 ms relaxation. Each axis is damped above the stiffness it pushes against times
   * the loop latency: the rigid contact sideways, and along the axis the contact or, with a feed spring, the spring.
   */
  static function admittance(limits:ScrewEntryLimits, axialDamping:Float):CartesianAdmittance {
    var lateral = 4000.0;
    return new CartesianAdmittance(new Wrench3(new Vec3(0, 0, -limits.contactForceN), Vec3.zero()),
      [lateral * 0.02, lateral * 0.02, axialDamping * 0.02, 1, 1, 1], [lateral, lateral, axialDamping, 20, 20, 20],
      limits.maximumLinearSpeed, 0.01, CONTROL_SECONDS);
  }

  /**
   * One entry. `feedSpring` (N/m) is the tool's compliant feed between the servoed arm and the tip along the axis;
   * without it the tip is held as stiffly along the axis as sideways.
   */
  static function simulate(plates:Array<PlantPlate>, ?feedSpring:Float):PlantRun {
    var limits = limitsFor();
    var controller = new ScrewEntryController(limits, axisU, axisV, 0.040, 0.00002,
      admittance(limits, feedSpring == null ? 4000.0 : 400.0));
    var servoStiffness = 2.0e5, contactStiffness = 2.0e5, tipMass = 0.5, friction = 0.3;
    var axialStiffness = feedSpring == null ? servoStiffness : (feedSpring : Float);
    var stiffness = [servoStiffness, servoStiffness, axialStiffness];
    var damping = [for (k in stiffness) 2.0 * Math.sqrt(k * tipMass)];
    // Hole-plane coordinates of the commanded (servo) point and the actual tip; the approach origin is zero.
    var commanded = [0.0, 0.0, 0.0], tip = [0.0, 0.0, 0.0], velocity = [0.0, 0.0, 0.0];
    var command = [0.0, 0.0, 0.0];
    var delayed = Wrench3.zero(), peak = 0.0, time = 0.0, peakAt = '';
    while (!controller.finished() && time < 600) {
      var twist = controller.step(axisU.scale(tip[0]).add(axisV.scale(tip[1])).add(axisN.scale(tip[2])), delayed,
        CONTROL_SECONDS);
      var linear = new Vec3(twist.linearX, twist.linearY, twist.linearZ);
      command = [linear.dot(axisU), linear.dot(axisV), linear.dot(axisN)];
      var force = [0.0, 0.0, 0.0];
      var h = CONTROL_SECONDS / SUBSTEPS;
      for (_ in 0...SUBSTEPS) {
        for (i in 0...3) commanded[i] += command[i] * h;
        force = contact(plates, tip, velocity, contactStiffness, friction);
        var magnitude = Math.sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]);
        if (magnitude > peak) {
          peak = magnitude;
          peakAt = '${controller.phase} at ${round(time)} s, depth ${round(tip[2] * 1000)} mm, force ${[for (f in force) round(f)]}';
        }
        for (i in 0...3) {
          var acceleration = (stiffness[i] * (commanded[i] - tip[i]) + damping[i] * (command[i] - velocity[i]) +
            force[i]) / tipMass;
          velocity[i] += acceleration * h;
          tip[i] += velocity[i] * h;
        }
      }
      // The sensor's sample is the wrench at the end of this period, read at the next step.
      delayed = new Wrench3(axisU.scale(force[0]).add(axisV.scale(force[1])).add(axisN.scale(force[2])), Vec3.zero());
      time += CONTROL_SECONDS;
    }
    return {phase: controller.phase, failure: controller.failure, peakForceN: peak, seconds: time,
      searches: controller.searches, depth: tip[2], u: tip[0], v: tip[1],
      contactDepth: controller.contactDepth, peakAt: peakAt};
  }

  /**
   * Environment force on the tip (u, v, n components). Plates are visited top down; the tip touches the first one it
   * is not passing through a hole of, and the shank stays in the bores of the plates it has passed. Below a face the
   * penetration is the smaller of the vertical one and the lateral one into the nearest bore wall, as a collision
   * would resolve it.
   */
  static function contact(plates:Array<PlantPlate>, tip:Array<Float>, velocity:Array<Float>, stiffness:Float,
      friction:Float):Array<Float> {
    var u = tip[0], v = tip[1], depth = tip[2];
    var force = [0.0, 0.0, 0.0];
    for (plate in plates) {
      if (depth <= plate.top) break;
      var hole:Null<PlantHole> = null, radial = Math.POSITIVE_INFINITY;
      for (candidate in plate.holes) {
        var distance = Math.sqrt((u - candidate.u) * (u - candidate.u) + (v - candidate.v) * (v - candidate.v));
        if (distance < radial) { radial = distance; hole = candidate; }
      }
      if (depth > plate.top + plate.thickness) {
        // Passed through: the shank is in this plate's bore.
        if (hole != null && radial > hole.radius) {
          force[0] -= stiffness * (radial - hole.radius) * (u - hole.u) / radial;
          force[1] -= stiffness * (radial - hole.radius) * (v - hole.v) / radial;
        }
        continue;
      }
      if (hole != null && radial < hole.radius) {
        var seat = hole.depth;
        if (seat != null && depth > plate.top + seat) {
          var penetration = depth - plate.top - seat;
          force[2] -= stiffness * penetration;
          addFriction(force, velocity, stiffness * penetration, friction);
        }
        return force;
      }
      var vertical = depth - plate.top;
      if (hole == null) {
        if (vertical > 0) {
          force[2] -= stiffness * vertical;
          addFriction(force, velocity, stiffness * vertical, friction);
        }
        return force;
      }
      var near:PlantHole = hole;
      // A 45 degree chamfer lowers the face towards the hole and pushes the tip towards its axis.
      var chamfered = radial < near.radius + near.chamfer;
      if (chamfered) vertical = depth - (plate.top + near.radius + near.chamfer - radial);
      var lateral = radial - near.radius;
      if (lateral < vertical) {
        // Inside the bore, pressed against its wall; a blind bore's bottom still carries it.
        force[0] -= stiffness * lateral * (u - near.u) / radial;
        force[1] -= stiffness * lateral * (v - near.v) / radial;
        var seat = near.depth;
        if (seat != null && depth > plate.top + seat) {
          force[2] -= stiffness * (depth - plate.top - seat);
          addFriction(force, velocity, stiffness * (depth - plate.top - seat), friction);
        }
      } else if (vertical > 0) {
        var normal = chamfered ? 0.5 * stiffness * vertical : stiffness * vertical;
        force[2] -= normal;
        if (chamfered) {
          force[0] -= normal * (u - near.u) / radial;
          force[1] -= normal * (v - near.v) / radial;
        }
        addFriction(force, velocity, normal, friction);
      }
      return force;
    }
    return force;
  }

  static function addFriction(force:Array<Float>, velocity:Array<Float>, normal:Float, friction:Float):Void {
    var speed = Math.sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
    var scale = friction * normal / Math.max(speed, 0.0005);
    force[0] -= scale * velocity[0];
    force[1] -= scale * velocity[1];
  }
}
