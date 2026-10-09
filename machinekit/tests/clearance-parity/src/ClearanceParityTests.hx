import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import collisionkit.native.NativeCollisionWorld;
import machinekit.assembly.AssemblyPreview;
import machinekit.gantry.LinearTrack;
import materia.project.SceneArtifact;
import motionkit.planner.JointPathSamples;
import motionkit.robot.JointCurveClearance;
import motionkit.robot.ManipulatorKinematics;
import processkit.WeldPathPlanner;
import robotkit.collision.CollisionClearance;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.ArmClearance.ClearanceBodyData;
import robotkit.manipulation.ClearanceViolation;
import robotkit.manipulation.ClearanceWorld;
import robotkit.manipulation.Manipulator;

/**
 * CL4a parity (COLLISION.md CL-D12): `ArmClearance` and the collisionkit
 * adapter `CollisionClearance` on the same scenes (main's weld cell, the
 * rail and the G17 track welder) agree on violations, closest pairs and
 * sweeps within the solver tolerance; `JointCurveClearance` certifies the
 * same paths with either; and the weld planner plans the same weld.
 *
 * A disagreement is accepted only at the boundary: when the distance of the
 * pair in question is within `TOLERANCE` of the clearance it needs, where two
 * correct distance solvers may round to different sides. Every one is
 * counted and printed.
 */
@:access(robotkit.manipulation.ArmClearance)
@:access(WeldPlanningTests)
class ClearanceParityTests {
  static inline var TOLERANCE = 1e-6;
  static var assertions = 0;

  public static function main():Void {
    weldCell();
    rail();
    trackWelder();
    Sys.println('Clearance parity tests passed ($assertions assertions)');
  }

  static function adapter(clearance:ArmClearance):CollisionClearance {
    var data:Array<ClearanceBodyData> = [for (body in clearance.bodies)
      {name: body.name, link: body.link, vertices: body.corners.copy(), tool: body.tool}];
    return new CollisionClearance(clearance.arm, data, clearance.reference, () -> new NativeCollisionWorld(),
      clearance.margin, clearance.contactMargin);
  }

  static function weldCell():Void {
    var walls:Array<Null<Float>> = [null, 0.48];
    for (wall in walls) {
      var cell = WeldPlanningTests.cell(wall);
      var hull = cell.clearance;
      var native = adapter(hull);
      var rng = new Rng(wall == null ? 3 : 4);
      var samples = [cell.start].concat([for (_ in 0...300) [for (j in 0...6) cell.start[j] + rng.signed() * 1.6]]);
      compare(wall == null ? "weld cell" : "weld cell with a wall", hull, native, samples, rng);
      // The weld planner plans the same weld with either world.
      var solver = new ManipulatorKinematics(cast hull.arm, 1e-8);
      solver.preferTargetOrientation = true;
      var planner = new WeldPathPlanner(solver, new motionkit.kinematics.IkTolerance(2e-4, 1e-3, 300, 0.03),
        [for (_ in 0...6) 3.0], WeldPlanningTests.WRIST, native);
      var withHulls = cell.planner.plan(WeldPlanningTests.seam(), cell.start);
      var withCollisionKit = planner.plan(WeldPlanningTests.seam(), cell.start);
      check(withHulls.entry.name == withCollisionKit.entry.name && withHulls.rolls.join(",") == withCollisionKit.rolls.join(",")
        && withHulls.checked == withCollisionKit.checked,
        'the weld planner plans the same weld (${withHulls.entry.name} ${withHulls.rolls} ${withHulls.checked} / '
        + '${withCollisionKit.entry.name} ${withCollisionKit.rolls} ${withCollisionKit.checked})');
      Sys.println('CL4A_PARITY weld planner ${wall == null ? "open" : "wall"}: entry "${withHulls.entry.name}", rolls ${withHulls.rolls}, ${withHulls.checked} poses, same with either world');
    }
  }

  /** The 3 m linear track carrying the robot arm (main's rail scene, LinearTrackCheck). */
  static function rail():Void {
    var track = new LinearTrack(3000, 300);
    track.includeArm("arm", new RobotArm(false));
    var scene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(track, "linear-track")));
    scenery("rail", AssemblySimulationBridge.toRobotModel(cast scene.assemblyDefinition,
      AssemblyPhysicalPartView.fromSceneArtifact(scene), scene.assemblyState), 0.003, 5);
  }

  /** The G17 track welder: the arm on the track, its torch, the 2.6 m weldment and the table. */
  static function trackWelder():Void {
    var scene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(new TrackWelder(), "track-welder")));
    scenery("G17 track welder", AssemblySimulationBridge.toRobotModel(cast scene.assemblyDefinition,
      AssemblyPhysicalPartView.fromSceneArtifact(scene), scene.assemblyState), processkit.WeldPathPlanner.AIR_MARGIN, 6);
  }

  static function scenery(name:String, converted:cadbridge.AssemblySimulationBridge.AssemblySimulationModel, margin:Float,
      seed:Int):Void {
    var model = converted.model;
    var toolFrames = [for (frame in model.frames) if (frame.name == "arm/toolFlange robot flange") frame];
    check(toolFrames.length == 1, '$name has its tool flange');
    var manipulator = new Manipulator(model, model.links[0].id, toolFrames[0].id);
    var count = manipulator.dofCount();
    var reference = [for (_ in 0...count) 0.0];
    var bodies:Array<ClearanceBodyData> = [for (hull in converted.linkHulls)
      {name: hull.part, link: model.links[hull.link].id, vertices: hull.vertices, tool: StringTools.startsWith(hull.part, "arm/tool/")}];
    var hull = new ArmClearance(manipulator, bodies, reference, margin);
    var native = new CollisionClearance(manipulator, bodies, reference, () -> new NativeCollisionWorld(), margin);
    var rng = new Rng(seed);
    var samples = [reference];
    for (_ in 0...200) samples.push([for (j in 0...count) {
      var limits = manipulator.group.limitsOf(j);
      var lower = Math.max(limits.lower, -2.5), upper = Math.min(limits.upper, 2.5);
      j == 0 && manipulator.external[0] ? lower + (upper - lower) * (0.5 + 0.5 * rng.signed()) : 0.6 * rng.signed() * Math.min(upper, -lower);
    }]);
    compare(name, hull, native, samples, rng);
  }

  /**
   * Violations (plain, contact, inflated by displacement bounds), closest
   * pairs and sweeps at `samples`; and a `JointCurveClearance` proof of
   * straight paths between consecutive samples with either world.
   */
  static function compare(name:String, hull:ArmClearance, native:CollisionClearance, samples:Array<Array<Float>>, rng:Rng):Void {
    check(hull.pairCount() == native.pairCount() && hull.bodyCount() == native.bodyCount(),
      '$name: the same bodies and checked pairs (${hull.pairCount()} / ${native.pairCount()})');
    var queries = 0, violations = 0, boundary = 0, worstClosest = 0.0;
    function agree(what:String, a:Null<ClearanceViolation>, b:Null<ClearanceViolation>):Void {
      queries++;
      if (a != null) violations++;
      if (a == null && b == null) return;
      if (a != null && b != null && a.a == b.a && a.b == b.b && Math.abs(a.required - b.required) < 1e-12) {
        // ArmClearance's distance can stop early once it is below the margin; only compare when it is clear of zero.
        return;
      }
      var near = a != null && Math.abs(a.distance - a.required) <= TOLERANCE || b != null && Math.abs(b.distance - b.required) <= TOLERANCE;
      if (near) {
        boundary++;
        Sys.println('CL4A_PARITY boundary $name $what: ${describe(a)} vs ${describe(b)}');
        return;
      }
      check(false, '$name $what: ${describe(a)} vs ${describe(b)}');
    }
    for (q in samples) for (contact in [false, true]) {
      agree("violation", hull.violation(q, contact), native.violation(q, contact));
      var errors = [for (_ in q) 0.01];
      var delta = hull.displacementBounds(q, errors);
      check(delta.join(",") == native.displacementBounds(q, errors).join(","), '$name: the same displacement bounds');
      agree("inflated violation", hull.violation(q, contact, null, delta), native.violation(q, contact, null, delta));
      var a = hull.closest(q, contact), b = native.closest(q, contact);
      if (a != null && b != null) {
        worstClosest = Math.max(worstClosest, Math.abs(a.distance - b.distance));
        check(Math.abs(a.distance - b.distance) <= TOLERANCE, '$name closest: ${describe(a)} vs ${describe(b)}');
      } else check(a == null && b == null, '$name: closest pairs exist with both');
    }
    var sweeps = 0, proofs = 0, certified = 0;
    for (i in 1...Std.int(Math.min(samples.length, 41))) {
      var from = samples[i - 1], to = samples[i];
      sweeps++;
      agree("sweep", hull.sweep(from, to), native.sweep(from, to));
      // A straight path as a quintic certificate, proved with either world.
      var path = straight(from, to);
      var withHulls = new JointCurveClearance(hull, path, 1e-4, false);
      var withCollisionKit = new JointCurveClearance(native, path, 1e-4, false);
      var a = withHulls.check(), b = withCollisionKit.check();
      proofs++;
      if (a) certified++;
      if (a != b || withHulls.failedSpan != withCollisionKit.failedSpan) {
        var nearA = withHulls.failure != null && Math.abs(withHulls.failure.distance - withHulls.failure.required) <= TOLERANCE;
        var nearB = withCollisionKit.failure != null && Math.abs(withCollisionKit.failure.distance - withCollisionKit.failure.required) <= TOLERANCE;
        if (nearA || nearB) {
          boundary++;
          Sys.println('CL4A_PARITY boundary $name proof: $a/$b at spans ${withHulls.failedSpan}/${withCollisionKit.failedSpan}');
        } else check(false, '$name: the proof differs ($a at ${withHulls.failedSpan} / $b at ${withCollisionKit.failedSpan})');
      } else assertions++;
    }
    Sys.println('CL4A_PARITY $name: ${samples.length} configurations, $queries violation queries ($violations violating), '
      + '$sweeps sweeps, $proofs proofs ($certified certified), $boundary at the boundary, worst closest difference $worstClosest m');
  }

  /** A straight joint path from `a` to `b` as path samples (zero curvature). */
  static function straight(a:Array<Float>, b:Array<Float>):JointPathSamples {
    var count = 9;
    var s = [for (i in 0...count) i / (count - 1)];
    var q = [for (t in s) [for (j in 0...a.length) a[j] + (b[j] - a[j]) * t]];
    var prime = [for (_ in s) [for (j in 0...a.length) b[j] - a[j]]];
    var zero = [for (_ in s) [for (_ in a) 0.0]];
    return new JointPathSamples(s, q, prime, zero, zero);
  }

  static function describe(v:Null<ClearanceViolation>):String
    return v == null ? "clear" : '${v.a}/${v.b} ${v.distance} < ${v.required}';

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}

private class Rng {
  var state:Int;
  public function new(seed:Int) state = seed;
  public function signed():Float {
    state = (state * 1103515245 + 12345) & 0x7fffffff;
    return state / 2147483647.0 * 2.0 - 1.0;
  }
}
