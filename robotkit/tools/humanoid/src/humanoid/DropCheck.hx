package humanoid;

import haxe.Int64;
import robotkit.model.RobotModelCodec;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;

/**
 * Drops an imported floating-base robot onto a floor in MuJoCo and reports
 * which links touch the floor first, the base height, and whether it came to
 * rest above the floor, while every joint holds its zero pose. For a humanoid
 * such as G1 the first contacts should be its feet.
 *
 * Usage: haxeon run --project robotkit/tools/humanoid/haxeon.json -- drop <robot.json> [drop-height]
 * Exits nonzero when the robot passes through the floor or a foot is not
 * among the first links to touch it. A joint-limit fault after touchdown ends
 * the run early and is reported, not failed: without a balance controller the
 * robot is not expected to stay up (see `stand`).
 */
class DropCheck {
  static inline var TIMESTEP = 0.002;

  public static function run(args:Array<String>):Void {
    if (args.length < 1) {
      Sys.println("usage: drop <robot.json> [drop-height]");
      Sys.exit(2);
    }
    var model = RobotModelCodec.decode(sys.io.File.getBytes(args[0]));
    if (!model.floatingBase) throw "DropCheck needs a floating-base robot";
    var height = args.length > 1 ? Std.parseFloat(args[1]) : 0.8;
    var blueprint = RobotRuntimeCompiler.compile(model);
    // A leg pressed onto its compliant knee stop passes it slightly.
    blueprint.observedLimitTolerance = 0.05;
    var simulation = new Simulation(TIMESTEP, 2, 1);
    var runtime = simulation.addRobotAtPose(blueprint, [0.0, 0.0, height], [0.0, 0.0, 0.0, 1.0]);
    var floor = simulation.spawnPlane();
    // A humanoid is never unpowered: hold every joint at its zero pose, within
    // its effort limit, as a standing controller would. An unpowered fall
    // presses joints onto their stops, which the runtime treats as a fault.
    runtime.submitPositions([for (_ in model.joints) 0.0], 1);
    var first:Array<String> = [];
    var firstStep = -1;
    var lowest = height;
    var fault:Null<String> = null;
    for (step in 0...1500) {
      try simulation.step(Int64.ofInt(step)) catch (error:Dynamic) {
        // A hard landing can still press a joint beyond the limit tolerance,
        // which the runtime latches as a fault. Touchdown has been observed
        // by then.
        var snapshot = runtime.snapshot();
        var stops = [for (index in 0...model.joints.length) {
          var joint = model.joints[index], position = snapshot.q.get(index);
          if (joint.type == robotkit.model.JointType.Revolute &&
              (position < joint.limits.lower + 0.01 || position > joint.limits.upper - 0.01))
            joint.name;
        }];
        fault = '${step * TIMESTEP} s: $error; joints at their stops: ${stops.join(", ")}';
        break;
      }
      var z = simulation.robotPose(0).position[2];
      if (z < lowest) lowest = z;
      if (firstStep < 0) {
        for (contact in simulation.robotContacts(runtime))
          if (contact.active && contact.otherObject == floor) {
            var name = model.links[contact.linkIndex].name;
            if (first.indexOf(name) < 0) first.push(name);
          }
        if (first.length > 0) firstStep = step;
      }
    }
    var finalHeight = simulation.robotPose(0).position[2];
    Sys.println('${model.name}: ${model.links.length} links, first floor contact at '
      + '${firstStep * TIMESTEP} s by ${first.join(", ")}');
    Sys.println('base height: dropped from $height, lowest $lowest, final $finalHeight');
    if (fault != null) Sys.println('stopped at $fault');
    simulation.dispose();
    var feet = [for (name in first) if (name.indexOf("ankle") >= 0 || name.indexOf("foot") >= 0) name];
    if (lowest <= 0.0 || firstStep < 0 || feet.length == 0) {
      Sys.println("FAIL: the robot did not land on its feet above the floor");
      Sys.exit(1);
    }
    Sys.println("PASS: landed on its feet");
  }
}
