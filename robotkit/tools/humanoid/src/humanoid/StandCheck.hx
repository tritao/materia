package humanoid;

import haxe.Int64;
import robotkit.model.RobotModelCodec;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.world.JointTarget;

/**
 * Starts an imported humanoid in a pose on a floor and holds that pose with
 * each actuator's default servo gains, as the MJCF position actuators do.
 * Passes when the base keeps its height and stays upright.
 *
 * Usage: stand <robot.json> <poses.json> <pose> <seconds> <control-period-s> <substeps> <integrator>
 *          <solver-iterations> <line-search-iterations>
 */
class StandCheck {
  public static function run(args:Array<String>):Void {
    if (args.length != 9) {
      Sys.println("usage: stand <robot.json> <poses.json> <pose> <seconds> <control-period-s> <substeps> "
        + "<integrator> <solver-iterations> <line-search-iterations>");
      Sys.exit(2);
    }
    var model = RobotModelCodec.decode(sys.io.File.getBytes(args[0]));
    var pose = Pose.load(args[1], args[2]);
    var seconds = Std.parseFloat(args[3]), period = Std.parseFloat(args[4]);
    var simulation = new Simulation(period, Std.parseInt(args[5]), 1, null, Std.parseInt(args[6]), 0,
      Std.parseInt(args[7]), Std.parseInt(args[8]));
    var blueprint = RobotRuntimeCompiler.compile(model);
    blueprint.observedLimitTolerance = 0.05;
    var runtime = simulation.addRobotAtPose(blueprint, pose.rootPosition, pose.rootRotation);
    simulation.spawnPlane();
    simulation.setJointPositions(0, pose.jointPositions(model));

    var actuators = new ActuatorMap(model);
    var hold:Array<JointTarget> = [];
    for (i in 0...model.actuators.length) {
      var actuator = model.actuators[i];
      var gear = actuators.gears[i];
      if (actuator.servoStiffness <= 0.0) continue;
      hold.push(JointTarget.servo(actuators.joints[i], pose.actuatorTarget(actuator.id) / gear, 0.0,
        gear * gear * actuator.servoStiffness, gear * gear * actuator.servoDamping, 0.0));
    }
    if (hold.length == 0) throw "the model has no actuators with servo gains";
    runtime.submitTargets(hold, 1);

    var startHeight = pose.rootPosition[2], lowest = startHeight, worstTilt = 0.0;
    var ticks = Std.int(Math.round(seconds / period));
    for (tick in 0...ticks) {
      simulation.step(Int64.ofInt(tick));
      var base = simulation.robotPose(0);
      lowest = Math.min(lowest, base.position[2]);
      // Tilt: angle between the base's up axis and world up.
      var q = base.rotation;
      var upZ = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
      worstTilt = Math.max(worstTilt, Math.acos(Math.max(-1.0, Math.min(1.0, upZ))));
      if ((tick + 1) % Std.int(Math.round(1.0 / period)) == 0)
        Sys.println('t=${(tick + 1) * period}: base height ${base.position[2]}, tilt so far $worstTilt rad');
    }
    var finalHeight = simulation.robotPose(0).position[2];
    simulation.dispose();
    Sys.println('${model.name} holding "${pose.name}" for $seconds s: base height $startHeight -> $finalHeight '
      + '(lowest $lowest), worst tilt $worstTilt rad');
    if (finalHeight < 0.9 * startHeight || worstTilt > 0.3) {
      Sys.println("FAIL: did not stay standing");
      Sys.exit(1);
    }
    Sys.println("PASS: stood");
  }
}
