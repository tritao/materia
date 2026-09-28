package humanoid;

import haxe.Int64;
import robotkit.model.RobotModelCodec;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.world.JointTarget;

/**
 * Sim-to-sim conformance (robotkit/plans/HUMANOID.md, HU-D5). Replays the
 * torque sequence that robotkit_mjcf_reference applied in plain MuJoCo, on the
 * same model imported into RobotKit, and compares the trajectories.
 *
 * Usage: conformance <robot.json> <reference.csv> <control-period-s> <substeps>
 *          <amplitude> <integrator> <solver-iterations> <line-search-iterations>
 *          <position-tolerance-m> <angle-tolerance-rad> [<poses.json> <pose>]
 * integrator is a SimulationSolver value (2 is implicit-fast); the solver
 * settings must match the MJCF file's, which robotkit_mjcf_import reports. With a pose, the
 * run starts from it and each actuator's control is the pose's control plus
 * the sequence, as robotkit_mjcf_reference does with that keyframe. An
 * actuator with servo gains is a position servo (a MuJoCo position actuator);
 * others apply effort. Exits nonzero when any tick exceeds a tolerance.
 */
class Conformance {
  public static function run(args:Array<String>):Void {
    if (args.length != 10 && args.length != 12) {
      Sys.println("usage: conformance <robot.json> <reference.csv> <control-period-s> <substeps> "
        + "<amplitude> <integrator> <solver-iterations> <line-search-iterations> "
        + "<position-tolerance-m> <angle-tolerance-rad> [<poses.json> <pose>]");
      Sys.exit(2);
    }
    var model = RobotModelCodec.decode(sys.io.File.getBytes(args[0]));
    var rows = [for (line in sys.io.File.getContent(args[1]).split("\n")) if (StringTools.trim(line) != "") line];
    var header = rows.shift().split(",");
    var reference = [for (row in rows) [for (value in row.split(",")) Std.parseFloat(value)]];
    var period = Std.parseFloat(args[2]);
    var substeps = Std.parseInt(args[3]);
    var amplitude = Std.parseFloat(args[4]);
    var integrator = Std.parseInt(args[5]);
    var solverIterations = Std.parseInt(args[6]), lineSearchIterations = Std.parseInt(args[7]);
    var positionTolerance = Std.parseFloat(args[8]);
    var angleTolerance = Std.parseFloat(args[9]);

    // Reference columns 8.. are joints in actuator order.
    var jointColumns:Array<Int> = [];
    for (column in 8...header.length) {
      var index = -1;
      for (joint in 0...model.joints.length) if (model.joints[joint].name == header[column]) index = joint;
      if (index < 0) throw 'reference joint ${header[column]} is not in the model';
      jointColumns.push(index);
    }
    var actuators = new ActuatorMap(model);
    var pose = args.length == 12 ? Pose.load(args[10], args[11]) : null;

    var start = reference[0];
    var simulation = new Simulation(period, substeps, 1, null, integrator, 0, solverIterations,
      lineSearchIterations);
    var blueprint = RobotRuntimeCompiler.compile(model);
    // A leg pressed onto its compliant knee stop passes it slightly.
    blueprint.observedLimitTolerance = 0.05;
    var runtime = simulation.addRobotAtPose(blueprint,
      [start[1], start[2], start[3]], [start[4], start[5], start[6], start[7]]);
    simulation.spawnPlane();
    if (pose != null) simulation.setJointPositions(0, pose.jointPositions(model));

    var worstPosition = 0.0, worstAngle = 0.0, worstJoint = 0.0, firstFailure = -1;
    var sequence = 1;
    for (tick in 0...reference.length - 1) {
      var t = tick * period;
      // The same sequence robotkit_mjcf_reference applies; a motor's gear
      // turns its control into joint effort.
      var targets:Array<JointTarget> = [];
      for (i in 0...model.actuators.length) {
        var actuator = model.actuators[i];
        var joint = actuators.joints[i], gear = actuators.gears[i];
        var control = (pose == null ? 0.0 : pose.actuatorTarget(actuator.id))
          + amplitude * Math.sin(2.0 * Math.PI * (0.7 + 0.3 * i) * t + 0.5 * i);
        if (actuator.servoStiffness > 0.0) {
          // A position actuator: force = kp (ctrl - gear q) - kv gear qdot on
          // an actuator length of gear q, with ctrl clamped to the joint range.
          var limits = model.joints[joint].limits;
          var position = Math.min(limits.upper, Math.max(limits.lower, control / gear));
          targets.push(JointTarget.servo(joint, position, 0.0, gear * gear * actuator.servoStiffness,
            gear * gear * actuator.servoDamping, 0.0));
        } else {
          targets.push(JointTarget.effort(joint, gear * control));
        }
      }
      runtime.submitTargets(targets, sequence++);
      simulation.step(Int64.ofInt(tick));
      var expected = reference[tick + 1];
      var pose = simulation.robotPose(0);
      var positionError = 0.0;
      for (axis in 0...3) positionError = Math.max(positionError, Math.abs(pose.position[axis] - expected[axis + 1]));
      var dot = 0.0;
      for (axis in 0...4) dot += pose.rotation[axis] * expected[axis + 4];
      var angleError = 2.0 * Math.acos(Math.min(1.0, Math.abs(dot)));
      var snapshot = runtime.snapshot();
      var jointError = 0.0, worstColumn = 0;
      for (column in 0...jointColumns.length) {
        var error = Math.abs(snapshot.q.get(jointColumns[column]) - expected[8 + column]);
        if (error > jointError) { jointError = error; worstColumn = column; }
      }
      worstPosition = Math.max(worstPosition, positionError);
      worstAngle = Math.max(worstAngle, Math.max(angleError, jointError));
      worstJoint = Math.max(worstJoint, jointError);
      if (firstFailure < 0 && (positionError > positionTolerance ||
          Math.max(angleError, jointError) > angleTolerance)) {
        firstFailure = tick + 1;
        Sys.println('first over tolerance, t=${expected[0]}: base ${positionError} m, ${angleError} rad; '
          + 'joints ${jointError} rad (worst ${header[8 + worstColumn]} at '
          + '${snapshot.q.get(jointColumns[worstColumn])}, reference ${expected[8 + worstColumn]})');
      }
      if (tick < 3 || (tick + 1) % Std.int(Math.max(1, Math.round(0.25 / period))) == 0)
        Sys.println('t=${expected[0]}: base ${positionError} m, ${angleError} rad; joints ${jointError} rad'
          + ' (worst ${header[8 + worstColumn]})');
    }
    simulation.dispose();
    Sys.println('${model.name}: ${reference.length - 1} ticks, worst base position $worstPosition m, '
      + 'worst angle $worstAngle rad (joints $worstJoint rad)');
    if (firstFailure >= 0) {
      Sys.println('FAIL: first exceeds tolerance at t=${reference[firstFailure][0]} s');
      Sys.exit(1);
    }
    Sys.println("PASS: RobotKit matches plain MuJoCo within tolerance");
  }
}
