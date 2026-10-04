package app;

import haxe.Int64;
import materia.project.SceneArtifact.SceneArtifactAxisProgram;
import motionkit.MotionOptions;
import motionkit.program.MotionProgram;
import motionkit.program.MotionOp;
import motionkit.program.MoveTarget;
import motionkit.program.Blend;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.JointKinematics;
import motionkit.robot.MotorSpaceConstraints;
import motionkit.robot.PlanCheck;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.StartTolerances;
import motionkit.trajectory.ValidationLimits;
import nativekit.sim.SimSession;

/** Executes the generated independent-axis program through the normal robot plan stream. */
class MachineProgramPlayer implements SessionMember {
  final robot:AssemblyRobot;
  final session:SimSession;
  final authored:SceneArtifactAxisProgram;
  final compiler:ProgramCompiler;
  final indices:Array<Int>;
  final program:MotionProgram;
  final tolerance:Float;
  final synchronizationDelay:Float;
  var readyAt:Float;
  public var motion(default, null):ManipulatorMotion;
  public var passes(default, null):Int = 0;
  public var failure(default, null):Null<String> = null;
  var started = false;
  var finished = false;

  public function new(authored:SceneArtifactAxisProgram, robot:AssemblyRobot, session:SimSession, virtualDevice:Bool = false) {
    this.authored = authored; this.robot = robot; this.session = session;
    tolerance = authored.positionTolerance == null ? 1e-5 : authored.positionTolerance;
    // Two deterministic time-sync replies establish the board's host clock mapping.
    synchronizationDelay = virtualDevice ? 0.15 : 0.0;
    readyAt = session.simulationTime() + synchronizationDelay;
    indices = [];
    var velocities:Array<Float> = [], accelerations:Array<Float> = [], jerks:Array<Float> = [];
    var limits = new ValidationLimits(authored.axes.length, Int64.ofInt(robot.blueprint.revision),
      Int64.ofInt(robot.blueprint.calibrationRevision));
    for (i in 0...authored.axes.length) {
      var found = [for (j in 0...robot.model.joints.length) if (robot.model.joints[j].id == authored.axes[i]) j];
      if (found.length != 1) throw "Machine program axis is missing";
      indices.push(found[0]);
      var joint = robot.model.joints[found[0]], bound = joint.limits;
      if ([for (coupling in robot.model.couplings) if (coupling.follower == joint.id) coupling].length > 0)
        throw "Machine programs must plan independent axes";
      velocities.push(bound.requireVelocity() * 0.95); accelerations.push(bound.requireAcceleration() * 0.95);
      jerks.push(bound.requireAcceleration() * 40.0);
      if (bound.lower < bound.upper) limits.position(i, bound.lower, bound.upper);
      limits.velocity(i, velocities[i]); limits.acceleration(i, accelerations[i]); limits.jerk(i, jerks[i]);
      for (point in authored.waypoints)
        if (bound.lower < bound.upper && (point[i] < bound.lower || point[i] > bound.upper))
          throw "Machine program waypoint is outside axis travel";
    }
    compiler = new ProgramCompiler(new JointKinematics(indices.length), limits, "machine-axes",
      velocities, accelerations, jerks, StartTolerances.uniform(indices.length, tolerance, 0.02, 0.02));
    compiler.motorSpace = MotorSpaceConstraints.of(robot.model, authored.axes, 0.95);
    compiler.planCheck = new PlanCheck(robot.model, authored.axes);
    program = new MotionProgram([for (point in authored.waypoints)
      MotionOp.MoveJ(MoveTarget.JointTarget(point.copy()), new MotionOptions(), Blend.ExactStop)]);
    motion = newMotion();
  }

  function newMotion():ManipulatorMotion
    return new ManipulatorMotion(robot.robot, compiler, _ -> null, () -> robot.runtime.pollEvents(), indices,
      [for (_ in indices) tolerance]);

  public function feed():Void {
    if (failure != null || finished || session.simulationTime() < readyAt) return;
    if (!started) { motion.run(program); started = true; }
    motion.update(session.fixedTimestep());
    failure = motion.failure;
    if (motion.completed) {
      passes++;
      if (authored.loop) { started = false; motion = newMotion(); } else finished = true;
    }
  }

  public function beforeReset():Void { if (motion.running) motion.abort(); }
  public function reset():Void { motion = newMotion(); started = false; finished = false; passes = 0; failure = null; readyAt = session.simulationTime() + synchronizationDelay; }
  public function present():Void {}
}
