package motionkit.robot;

import cnckit.CncCompiler;
import cnckit.CncMachine;
import haxe.Int64;
import motionkit.trajectory.ValidationLimits;

/** Binds pure CncKit programs to the physical joints of a direct XYZ machine. */
class CncMotionBinding {
  public final machine:CncMachine;
  public final blueprint:MotionSystemBlueprint;
  public final solver:AxisKinematics;
  public final compiler:ProgramCompiler;

  public function new(machine:CncMachine, blueprint:MotionSystemBlueprint) {
    if (machine == null || blueprint == null)
      throw "CNC binding needs a machine and motion-system blueprint";
    this.machine = machine;
    this.blueprint = blueprint;
    solver = new AxisKinematics(blueprint, machine.xAxisId,
      machine.yAxisId, machine.zAxisId);
    var count = solver.jointCount();
    var limits = new ValidationLimits(count, Int64.ofInt(blueprint.runtime.revision),
      Int64.ofInt(blueprint.runtime.calibrationRevision));
    var velocity = [for (_ in 0...count) 0.0];
    var acceleration = [for (_ in 0...count) 0.0];
    var jerk = [for (_ in 0...count) 0.0];
    var jump = [for (_ in 0...count) 0.1];
    for (axis in [solver.x, solver.y, solver.z]) {
      if (axis.maxVelocity <= 0.0 || axis.maxAcceleration <= 0.0)
        throw 'CNC axis "${axis.id}" needs velocity and acceleration limits';
      for (entry in 0...axis.jointIndices.length) {
        var joint = axis.jointIndices[entry];
        var scale = Math.abs(axis.jointScale(entry));
        velocity[joint] = axis.maxVelocity * scale;
        acceleration[joint] = axis.maxAcceleration * scale;
        jerk[joint] = acceleration[joint] * 25.0;
        jump[joint] = Math.max(0.1, 0.1 * scale);
      }
    }
    for (joint in 0...count) {
      if (velocity[joint] <= 0.0 || acceleration[joint] <= 0.0) {
        if (blueprint.model.joints[joint].type != robotkit.model.JointType.Fixed)
          throw 'CNC joint $joint is not bound to an XYZ axis';
        // Assembly root and mounting joints remain stationary, but native
        // trajectory validation still needs finite positive limit arrays.
        velocity[joint] = 1.0;
        acceleration[joint] = 1.0;
        jerk[joint] = 25.0;
      }
      var bounds = blueprint.model.joints[joint].limits;
      if (bounds.lower < bounds.upper) limits.position(joint,
        bounds.lower, bounds.upper);
      limits.velocity(joint, velocity[joint]);
      limits.acceleration(joint, acceleration[joint]);
      limits.jerk(joint, jerk[joint]);
    }
    compiler = new ProgramCompiler(solver, limits, machine.frameId,
      velocity, acceleration, jerk, null, 0.002, 0.1,
      machine.positionTolerance, machine.orientationTolerance,
      null, null, jump,
      [for (joint in blueprint.model.joints) joint.id], blueprint.model.couplings);
  }

  public function compile(source:String, initialJoints:Array<Float>,
      firstPlanId:Int64):CompiledProgram {
    var current = solver.forward(initialJoints);
    var initial = machine.initialPosition;
    if (Math.abs(current.x - initial[0]) > machine.positionTolerance ||
        Math.abs(current.y - initial[1]) > machine.positionTolerance ||
        Math.abs(current.z - initial[2]) > machine.positionTolerance)
      throw "CNC machine initial position differs from commanded joints";
    return compiler.compile(new CncCompiler(machine).compile(source),
      initialJoints, firstPlanId);
  }
}
