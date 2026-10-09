package toolpathkit.motion;

import haxe.Int64;
import motionkit.trajectory.ValidationLimits;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.robot.AxisKinematics;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.CompiledProgram;
import motionkit.robot.StartTolerances;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.path.Point3;
import toolpathkit.setup.TravelEnvelope;

/** Binds pure CncKit programs to the physical joints of a direct XYZ machine. */
class ToolpathMotionBinding {
  public final machine:MachineBinding;
  public final blueprint:MotionSystemBlueprint;
  public final solver:AxisKinematics;
  public final compiler:ProgramCompiler;

  /**
    `heldJoints` are joints the XYZ axes do not move and this binding does not plan (a printer's extruder
    and the motor that turns it): they hold their position.
  **/
  public function new(machine:MachineBinding, blueprint:MotionSystemBlueprint,
      ?modelRevision:Int64, ?calibrationRevision:Int64, ?heldJoints:Array<String>) {
    if (machine == null || blueprint == null)
      throw "CNC binding needs a machine and motion-system blueprint";
    this.machine = machine;
    this.blueprint = blueprint;
    solver = new AxisKinematics(blueprint, machine.xAxisId,
      machine.yAxisId, machine.zAxisId);
    var count = solver.jointCount();
    var envelopeLower:Array<Float> = [];
    var envelopeUpper:Array<Float> = [];
    for (axis in [solver.x, solver.y, solver.z]) {
      var low = axis.lowerLimit, high = axis.upperLimit;
      var zero = [for (_ in 0...count) 0.0];
      var one = zero.copy();
      axis.writeLogicalPosition(zero, 0.0);
      axis.writeLogicalPosition(one, 1.0);
      for (joint in axis.jointIndices) {
        var bounds = blueprint.model.joints[joint].limits;
        var scale = one[joint] - zero[joint];
        if (bounds.lower < bounds.upper && Math.abs(scale) > 1e-12) {
          var a = (bounds.lower - zero[joint]) / scale;
          var b = (bounds.upper - zero[joint]) / scale;
          low = Math.max(low, Math.min(a, b));
          high = Math.min(high, Math.max(a, b));
        }
      }
      envelopeLower.push(low); envelopeUpper.push(high);
    }
    var travel = machine.travel;
    if (travel != null)
      for (axis in 0...3) {
        var lower = [travel.lower.x, travel.lower.y, travel.lower.z];
        var upper = [travel.upper.x, travel.upper.y, travel.upper.z];
        envelopeLower[axis] = Math.max(envelopeLower[axis], lower[axis]);
        envelopeUpper[axis] = Math.min(envelopeUpper[axis], upper[axis]);
      }
    machine.setTravel(new TravelEnvelope(
      new Point3(envelopeLower[0], envelopeLower[1], envelopeLower[2]),
      new Point3(envelopeUpper[0], envelopeUpper[1], envelopeUpper[2])));
    var limits = new ValidationLimits(count,
      modelRevision == null ? Int64.ofInt(blueprint.runtime.revision) : modelRevision,
      calibrationRevision == null ? Int64.ofInt(blueprint.runtime.calibrationRevision) : calibrationRevision);
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
    // The extruder: a joint the XYZ axes do not move, driven by distance along the path. Its limits are the
    // drive's (its motor through the gears); the melt-rate limit is applied to the speed of the moves that feed it.
    var printer = machine.printer;
    if (printer != null) {
      var extruder = -1;
      for (joint in 0...count) if (blueprint.model.joints[joint].id == printer.extruderJoint) extruder = joint;
      if (extruder < 0) throw 'Printer extruder joint "${printer.extruderJoint}" is not in the machine';
      // A planning model of the axes alone holds the drive's limits on the joint; the full machine derives them.
      var own = blueprint.model.joints[extruder].limits;
      var drive = own.velocity != null && own.maxAcceleration != null ? own :
        blueprint.model.coupledLimits(printer.extruderJoint, new robotkit.model.SteadyLoads());
      // Pressure advance adds `lead` times the extruder's acceleration to its speed and `lead` times its jerk to its
      // acceleration: the plan leaves room for both.
      var room = printer.pressureAdvance > 0.0;
      velocity[extruder] = drive.requireVelocity() * (room ? 0.8 : 1.0);
      acceleration[extruder] = drive.requireAcceleration() * (room ? 0.6 : 1.0);
      jerk[extruder] = acceleration[extruder] * 25.0;
      jump[extruder] = 0.1;
      printer.bind(extruder);
    }
    // A joint that follows several axes (a CoreXY motor's pulley) belongs to none: it moves at most as
    // fast as its terms add up to, which the coupled trajectory keeps exactly.
    var ids = [for (joint in blueprint.model.joints) joint.id];
    for (_ in 0...count) for (joint in 0...count) if (velocity[joint] <= 0.0) {
      var terms = [for (coupling in blueprint.model.couplings) if (coupling.follower == ids[joint]) coupling];
      if (terms.length == 0) continue;
      var ready = true;
      for (term in terms) if (velocity[ids.indexOf(term.leader)] <= 0.0) ready = false;
      if (!ready) continue;
      velocity[joint] = 0.0; acceleration[joint] = 0.0; jerk[joint] = 0.0; jump[joint] = 0.0;
      for (term in terms) {
        var leader = ids.indexOf(term.leader), scale = Math.abs(term.ratio);
        velocity[joint] += scale * velocity[leader];
        acceleration[joint] += scale * acceleration[leader];
        jerk[joint] += scale * jerk[leader];
        jump[joint] += scale * jump[leader];
      }
    }
    for (joint in 0...count) {
      if (velocity[joint] <= 0.0 || acceleration[joint] <= 0.0) {
        if (blueprint.model.joints[joint].type != robotkit.model.JointType.Fixed &&
            (heldJoints == null || heldJoints.indexOf(blueprint.model.joints[joint].id) < 0))
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
    var startTolerances = new StartTolerances(
      [for (_ in 0...count) machine.positionTolerance],
      [for (joint in 0...count) acceleration[joint] * blueprint.fixedTimestepSeconds],
      [for (joint in 0...count) jerk[joint] * blueprint.fixedTimestepSeconds]);
    compiler = new ProgramCompiler(solver, limits, machine.frameId,
      velocity, acceleration, jerk, startTolerances, null, 0.002, 0.1,
      machine.positionTolerance, machine.orientationTolerance,
      null, null, jump,
      [for (joint in blueprint.model.joints) joint.id], blueprint.model.couplings,
      blueprint.fixedTimestepSeconds);
    if (printer != null) { compiler.pressureAdvance = printer.pressureAdvance; compiler.pressureAdvanceWindow = printer.pressureAdvanceWindow; }
  }

  public function compile(program:ToolpathProgram, initialJoints:Array<Float>,
      firstPlanId:Int64):CompiledProgram {
    var lowered = ToolpathMotion.lower(program, machine);
    if (lowered.program == null) throw "toolpath has no executable motion";
    return compiler.compile(lowered.program, initialJoints, firstPlanId);
  }
}
