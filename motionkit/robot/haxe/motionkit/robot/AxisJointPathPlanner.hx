package motionkit.robot;

import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Twist6;
import motionkit.path.PosePath;
import motionkit.path.PoseMath;
import motionkit.planner.JointPathSamples;

/** Exact affine logical-axis mapping, including motor scales and coupled followers. */
class AxisJointPathPlanner implements JointPathPlanner {
  public final solver:AxisKinematics;
  public function new(solver:AxisKinematics) {
    if (solver == null) throw "Axis path planner requires axis kinematics";
    this.solver = solver;
  }
  public function withConfiguration(configuration:motionkit.kinematics.SixAxisConfiguration):JointPathPlanner
    throw "Configuration pin requires a labelled six-axis geometric backend";
  public function withSolver(worker:KinematicsSolver):JointPathPlanner {
    if (!Std.isOfType(worker, AxisKinematics)) throw "Axis planner worker requires axis kinematics";
    return new AxisJointPathPlanner(cast worker);
  }
  public function allowsFreeStart():Bool return false;
  public function retreatTarget():Null<Array<Float>> return null;
  public function checkPathClearance(path:JointPathSamples,tolerance:Float):Bool return false;
  public function checkMotion(trajectory:motionkit.trajectory.Trajectory):Null<robotkit.manipulation.ClearanceViolation> return null;
  public function plan(path:PosePath, request:PathRequest, ?pinStart:Bool,
      ?entryCheck:(Array<Float>,Array<Float>)->Null<robotkit.manipulation.ClearanceViolation>,
      ?exitCheck:Array<Float>->Null<robotkit.manipulation.ClearanceViolation>):JointPathSamples {
    if (path == null || request == null || request.distances.length < 2 ||
        request.distances[0] != 0 || request.distances[request.distances.length - 1] != path.length())
      throw "Axis path request must span its complete authored path";
    var provider = new PosePathRefinement(path);
    var positions:Array<Array<Float>> = [], first:Array<Array<Float>> = [];
    var second:Array<Array<Float>> = [], before:Array<Array<Float>> = [];
    var previous = request.startQ;
    for (i in 0...request.distances.length) {
      var task = provider.at(request.distances[i]);
      if (!StructuredJointPathPlanner.sameFreedom(task.freedom, request.freedoms[i]) ||
          PoseMath.distance(task.pose, request.poses[i]) > request.tolerance.position ||
          ToolFreedom.orientationError(task.pose, request.poses[i], task.freedom) > request.tolerance.orientation)
        throw 'Axis path request differs from authored geometry or freedom at sample $i';
      var q = i == 0 ? request.startQ.copy() : solver.solvePose(task.pose, previous, request.tolerance, task.freedom);
      if (q == null || PoseMath.distance(solver.forward(q), task.pose) > request.tolerance.position ||
          ToolFreedom.orientationError(solver.forward(q), task.pose, task.freedom) > request.tolerance.orientation ||
          solver.solvePose(task.pose, q, request.tolerance, task.freedom) == null)
        throw 'Axis path is unreachable at sample $i, distance ${request.distances[i]}';
      // A measured start may differ from the authored pose within task tolerance.
      // Validate its own affine mapping in logical metres, including rotary motor scales.
      var exact = solver.solvePose(solver.forward(q), q, request.tolerance, motionkit.path.OrientationPolicy.Interpolated);
      for (axis in [solver.x,solver.y,solver.z]) for (slot in 0...axis.jointIndices.length) {
        var joint = axis.jointIndices[slot];
        if (Math.abs(q[joint]-exact[joint])/Math.abs(axis.jointScale(slot)) > request.tolerance.position)
          throw 'Axis path start violates joint mapping at sample $i';
      }
      for (joint in 0...q.length) if (Math.abs(q[joint] - previous[joint]) > request.maxJump[joint])
        throw 'Axis path exceeds joint $joint jump at sample $i, distance ${request.distances[i]}';
      function map(values:Array<Float>):Array<Float> {
        var mapped = solver.solveDifferential(q, new Twist6(values[0], values[1], values[2],
          values[3], values[4], values[5]), null, task.freedom);
        if (mapped == null) throw 'Axis path derivatives are unreachable at sample $i';
        return mapped;
      }
      positions.push(q); first.push(map(task.velocity));
      second.push(map(task.acceleration)); before.push(map(task.accelerationBefore));
      previous = q;
    }
    return new JointPathSamples(request.distances, positions, first, second, before);
  }
}
