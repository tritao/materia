package robotkit.manipulation;

import robotkit.spatial.Vec3;
import robotkit.tool.MassProperties;
import robotkit.tool.Tool;
import robotkit.tool.WorkpieceLoad;

/** Sample a joint path against a robot's static carried-load chart. Gravity
 * is in base -Z; acceleration and dynamic joint torques are outside this check. */
class PayloadChecker {
  public static inline var GRAVITY:Float = 9.80665;

  public static function checkPath(manipulator:Manipulator, tool:Tool,
      workpiece:Null<WorkpieceLoad>, chart:RobotLoadChart,
      waypoints:Array<Array<Float>>, ?maxJointStep:Float = 0.02):PayloadCheckResult {
    if (manipulator == null || tool == null || chart == null || waypoints == null ||
        waypoints.length == 0 || !Math.isFinite(maxJointStep) || maxJointStep <= 0)
      throw "Payload check requires a manipulator, tool, chart, path and positive joint step";
    if (tool.flangeTTcp.translation.sub(manipulator.flangeTTcp.translation).norm() > 1e-6 ||
        tool.flangeTTcp.rotation.angularDistance(manipulator.flangeTTcp.rotation) > 1e-6)
      throw "Payload check tool TCP does not match the manipulator TCP";
    if (tool.massProperties == null)
      throw "Payload check requires the tool's centre of mass, not mass alone";
    var load:MassProperties = cast tool.massProperties;
    if (workpiece != null) load = load.combined(workpiece.atFlange());
    var dof = manipulator.dofCount();
    for (waypoint in waypoints) {
      if (waypoint == null || waypoint.length != dof)
        throw "Payload path waypoint has the wrong joint count";
      for (value in waypoint) if (!Math.isFinite(value))
        throw "Payload path joint values must be finite";
    }
    var checked = 0;
    for (segment in 0...waypoints.length) {
      var from = segment == 0 ? waypoints[0] : waypoints[segment - 1];
      var to = waypoints[segment];
      var steps = 1;
      if (segment > 0) for (i in 0...dof)
        steps = Std.int(Math.max(steps, Math.ceil(Math.abs(to[i] - from[i]) / maxJointStep)));
      var firstStep = segment == 0 ? 0 : 1;
      var lastStep = segment == 0 ? 0 : steps;
      for (step in firstStep...(lastStep + 1)) {
        var fraction = step / steps;
        var q = [for (i in 0...dof) from[i] + (to[i] - from[i]) * fraction];
        var pose = manipulator.forwardKinematics(q);
        var limit = chart.limitsAt(q, pose);
        checked++;
        if (limit == null)
          return new PayloadCheckResult(false, segment, fraction,
            "Flange pose is outside the robot load chart", checked);
        if (load.massKg > limit.maxMassKg + 1e-9)
          return new PayloadCheckResult(false, segment, fraction,
            'Carried mass ${load.massKg} kg exceeds ${limit.maxMassKg} kg', checked);
        var offset = pose.transformVector(load.centerOfMass);
        var gravity = new Vec3(0, 0, -GRAVITY * load.massKg);
        var moment = offset.cross(gravity).norm();
        if (moment > limit.maxFlangeMomentNm + 1e-9)
          return new PayloadCheckResult(false, segment, fraction,
            'Flange gravity moment $moment Nm exceeds ${limit.maxFlangeMomentNm} Nm', checked);
      }
    }
    return new PayloadCheckResult(true, -1, 1.0, null, checked);
  }
}
