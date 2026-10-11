package app;

import motionkit.planner.MotionPlanner;
import motionkit.planner.MotionPlan;
import motionkit.planner.PlannerSpace;
import motionkit.planner.RrtConnect;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import collisionkit.CollisionWorld;

typedef ScrewEscapeMargin = {
  var first:Int;
  var second:Int;
  var margin:Float;
  var air:Float;
  var contact:Float;
}

/** Withdraw with measured tool orientation, then restore normal travel clearance and orientation. */
class ScrewEscapePlanner implements MotionPlanner {
  final escape:PlannerSpace;
  final normal:PlannerSpace;
  final solver:KinematicsSolver;
  final world:CollisionWorld;
  final margins:Array<ScrewEscapeMargin>;

  public function new(escape:PlannerSpace, normal:PlannerSpace, solver:KinematicsSolver,
      world:CollisionWorld, margins:Array<ScrewEscapeMargin>) {
    this.escape = escape;
    this.normal = normal;
    this.solver = solver.fork();
    this.world = world;
    this.margins = margins.copy();
  }

  public function plan(start:Array<Float>, goal:Array<Float>):MotionPlan {
    var began = Sys.time(), problem = normal.invalid(goal);
    var checkSeconds = Sys.time()-began;
    if (problem != null) return failed('retreat has no normal clearance: $problem',began);
    var startProblem = escapeInvalid(start);
    if (startProblem != null) return failed('retreat starts without identified escape clearance: $startProblem',began);
    var from = solver.forward(start), to = solver.forward(goal);
    var dx = to.x-from.x, dy = to.y-from.y, dz = to.z-from.z;
    var count = Std.int(Math.ceil(Math.sqrt(dx*dx+dy*dy+dz*dz)/0.001));
    if (count < 1) count = 1;
    var path = [start.copy()], edges:Array<Float> = [], previous = start.copy();
    var tolerance = new IkTolerance(0.00005,0.001,300,0.03);
    for (i in 1...count+1) {
      var t = i/count;
      var target = new Pose3(from.x+dx*t,from.y+dy*t,from.z+dz*t,from.qx,from.qy,from.qz,from.qw);
      var next = solver.solvePose(target,previous,tolerance);
      if (next == null) return failed('withdrawal is unreachable at sample $i of $count',began);
      var checkBegan = Sys.time(), invalid = escapeInvalid(next);
      checkSeconds += Sys.time()-checkBegan;
      if (invalid != null) return failed('withdrawal sample $i of $count: $invalid',began);
      for (value in previous) edges.push(value);
      for (value in next) edges.push(value);
      path.push(next); previous = next;
    }
    var edgesBegan = Sys.time(), checked = escapeEdgesClear(edges,count);
    checkSeconds += Sys.time()-edgesBegan;
    for (i in 0...checked.length) if (!checked[i])
      return failed('withdrawal edge ${i+1} of $count could not be certified',began);
    var endBegan = Sys.time(), clear = normal.invalid(previous);
    checkSeconds += Sys.time()-endBegan;
    if (clear != null) return failed('withdrawal does not restore normal clearance: $clear',began);
    var finalTravel = new RrtConnect(normal,17).plan(previous,goal);
    var tail = finalTravel.waypoints;
    if (tail == null) return failed('orientation after withdrawal: ${finalTravel.failure}',began);
    for (i in 1...tail.length) path.push(tail[i]);
    return new MotionPlan(path,"",finalTravel.iterations,path.length,count+finalTravel.edgeChecks,
      checkSeconds+finalTravel.checkSeconds,Sys.time()-began);
  }

  function escapeInvalid(q:Array<Float>):Null<String> {
    enableEscapeMargins(true);
    var result:Null<String> = null;
    try {
      result = escape.invalid(q);
    } catch (error:Dynamic) {
      enableEscapeMargins(false);
      throw error;
    }
    enableEscapeMargins(false);
    return result;
  }

  function escapeEdgesClear(edges:Array<Float>, count:Int):Array<Bool> {
    enableEscapeMargins(true);
    var result:Array<Bool> = [];
    try {
      result = escape.edgesClear(edges,count);
    } catch (error:Dynamic) {
      enableEscapeMargins(false);
      throw error;
    }
    enableEscapeMargins(false);
    return result;
  }

  function enableEscapeMargins(enabled:Bool):Void {
    for (pair in margins) world.setObjectMargins(pair.first,pair.second,
      enabled ? pair.margin : pair.air, enabled ? pair.margin : pair.contact);
  }

  function failed(reason:String, began:Float):MotionPlan
    return new MotionPlan(null,'lost-screw $reason',0,0,0,0,Sys.time()-began);
}
