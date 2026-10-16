import haxe.Int64;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import motionkit.path.DistanceDrive;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PosePath;
import motionkit.path.PoseWaypoint;
import motionkit.planner.JointPathSamples;
import motionkit.planner.JointPathClearanceProof;
import motionkit.planner.ToppraPathTiming;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.robot.DistanceDriveColumns;
import motionkit.robot.JointPathPlanner;
import motionkit.robot.PosePathRefinement;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
import motionkit.robot.TrajectoryClearanceProof;
import motionkit.robot.TrajectoryClearanceProof.ClearanceEvent;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ValidationLimits;
import robotkit.manipulation.ClearanceViolation;
import robotkit.manipulation.ClearanceWorld;
import robotkit.model.JointCoupling;
import MotionKitTestSupport.PlanarSolver;

private class OldProcessProof implements JointPathClearanceProof {
  public function new() {}
  public function covers(path:JointPathSamples, tolerance:Float):Bool return true;
}

/** Geometric refinement fixture: process columns must be written and certified by the caller. */
private class ProcessGeometryPlanner implements JointPathPlanner {
  final solver:KinematicsSolver;
  final rejectDrive:Bool;
  public var checkedUpdatedCurve = false;
  public function new(solver:KinematicsSolver, rejectDrive:Bool = false) {
    this.solver = solver; this.rejectDrive = rejectDrive;
  }
  public function withConfiguration(configuration:motionkit.kinematics.SixAxisConfiguration):JointPathPlanner
    throw "Fixture has no labelled branches";
  public function withSolver(worker:KinematicsSolver):JointPathPlanner return new ProcessGeometryPlanner(worker, rejectDrive);
  public function allowsFreeStart():Bool return true;
  public function retreatTarget():Null<Array<Float>> return [0.0, 0, 0, 0, 2, 6.1];
  public function clearanceWorld():Null<ClearanceWorld> return null;
  public function checkMotion(trajectory:Trajectory):Null<ClearanceViolation> return null;
  public function proveMotion(trajectory:Trajectory, velocity:Array<Float>, events:Array<ClearanceEvent>,
      depthLimit:Int):Null<TrajectoryClearanceProof> return null;
  public function checkPathClearance(curve:JointPathSamples, tolerance:Float):Bool {
    if (curve.clearanceProof != null) throw "Process columns retained an obsolete clearance proof";
    if (curve.q[curve.q.length - 1][4] <= 3) throw "Clearance did not receive the driven curve";
    checkedUpdatedCurve = true;
    if (rejectDrive) throw "Updated process motion hits the fixture";
    return true;
  }
  public function plan(path:PosePath, request:PathRequest, ?pinStart:Bool,
      ?entryCheck:(Array<Float>,Array<Float>)->Null<ClearanceViolation>,
      ?exitCheck:Array<Float>->Null<ClearanceViolation>):JointPathSamples {
    var provider = new PosePathRefinement(path);
    var q:Array<Array<Float>> = [], first:Array<Array<Float>> = [], second:Array<Array<Float>> = [], before:Array<Array<Float>> = [];
    var previous = request.startQ;
    for (i in 0...request.distances.length) {
      var task = provider.at(request.distances[i]);
      var solved = solver.solvePose(task.pose, previous, request.tolerance, task.freedom);
      if (solved == null) throw "Fixture path unreachable";
      q.push(solved); previous = solved;
      function mapped(values:Array<Float>):Array<Float> {
        var result = solver.solveDifferential(solved, new Twist6(values[0],values[1],values[2],values[3],values[4],values[5]), null, task.freedom);
        if (result == null) throw "Fixture derivatives unreachable";
        return result;
      }
      first.push(mapped(task.velocity)); second.push(mapped(task.acceleration)); before.push(mapped(task.accelerationBefore));
    }
    if (entryCheck != null && entryCheck(request.startQ, q[0]) != null) throw "Fixture entry blocked";
    if (exitCheck != null && exitCheck(q[q.length - 1]) != null) throw "Fixture exit blocked";
    var curve = new JointPathSamples(request.distances, q, first, second, before);
    curve.clearanceProof = new OldProcessProof();
    return curve;
  }
}

class DistanceDriveCompilerTests extends MotionKitTestSupport {
  static final START:Array<Float> = [0.0, 0, 0, 0, 2, 6.1];
  static function waypoint(x:Float, y:Float):PoseWaypoint return new PoseWaypoint(new Pose3(x,y), 1e-5, 1e-5);
  static function line(x0:Float,y0:Float,x1:Float,y1:Float):PoseLine
    return new PoseLine(waypoint(x0,y0),waypoint(x1,y1),OrientationPolicy.Interpolated,0.1,0.1);
  static function driven(path:PosePath, joint:Int = 4, rate:Float = 1000):PosePath
    return path.withDrive(new DistanceDrive(joint,[{start:0.0,end:path.length(),rate:rate}],path.length(),0.002));
  static function compiler(?planner:JointPathPlanner, coupled:Bool = true, maximum:Float = 1000):ProgramCompiler {
    var velocity:Array<Float> = [1.0,1,1,1,10000,30000], acceleration:Array<Float> = [10.0,10,10,10,1e7,3e7];
    var jerk:Array<Float> = [1000.0,1000,1000,1000,1e9,3e9];
    var limits = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(1));
    for (j in 0...6) { limits.velocity(j,velocity[j]); limits.acceleration(j,acceleration[j]); limits.jerk(j,jerk[j]); }
    limits.position(4,0,maximum);
    return new ProgramCompiler(new PlanarSolver(),limits,"work",velocity,acceleration,jerk,
      StartTolerances.uniform(6,1e-6,1e-6,1e-6),new ToppraPathTiming(),0.01,0.5,1e-5,1e-5,null,planner,null,
      ["x","y","unused2","unused3","e","motor"],coupled ? [new JointCoupling("process-gear","e","motor",3,0.1)] : null);
  }
  static function program(path:PosePath):MotionProgram return new MotionProgram([MotionOp.FollowPath(path,"work",0.1,[])]);
  function fails(action:Void->Void, fragment:String):Void {
    var failure = "";
    try action() catch (error:Dynamic) failure = Std.string(error);
    check(failure.indexOf(fragment) >= 0,'expected "$fragment", got "$failure"');
  }

  public function allSolversAndSections():Void {
    var path = driven(new PosePath("work",[line(0,0,0.1,0),line(0.1,0,0.1,0.1)]));
    var sections = ProgramCompiler.timingSections(path);
    var compiled = compiler().compile(program(path),START,Int64.ofInt(1));
    try {
      var plans = compiled.blocks[0].plans;
      check(plans.length == 2,"a geometric corner remains two stopped process sections");
      var origin = START[4];
      for (k in 0...plans.length) {
        var plan = plans[k], drive = sections[k].path.drive;
        if (drive == null) throw "Fixture lost its process drive";
        near(plan.evaluate(0).positions[4],origin,"each section starts at the previous extruder position",1e-9);
        for (sample in 0...101) {
          var state = plan.evaluate(plan.durationSeconds * sample / 100);
          var distance = Math.max(0.0,Math.min(drive.pathLength,state.positions[0]+state.positions[1]-sections[k].offset));
          var tolerance = 1e-6 * (1 + drive.maximumRate());
          check(Math.abs(state.positions[4]-origin-drive.offsetAt(distance)) <= tolerance,
            "generic IK's extruder follows path distance within the lowered joint/pose error budget");
          near(state.positions[5],3*state.positions[4]+0.1,"motor follows the process joint, including its offset",1e-9);
        }
        origin += drive.total();
        near(plan.evaluate(plan.durationSeconds).positions[4],origin,"the section conserves its commanded extrusion",1e-9);
      }
      near(origin,202,"both sections preserve total feed despite large authored process increments",1e-9);
    } catch (error:Dynamic) { compiled.dispose(); throw error; }
    compiled.dispose();
    fails(() -> compiler(null,true,50).compile(program(path),START,Int64.ofInt(2)),"joint limit 4");
    fails(() -> compiler().compile(program(driven(new PosePath("work",[line(0,0,0.1,0)]),5)),START,Int64.ofInt(3)),"coupling follower");
    fails(() -> compiler(null,false).compile(program(driven(new PosePath("work",[line(0,0,0.1,0)]),0,0.5)),START,Int64.ofInt(4)),"task-space tolerance");
  }

  public function refinementAndTransitions():Void {
    var planner = new ProcessGeometryPlanner(new PlanarSolver());
    var path = driven(new PosePath("work",[line(0.02,0,0.12,0)]));
    var compiled = compiler(planner,false).compile(program(path),START,Int64.ofInt(10));
    try {
      var plans = compiled.blocks[0].plans;
      check(plans.length == 3,"free path entry and retreat remain separate from deposition");
      check(planner.checkedUpdatedCurve,"clearance checks updated process columns with the old proof invalidated");
      for (k in [0,2]) for (sample in 0...21) {
        var state = plans[k].evaluate(plans[k].durationSeconds*sample/20);
        near(state.positions[4],k == 0 ? 2 : 102,"pose entry/retreat does not extrude or reset the process",1e-9);
      }
      near(plans[2].evaluate(plans[2].durationSeconds).positions[0],0,"retreat still reaches its geometric target",1e-9);
    } catch (error:Dynamic) { compiled.dispose(); throw error; }
    compiled.dispose();
    var blocked = new ProcessGeometryPlanner(new PlanarSolver(),true);
    fails(() -> compiler(blocked,false).compile(program(path),START,Int64.ofInt(11)),"Updated process motion hits");
  }

  public function failedColumnsInvalidateProof():Void {
    var curve = new JointPathSamples([0.0,0.1],[[0.0],[0.0]],[[0.0],[0.0]],[[0.0],[0.0]]);
    curve.clearanceProof = new OldProcessProof();
    var columns = new DistanceDriveColumns(new DistanceDrive(0,[{start:0.0,end:0.1,rate:1e308}],0.1,0.002),[0.0],null);
    fails(() -> columns.applyOwned(curve),"non-finite");
    check(curve.clearanceProof == null,"a failed process-column update cannot retain the old geometry proof");
  }

  public static function run():Void {
    var test = new DistanceDriveCompilerTests();
    test.allSolversAndSections(); test.refinementAndTransitions(); test.failedColumnsInvalidateProof();
  }
  public static function main():Void { run(); Sys.println("Distance-drive compiler tests passed"); }
}
