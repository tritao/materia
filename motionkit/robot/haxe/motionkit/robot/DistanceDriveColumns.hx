package motionkit.robot;

import motionkit.kinematics.KinematicsSolver;
import motionkit.path.DistanceDrive;
import motionkit.path.PosePath;
import motionkit.planner.JointPathSamples;

/** The process coordinate and its followers, applied after any geometric IK/refinement. */
class DistanceDriveColumns {
  final drive:DistanceDrive;
  final origin:Float;
  final projection:Null<CouplingProjection>;
  final affected:Array<Bool>;

  public static function forPath(path:PosePath, start:Array<Float>, solver:KinematicsSolver,
      projection:Null<CouplingProjection>):Null<DistanceDriveColumns> {
    if (path.drive == null) return null;
    if (Std.isOfType(solver, AxisKinematics)) {
      var axis:AxisKinematics = cast solver;
      projection = axis.derivedProjection();
    }
    return new DistanceDriveColumns(path.drive, start, projection);
  }

  public function new(drive:DistanceDrive, start:Array<Float>, projection:Null<CouplingProjection>) {
    if (drive == null || start == null || drive.joint >= start.length || !Math.isFinite(start[drive.joint]))
      throw "Distance drive needs a joint in the current configuration";
    this.drive = drive;
    this.origin = start[drive.joint];
    this.projection = projection;
    affected = [for (_ in start) false];
    affected[drive.joint] = true;
    if (projection != null) for (follower in projection.followers) {
      if (follower == drive.joint) throw "A distance drive cannot also be a coupling follower";
      for (term in projection.termsOf(follower)) if (affected[term.leader]) affected[follower] = true;
    }
  }

  /** These authored process changes are not inverse-kinematic branch jumps. */
  public function affects(joint:Int):Bool return affected[joint];

  public function position(values:Array<Float>, distance:Float):Void
    write(values, origin + drive.offsetAt(distance), true);

  public function velocity(values:Array<Float>, distance:Float):Void
    write(values, drive.rateAt(distance), false);

  public function acceleration(values:Array<Float>, distance:Float, leaving:Bool):Void
    write(values, drive.curvatureAt(distance, leaving), false);

  /** A pose entry/retreat holds the process coordinate at its actual current position. */
  public function holdPosition(values:Array<Float>, current:Array<Float>):Void
    write(values, current[drive.joint], true);

  function write(values:Array<Float>, value:Float, positions:Bool):Void {
    if (values.length != affected.length) throw "Distance drive joint counts do not match";
    values[drive.joint] = value;
    if (projection != null) projection.complete(values, positions);
    for (item in values) if (!Math.isFinite(item)) throw "Distance drive produced a non-finite joint value";
  }

  /** Consumes a caller-owned curve. No proof for its old columns survives, including a failed update. */
  public function applyOwned(curve:JointPathSamples):Void {
    if (curve.jointCount != affected.length || curve.start() != 0 ||
        Math.abs(curve.end() - drive.pathLength) > 1e-9 * Math.max(1.0, drive.pathLength))
      throw "Distance drive curve must span its complete section";
    curve.clearanceProof = null;
    for (sample in 0...curve.s.length) {
      var distance = curve.s[sample];
      position(curve.q[sample], distance);
      velocity(curve.qPrime[sample], distance);
      acceleration(curve.qDoublePrime[sample], distance, true);
      acceleration(curve.qDoublePrimeBefore[sample], distance, false);
    }
  }
}
