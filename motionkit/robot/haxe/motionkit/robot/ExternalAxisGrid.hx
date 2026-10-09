package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import robotkit.manipulation.KinematicGroup;

/** Native external-cell product. Omitted axes are held at the sample's seed. */
class ExternalAxisGrid {
  static inline var STOP_ROUNDOFF_ABSOLUTE:Float = 1e-9;

  /** Snap solver/representation residue at a finite external-axis stop without accepting real overtravel. */
  static function withinLimits(value:Float, joint:Int, limits:robotkit.model.JointLimits):Float {
    if (limits.lower >= limits.upper) return value;
    if (Math.isFinite(limits.lower) && value < limits.lower) {
      var tolerance = Math.max(STOP_ROUNDOFF_ABSOLUTE,
        1e-12 * Math.max(1.0, Math.max(Math.abs(value), Math.abs(limits.lower))));
      if (limits.lower - value <= tolerance) return limits.lower;
      throw 'External joint $joint value $value is below planning limit ${limits.lower}';
    }
    if (Math.isFinite(limits.upper) && value > limits.upper) {
      var tolerance = Math.max(STOP_ROUNDOFF_ABSOLUTE,
        1e-12 * Math.max(1.0, Math.max(Math.abs(value), Math.abs(limits.upper))));
      if (value - limits.upper <= tolerance) return limits.upper;
      throw 'External joint $joint value $value is above planning limit ${limits.upper}';
    }
    return value;
  }

  /** Canonical external seeds with sub-nanometre/sub-nanoradian stop residuals. */
  public static function normalizeSeed(group:KinematicGroup, seed:Array<Float>):Array<Float> {
    if (group == null || seed == null || seed.length != group.group.count())
      throw "External grid requires a group and complete seed";
    var result = seed.copy();
    for (joint in 0...result.length) {
      if (!Math.isFinite(result[joint])) throw 'External grid seed joint $joint must be finite';
      if (group.external[joint]) result[joint] = withinLimits(result[joint], joint, group.group.limitsOf(joint));
    }
    return result;
  }

  public static function describe(group:KinematicGroup,seed:Array<Float>,ranges:Array<ExternalAxisRange>):mk_external_lattice {
    if (group == null || seed == null || seed.length != group.group.count() || ranges == null)
      throw "External grid requires a group, complete seed and ranges";
    var planningSeed = normalizeSeed(group, seed);
    var indices = [for (i in 0...group.group.count()) if (group.external[i]) i];
    var byJoint = new Map<Int,ExternalAxisRange>();
    for (range in ranges) {
      if (range == null || range.joint < 0 || range.joint >= seed.length || !group.external[range.joint] || byJoint.exists(range.joint))
        throw "External grid ranges must name distinct external group joints";
      byJoint.set(range.joint,range);
    }
    var native = new mk_external_lattice();native.set_struct_size(mk_external_lattice.size());
    native.set_joint_count(seed.length);native.set_axis_count(indices.length);
    for (i in 0...indices.length) {
      var joint = indices[i],range = byJoint.get(joint);
      var lower = range == null ? planningSeed[joint] : range.lower;
      var upper = range == null ? planningSeed[joint] : range.upper;
      var limits = group.group.limitsOf(joint);
      if (!Math.isFinite(lower) || !Math.isFinite(upper))
        throw 'External grid range for joint $joint must be finite';
      if (limits.lower < limits.upper) {
        lower = withinLimits(lower, joint, limits);
        upper = withinLimits(upper, joint, limits);
      }
      if (lower > upper) throw 'External grid range for joint $joint is reversed after limit normalization';
      native.set_joint_indices(i,joint);native.set_point_counts(i,range == null ? 1 : range.points);
      native.set_lower(i,lower);native.set_upper(i,upper);
    }
    return native;
  }
  public static function sample(group:KinematicGroup,seed:Array<Float>,ranges:Array<ExternalAxisRange>):Array<ExternalCell> {
    var planningSeed = normalizeSeed(group, seed);
    var native = describe(group,planningSeed,ranges);
    var indices = [for (i in 0...group.group.count()) if (group.external[i]) i];
    var size = MotionKitNative.mk_external_lattice_count(native);
    if (size.status != TrajectoryCoreConstants.MK_OK) throw "Invalid external grid dimensions or product overflow";
    var result = MotionKitNative.mk_sample_external_cells(native,planningSeed,size.out_count);
    if (result.status != TrajectoryCoreConstants.MK_OK) throw 'External grid sampling failed: ${result.status}';
    return [for (i in 0...result.out_count) {
      var cell = result.out_cells[i];
      new ExternalCell([for (j in 0...seed.length) cell.get_joints(j)],
        [for (j in 0...indices.length) cell.get_coordinates(j)]);
    }];
  }
}
class ExternalAxisRange {
  public final joint:Int;
  public final lower:Float;
  public final upper:Float;
  public final points:Int;
  public function new(joint:Int,lower:Float,upper:Float,points:Int) {
    if (points <= 0 || !Math.isFinite(lower) || !Math.isFinite(upper) ||
        lower > upper || (points == 1 ? lower != upper : lower == upper))
      throw "External range requires finite bounds and positive nonduplicate sampling";
    this.joint=joint;this.lower=lower;this.upper=upper;this.points=points;
  }
}
class ExternalCell {
  public final q:Array<Float>;
  public final coordinates:Array<Int>;
  public function new(q:Array<Float>,coordinates:Array<Int>) { this.q=q;this.coordinates=coordinates; }
}
