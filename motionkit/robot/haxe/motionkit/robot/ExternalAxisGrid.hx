package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import robotkit.manipulation.KinematicGroup;

/** Native external-cell product. Omitted axes are held at the sample's seed. */
class ExternalAxisGrid {
  public static function describe(group:KinematicGroup,seed:Array<Float>,ranges:Array<ExternalAxisRange>):mk_external_lattice {
    if (group == null || seed == null || seed.length != group.group.count() || ranges == null)
      throw "External grid requires a group, complete seed and ranges";
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
      var lower = range == null ? seed[joint] : range.lower;
      var upper = range == null ? seed[joint] : range.upper;
      var limits = group.group.limitsOf(joint);
      if (!Math.isFinite(lower) || !Math.isFinite(upper) || lower < limits.lower || upper > limits.upper)
        throw "External grid range exceeds compiled planning limits";
      native.set_joint_indices(i,joint);native.set_point_counts(i,range == null ? 1 : range.points);
      native.set_lower(i,lower);native.set_upper(i,upper);
    }
    return native;
  }
  public static function sample(group:KinematicGroup,seed:Array<Float>,ranges:Array<ExternalAxisRange>):Array<ExternalCell> {
    var native = describe(group,seed,ranges);
    var indices = [for (i in 0...group.group.count()) if (group.external[i]) i];
    var size = MotionKitNative.mk_external_lattice_count(native);
    if (size.status != TrajectoryCoreConstants.MK_OK) throw "Invalid external grid dimensions or product overflow";
    var result = MotionKitNative.mk_sample_external_cells(native,seed,size.out_count);
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
