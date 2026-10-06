package motionkit.robot;

import motionkit.kinematics.SixAxisConfiguration;

import MotionKitNative;
import TrajectoryCore;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import robotkit.manipulation.KinematicGroup;

/** One native Cartesian orientation/branch/lift call, with model-derived limits. */
class CartesianCandidateSampler {
  public final analytic:CartesianAnalyticIk;
  final group:KinematicGroup;
  final model:mk_serial_cell_model;
  final external:mk_external_lattice;
  final limits:mk_joint_lift_request;

  public function new(group:KinematicGroup) {
    this.group=group;
    analytic=new CartesianAnalyticIk(group);
    // The verified Cartesian descriptor includes every joint, including
    // leading axes marked external by the group classification.
    var count=group.group.count();
    model=new mk_serial_cell_model();model.set_struct_size(mk_serial_cell_model.size());
    model.set_joint_count(count);model.set_arm_joint_count(count);model.set_external_count(0);
    model.set_base_quaternion(3,1);model.set_work_quaternion(3,1);model.set_tool_quaternion(3,1);
    external=new mk_external_lattice();external.set_struct_size(mk_external_lattice.size());
    external.set_joint_count(count);external.set_axis_count(0);
    limits=new mk_joint_lift_request();limits.set_struct_size(mk_joint_lift_request.size());limits.set_joint_count(count);
    for (i in 0...count) {
      model.set_arm_joint_indices(i,i);
      var bound=group.group.limitsOf(i);
      limits.set_lower(i,bound.lower);limits.set_upper(i,bound.upper);limits.set_periodic(i,i>=3 ? 1 : 0);
    }
  }
  public function sample(target:Pose3,seed:Array<Float>,freedom:OrientationPolicy,
      rollCount:Int=12,tiltRings:Int=3,azimuthCount:Int=8):Array<LatticeCandidate> {
    if (target==null || seed==null || seed.length!=group.group.count())
      throw "Cartesian candidate sampling requires a target and complete seed";
    var orientation=OrientationLattice.describe(freedom,rollCount,tiltRings,azimuthCount);
    var centre=OrientationLattice.centre(target,freedom);
    var pose=new mk_analytic_pose();pose.set_struct_size(mk_analytic_pose.size());
    var p=[centre.x,centre.y,centre.z],r=[centre.qx,centre.qy,centre.qz,centre.qw];
    for (i in 0...3)pose.set_position(i,p[i]);for (i in 0...4)pose.set_quaternion(i,r[i]);
    var size=MotionKitNative.mk_cartesian_candidate_count(analytic.nativeModel(),model,external,orientation,limits,pose,seed);
    if (size.status!=TrajectoryCoreConstants.MK_OK)throw 'Native Cartesian candidate count failed: ${size.status}';
    if (size.out_count==0)return [];
    var lengths=compactLengths(size.out_count,seed.length,0);
    var result=MotionKitNative.mk_sample_cartesian_candidates_compact(analytic.nativeModel(),model,external,orientation,limits,pose,seed,lengths.joints,lengths.coordinates);
    if (result.status!=TrajectoryCoreConstants.MK_OK)throw 'Native Cartesian candidate sampling failed: ${result.status}';
    return decodeCompact(result.out_joints,result.out_wraps,result.out_coordinates,result.out_count,seed.length,0);
  }
  public static function compactLengths(count:Int,joints:Int,externals:Int):{joints:Int,coordinates:Int} {
    if(count<0 || joints<1 || joints>64 || externals<0 || externals>joints ||
        count>Std.int(268435456/Math.max(joints*8,(externals+5)*4)))
      throw "Native candidate layer exceeds compact FFI array dimensions; streaming sampling is required";
    return {joints:count*joints,coordinates:count*(externals+5)};
  }
  public static function decodeCompact(joints:Array<Float>,wraps:Array<Int>,coordinates:Array<Int>,
      count:Int,jointCount:Int,externalCount:Int):Array<LatticeCandidate> {
    return [for(i in 0...count){
      var jointOffset=i*jointCount,cellOffset=i*(externalCount+5);
      new LatticeCandidate([for(j in 0...jointCount)joints[jointOffset+j]],
        [for(j in 0...jointCount)wraps[jointOffset+j]],
        [for(j in 0...externalCount)coordinates[cellOffset+j]],
        coordinates[cellOffset+externalCount],coordinates[cellOffset+externalCount+1],
        coordinates[cellOffset+externalCount+2],coordinates[cellOffset+externalCount+3],
        coordinates[cellOffset+externalCount+4]);
    }];
  }
}
class LatticeCandidate {
  public final q:Array<Float>;
  public final wraps:Array<Int>;
  public final external:Array<Int>;
  public final roll:Int;
  public final tilt:Int;
  public final azimuth:Int;
  public final branch:Int;
  public final singular:Int;
  public final singularityKnown:Bool;
  public var configuration:Null<SixAxisConfiguration>;
  public function new(q:Array<Float>,wraps:Array<Int>,external:Array<Int>,roll:Int,tilt:Int,azimuth:Int,branch:Int,singular:Int,singularityKnown:Bool=true,?configuration:SixAxisConfiguration) {
    this.q=q;this.wraps=wraps;this.external=external;this.roll=roll;this.tilt=tilt;this.azimuth=azimuth;
    this.branch=branch;this.singular=singular;this.singularityKnown=singularityKnown;this.configuration=configuration;
  }
}
