package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;

/** Native orientation cells, shared by geometric candidate families. */
class OrientationLattice {
  public static function describe(freedom:OrientationPolicy,rollCount:Int = 12,
      tiltRings:Int = 3,azimuthCount:Int = 8):mk_orientation_lattice {
    if (rollCount <= 0 || tiltRings <= 0 || azimuthCount <= 0)
      throw "Orientation lattice requires positive resolutions";
    var lattice = new mk_orientation_lattice(); lattice.set_struct_size(mk_orientation_lattice.size());
    lattice.set_roll_count(rollCount);lattice.set_tilt_rings(tiltRings);lattice.set_azimuth_count(azimuthCount);
    lattice.set_half_angle(0.0);
    switch freedom {
      case null | Fixed | Interpolated: lattice.set_mode(0);
      case FreeAboutTool: lattice.set_mode(1);
      case Cone(_,halfAngle): lattice.set_mode(2);lattice.set_half_angle(halfAngle);
      case Free: throw "Free orientation needs a full rotation lattice";
    }
    return lattice;
  }
  public static function sample(target:Pose3, freedom:OrientationPolicy, rollCount:Int = 12,
      tiltRings:Int = 3, azimuthCount:Int = 8):Array<OrientationCell> {
    if (target == null || rollCount <= 0 || tiltRings <= 0 || azimuthCount <= 0)
      throw "Orientation lattice requires a target and positive resolutions";
    var lattice = describe(freedom,rollCount,tiltRings,azimuthCount);
    var count = MotionKitNative.mk_orientation_lattice_count(lattice);
    if (count.status != TrajectoryCoreConstants.MK_OK)
      throw 'Invalid orientation lattice: ${count.status}';
    var pose = new mk_opw_pose();pose.set_struct_size(mk_opw_pose.size());
    var centre = ToolFreedom.of(target,freedom,0.0).target;
    var p = [centre.x,centre.y,centre.z],r = [centre.qx,centre.qy,centre.qz,centre.qw];
    for (i in 0...3) pose.set_position(i,p[i]);
    for (i in 0...4) pose.set_quaternion(i,r[i]);
    var result = MotionKitNative.mk_sample_orientations(lattice,pose,count.out_count);
    if (result.status != TrajectoryCoreConstants.MK_OK)
      throw 'Orientation lattice sampling failed: ${result.status}';
    return [for (i in 0...result.out_count) {
      var cell = result.out_samples[i];
      new OrientationCell(new Pose3(cell.get_position(0),cell.get_position(1),cell.get_position(2),
        cell.get_quaternion(0),cell.get_quaternion(1),cell.get_quaternion(2),cell.get_quaternion(3)),
        cell.get_roll_index(),cell.get_tilt_index(),cell.get_azimuth_index());
    }];
  }
}
class OrientationCell {
  public final pose:Pose3;
  public final roll:Int;
  public final tilt:Int;
  public final azimuth:Int;
  public function new(pose:Pose3,roll:Int,tilt:Int,azimuth:Int) {
    this.pose=pose;this.roll=roll;this.tilt=tilt;this.azimuth=azimuth;
  }
}
