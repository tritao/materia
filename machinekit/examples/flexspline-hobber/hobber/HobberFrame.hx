package hobber;

import machinekit.hobbing.*;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Machine placements: floor z=0, radial travel along +Y, spindle along +X.
 * Made parts retain their own authored coordinates; only their mounting pose
 * is expressed here. No assembly builder accepts a work-frame placement. */
class HobberFrame {
  /** Height of the authored steel/casting datum above the floor. */
  public static inline final PART_DATUM_Z = 637.0;

  /** Mount a part with radial/transverse native axes at a machine position. */
  public static function partAt(x:Float, y:Float, z:Float, ?rotation:Array<Float>):AssemblyFrame {
    var basis = F.fromRotationMatrix(0, 0, 0, [0.0,-1,0,1,0,0,0,0,1]);
    var pose = rotation == null ? basis : F.compose(basis, F.fromRotationMatrix(0,0,0,rotation));
    pose.x = x;
    pose.y = y;
    pose.z = z;
    return pose;
  }

  /** Lift a source-declared part/fixture datum into the machine frame. Source
   * hole derivation stays in the same native coordinates as the part geometry. */
  public static function sourcePose(pose:AssemblyFrame):AssemblyFrame {
    var at = F.compose(partAt(0,0,0),pose);
    at.x = -pose.y;
    at.y = pose.x;
    at.z = PART_DATUM_Z + pose.z;
    return at;
  }

  /** The sole work-to-machine conversion, for process and cutting checks. */
  public static function workToMachine():AssemblyFrame return partAt(0,0,PART_DATUM_Z);
}
