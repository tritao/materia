package robotkit.navigation;

import robotkit.mobile.Pose2;

/** Closest point on a path segment, with signed left-positive cross-track error. */
class PathProjection {
  public final distanceAlongPath:Float;
  public final segmentIndex:Int;
  public final pose:Pose2;
  public final tangentYaw:Float;
  public final crossTrackError:Float;
  public final distanceToPath:Float;

  public function new(distanceAlongPath:Float, segmentIndex:Int, pose:Pose2,
      tangentYaw:Float, crossTrackError:Float, distanceToPath:Float) {
    this.distanceAlongPath = distanceAlongPath;
    this.segmentIndex = segmentIndex;
    this.pose = pose;
    this.tangentYaw = tangentYaw;
    this.crossTrackError = crossTrackError;
    this.distanceToPath = distanceToPath;
  }
}
