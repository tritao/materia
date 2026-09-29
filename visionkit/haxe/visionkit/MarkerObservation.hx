package visionkit;

class MarkerObservation {
  public final id:Int;
  public final corners:Array<{x:Float, y:Float}>;
  public final camera_T_marker:Pose3;
  public final rmsReprojectionError:Float;
  public final confidence:Float;
  public function new(id:Int, corners:Array<{x:Float, y:Float}>, pose:Pose3,
      rmsReprojectionError:Float, confidence:Float) {
    this.id=id; this.corners=corners; this.camera_T_marker=pose;
    this.rmsReprojectionError=rmsReprojectionError; this.confidence=confidence;
  }
}
