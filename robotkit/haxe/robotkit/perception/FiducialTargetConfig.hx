package robotkit.perception;

import robotkit.mobile.Pose2;

/** Semantic meaning and geometry assigned to one detector marker ID. */
class FiducialTargetConfig {
  public final markerId:Int;
  public final targetKind:String;
  public final lengthMeters:Float;
  public final widthMeters:Float;
  public final heightMeters:Float;
  public final approachOffset:Null<Pose2>;

  /**
   * `targetKind` is `pallet` or `dock`. Dimensions are required for pallets;
   * docks require an approach pose expressed in the detected marker frame.
   */
  public function new(markerId:Int, targetKind:String, ?lengthMeters:Float = 0.0,
      ?widthMeters:Float = 0.0, ?heightMeters:Float = 0.0,
      ?approachOffset:Pose2) {
    if (markerId < 0 || (targetKind != "pallet" && targetKind != "dock"))
      throw "Fiducial target requires a non-negative marker ID and pallet or dock kind";
    if (targetKind == "pallet" &&
        (!Math.isFinite(lengthMeters) || lengthMeters <= 0.0 ||
         !Math.isFinite(widthMeters) || widthMeters <= 0.0 ||
         !Math.isFinite(heightMeters) || heightMeters <= 0.0))
      throw "Fiducial pallet targets require positive finite dimensions";
    if (targetKind == "dock" && approachOffset == null)
      throw "Fiducial dock targets require an approach offset";
    this.markerId = markerId;
    this.targetKind = targetKind;
    this.lengthMeters = lengthMeters;
    this.widthMeters = widthMeters;
    this.heightMeters = heightMeters;
    this.approachOffset = approachOffset == null ? null :
      new Pose2(approachOffset.x, approachOffset.y, approachOffset.yaw);
  }
}
