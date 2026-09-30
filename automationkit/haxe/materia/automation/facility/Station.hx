package materia.automation.facility;

import robotkit.mobile.Pose2;

/**
 * A named operational location in a zone. A station's pose is where a person stands to work at it; the
 * optional surface is the top they work over, placed relative to that pose.
 */
class Station {
  public final id:String;
  public final name:String;
  public final zoneId:String;
  public final frameId:String;
  public final pose:Pose2;
  /** The work top at this station, or null when the facility does not describe one. */
  public final surface:Null<Surface>;

  public function new(id:String, name:String, zoneId:String, frameId:String, pose:Pose2, ?surface:Surface) {
    if (id == null || id.length == 0 || name == null || name.length == 0 || zoneId == null ||
        zoneId.length == 0 || frameId == null || frameId.length == 0 || pose == null)
      throw "Station requires an ID, zone, frame, and pose";
    this.id = id;
    this.name = name;
    this.zoneId = zoneId;
    this.frameId = frameId;
    this.pose = new Pose2(pose.x, pose.y, pose.yaw);
    this.surface = surface;
  }
}
