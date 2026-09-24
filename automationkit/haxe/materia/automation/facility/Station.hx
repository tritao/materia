package materia.automation.facility;

import robotkit.mobile.Pose2;

/** A named operational location in a zone. */
class Station {
  public final id:String;
  public final name:String;
  public final zoneId:String;
  public final frameId:String;
  public final pose:Pose2;

  public function new(id:String, name:String, zoneId:String, frameId:String, pose:Pose2) {
    if (id == null || id.length == 0 || name == null || name.length == 0 || zoneId == null ||
        zoneId.length == 0 || frameId == null || frameId.length == 0 || pose == null)
      throw "Station requires an ID, zone, frame, and pose";
    this.id = id;
    this.name = name;
    this.zoneId = zoneId;
    this.frameId = frameId;
    this.pose = new Pose2(pose.x, pose.y, pose.yaw);
  }
}
