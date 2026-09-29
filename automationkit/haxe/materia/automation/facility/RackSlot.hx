package materia.automation.facility;

/** Identified slot with a pose relative to its rack. */
class RackSlot {
  public final id:String;
  public final pose:RackSlotPose;

  public function new(id:String, pose:RackSlotPose) {
    if (id == null || id.length == 0 || pose == null)
      throw "Rack slot requires an ID and relative pose";
    this.id = id;
    this.pose = new RackSlotPose(pose.x, pose.y, pose.z, pose.yaw);
  }
}
