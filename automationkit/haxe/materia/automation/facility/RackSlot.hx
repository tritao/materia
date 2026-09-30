package materia.automation.facility;

/** Identified slot with a pose relative to its rack, and optionally the size of what it holds. */
class RackSlot {
  public final id:String;
  public final pose:RackSlotPose;
  /** Half the extents (x, y, z) of the item kept here, or null when the facility does not say. */
  public final itemHalfExtents:Null<Array<Float>>;

  public function new(id:String, pose:RackSlotPose, ?itemHalfExtents:Array<Float>) {
    if (id == null || id.length == 0 || pose == null)
      throw "Rack slot requires an ID and relative pose";
    if (itemHalfExtents != null) {
      if (itemHalfExtents.length != 3) throw "Rack slot item extents need x, y, and z";
      for (extent in itemHalfExtents)
        if (!(extent > 0.0) || !Math.isFinite(extent)) throw "Rack slot item extents must be positive and finite";
    }
    this.id = id;
    this.pose = new RackSlotPose(pose.x, pose.y, pose.z, pose.yaw);
    this.itemHalfExtents = itemHalfExtents == null ? null : itemHalfExtents.copy();
  }
}
