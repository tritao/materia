package materia.automation.facility;

import robotkit.mobile.Pose2;

/** Storage station with identified three-dimensional slots. */
class Rack extends Station {
  final slotValues:Array<RackSlot>;

  public function new(id:String, name:String, zoneId:String, frameId:String, pose:Pose2,
      slots:Array<RackSlot>) {
    super(id, name, zoneId, frameId, pose);
    if (slots == null || slots.length == 0) throw "Rack requires at least one slot";
    slotValues = [];
    var seen = new Map<String,Bool>();
    for (slot in slots) {
      if (slot == null || seen.exists(slot.id))
        throw "Rack slot IDs must be non-empty and unique";
      seen.set(slot.id, true);
      slotValues.push(new RackSlot(slot.id, slot.pose));
    }
  }

  /** Slot IDs retained for existing facility callers. */
  public function slots():Array<String> return [for (slot in slotValues) slot.id];
  public function hasSlot(slotId:String):Bool return slot(slotId) != null;

  public function slot(slotId:String):Null<RackSlot> {
    for (slot in slotValues) if (slot.id == slotId) return slot;
    return null;
  }

  /** Slot pose in the rack's facility frame, including rack yaw. */
  public function slotPose(slotId:String):RackSlotPose {
    var found = slot(slotId);
    if (found == null) throw 'Rack has no slot "$slotId"';
    var relative = found.pose;
    var c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    return new RackSlotPose(pose.x + c * relative.x - s * relative.y,
      pose.y + s * relative.x + c * relative.y, relative.z,
      Pose2.wrapAngle(pose.yaw + relative.yaw));
  }
}
