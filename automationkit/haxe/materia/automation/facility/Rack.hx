package materia.automation.facility;

import robotkit.mobile.Pose2;

/** Storage station with an explicit set of pallet slot identifiers. */
class Rack extends Station {
  final slotIds:Array<String>;

  public function new(id:String, name:String, zoneId:String, frameId:String, pose:Pose2,
      slots:Array<String>) {
    super(id, name, zoneId, frameId, pose);
    if (slots == null || slots.length == 0) throw "Rack requires at least one slot";
    slotIds = [];
    var seen = new Map<String,Bool>();
    for (slot in slots) {
      if (slot == null || slot.length == 0 || seen.exists(slot))
        throw "Rack slot IDs must be non-empty and unique";
      seen.set(slot, true);
      slotIds.push(slot);
    }
  }

  public function slots():Array<String> return slotIds.copy();
  public function hasSlot(slotId:String):Bool return slotIds.indexOf(slotId) >= 0;
}
