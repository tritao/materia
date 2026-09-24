package materia.automation.facility;

import robotkit.mobile.Footprint;

/** Named planar work area with a boundary expressed in a facility frame. */
class Zone {
  public final id:String;
  public final name:String;
  public final frameId:String;
  public final boundary:Footprint;

  public function new(id:String, name:String, frameId:String, boundary:Footprint) {
    if (id == null || id.length == 0 || name == null || name.length == 0 || frameId == null ||
        frameId.length == 0 || boundary == null)
      throw "Zone requires an ID, frame, and boundary";
    this.id = id;
    this.name = name;
    this.frameId = frameId;
    this.boundary = new Footprint(boundary.vertices());
  }
}
