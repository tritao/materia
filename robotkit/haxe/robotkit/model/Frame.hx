package robotkit.model;

/** Named reference frame attached to a link (coincident with its origin). */
class Frame {
  public final id:FrameId;
  public var name:String;
  public final link:Link;

  public function new(name:String, link:Link, ?id:FrameId) {
    this.id = id == null ? name : id;
    this.name = name;
    this.link = link;
  }
}
