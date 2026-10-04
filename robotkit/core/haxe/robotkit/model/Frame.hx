package robotkit.model;

/** Editable link_T_frame mount, in meters and unit xyzw quaternion. */
class Frame {
  public final id:FrameId;
  public var name:String;
  /** Owning include when this is a standard robot flange; null for other frames. */
  public var flangeIncludePath:Null<String> = null;
  public final link:Link;
  public var position:Array<Float> = [0.0, 0.0, 0.0];
  public var rotation:Array<Float> = [0.0, 0.0, 0.0, 1.0];

  public function new(name:String, link:Link, ?id:FrameId) {
    this.id = id == null ? name : id;
    this.name = name;
    this.link = link;
  }
}
