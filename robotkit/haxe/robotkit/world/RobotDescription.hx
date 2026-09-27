package robotkit.world;

/** Transport-independent description of a logical robot. */
class RobotDescription {
  public final id:RobotId;
  public final name:String;
  public final links:Array<String>;
  public final joints:Array<String>;
  public final channels:Array<ProcessChannelDeclaration>;

  public function new(id:RobotId, name:String, links:Array<String>, joints:Array<String>,
      ?channels:Array<ProcessChannelDeclaration>) {
    this.id = id;
    this.name = name;
    this.links = links == null ?[] : links.copy();
    this.joints = joints == null ?[] : joints.copy();
    this.channels = channels == null ? [] : channels.copy();
  }
}
