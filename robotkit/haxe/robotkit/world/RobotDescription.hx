package robotkit.world;

/** Transport-independent description of a logical robot. */
class RobotDescription {
  public final id:RobotId;
  public final name:String;
  public final links:Array<String>;
  public final joints:Array<String>;
  public final channels:Array<ProcessChannelDeclaration>;
  /** Joints that move with another joint, such as lead screws with their axes. */
  public final couplings:Array<CoupledJoint>;

  public function new(id:RobotId, name:String, links:Array<String>, joints:Array<String>,
      ?channels:Array<ProcessChannelDeclaration>, ?couplings:Array<CoupledJoint>) {
    this.id = id;
    this.name = name;
    this.links = links == null ?[] : links.copy();
    this.joints = joints == null ?[] : joints.copy();
    this.channels = channels == null ? [] : channels.copy();
    this.couplings = couplings == null ? [] : couplings.copy();
  }
}
