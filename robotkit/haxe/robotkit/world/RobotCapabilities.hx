package robotkit.world;
/** Immutable transport-independent capabilities with bounded execution contracts. */
class RobotCapabilities {
  public final id:RobotId;
  public final jointCount:Int;
  final modes:Array<JointTargetMode>;
  final streamKinds:Array<String>;
  public var controlModes(get, never):Array<JointTargetMode>;
  public final execution:ExecutionCapabilities;
  public final timing:TimingCapabilities;
  public var streams(get, never):Array<String>;
  public function new(id:RobotId, jointCount:Int, controlModes:Array<JointTargetMode>,
      execution:ExecutionCapabilities, timing:TimingCapabilities, ?streams:Array<String>) {
    if (jointCount < 0 || controlModes == null || execution == null || timing == null)
      throw "Invalid robot capabilities";
    this.id = id;
    this.jointCount = jointCount;
    modes = controlModes.copy();
    streamKinds = streams == null ? [] : streams.copy();
    this.execution = execution;
    this.timing = timing;
  }
  function get_controlModes():Array<JointTargetMode> return modes.copy();
  function get_streams():Array<String> return streamKinds.copy();
  public function accepts(mode:JointTargetMode):Bool return modes.contains(mode);
}
