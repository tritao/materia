package robotkit.world;

import haxe.Int64;

/** Transport-independent immutable observation of one logical robot. */
class RobotSnapshot {
  public final id:RobotId;
  public final sourceSequence:Int64;
  public final timestampNs:Int64;
  public final positions:Array<Float>;
  public final velocities:Array<Float>;
  public final efforts:Array<Float>;
  public final mode:Int;
  public final faultCode:Int;

  public function new(
    id:RobotId,
    sourceSequence:Int64,
    timestampNs:Int64,
    positions:Array<Float>,
    velocities:Array<Float>,
    efforts:Array<Float>,
    mode:Int,
    faultCode:Int
  ) {
    this.id = id;
    this.sourceSequence = sourceSequence;
    this.timestampNs = timestampNs;
    this.positions = positions == null ?[] : positions.copy();
    this.velocities = velocities == null ?[] : velocities.copy();
    this.efforts = efforts == null ?[] : efforts.copy();
    this.mode = mode;
    this.faultCode = faultCode;
  }
}
