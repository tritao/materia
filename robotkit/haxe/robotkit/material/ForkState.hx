package robotkit.material;

import haxe.Int64;
import robotkit.world.RobotId;

/** Immutable fork observation derived from one RobotSnapshot. */
class ForkState {
  public final robotId:RobotId;
  public final lift:ForkAxisState;
  public final tilt:Null<ForkAxisState>;
  public final spread:Null<ForkAxisState>;
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final sourceClockId:String;
  public final receivedClockId:String;

  public function new(robotId:RobotId, lift:ForkAxisState,
      tilt:Null<ForkAxisState>, spread:Null<ForkAxisState>,
      sourceTimestampNs:Int64, receivedTimestampNs:Int64,
      sourceClockId:String, receivedClockId:String) {
    if (robotId == null || lift == null || sourceClockId == null || sourceClockId.length == 0 ||
        receivedClockId == null || receivedClockId.length == 0)
      throw "Fork state requires robot, lift, and clock identities";
    this.robotId = robotId;
    this.lift = lift;
    this.tilt = tilt;
    this.spread = spread;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs;
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
  }
}
