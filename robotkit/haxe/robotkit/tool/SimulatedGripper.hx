package robotkit.tool;

import haxe.Int64;

/**
 * In-memory `Gripper` that records every commanded state change with its
 * timestamp. There is no contact physics in this simulation, so `close`
 * grasps unconditionally; a caller that wants a missed grasp should not
 * call `close` (or should model failure above this interface).
 */
class SimulatedGripper implements Gripper {
  public final history:Array<GripperEvent> = [];

  var openState = true;
  var graspedState = false;

  public function new() {}

  public function open(timestampNs:Int64):Void {
    openState = true;
    graspedState = false;
    record(timestampNs);
  }

  public function close(timestampNs:Int64):Void {
    openState = false;
    graspedState = true;
    record(timestampNs);
  }

  public function isOpen():Bool return openState;

  public function isGrasped():Bool return graspedState;

  function record(timestampNs:Int64):Void
    history.push(new GripperEvent(openState, graspedState, timestampNs));
}

/** One recorded `SimulatedGripper` state change. */
class GripperEvent {
  public final open:Bool;
  public final grasped:Bool;
  public final timestampNs:Int64;

  public function new(open:Bool, grasped:Bool, timestampNs:Int64) {
    this.open = open;
    this.grasped = grasped;
    this.timestampNs = timestampNs;
  }
}
