package robotkit.tool;

import haxe.Int64;

/**
 * In-memory `Gripper` with separate commands and contact observations.
 * Closing does not imply a grasp; a simulation sensor calls observeContact.
 */
class SimulatedGripper implements Gripper {
  public final history:Array<GripperEvent> = [];
  public final observations:Array<GripperObservation> = [];

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
    graspedState = false;
    record(timestampNs);
  }

  /** Report measured contact; only a closed gripper can hold an object. */
  public function observeContact(detected:Bool, timestampNs:Int64):Void {
    graspedState = !openState && detected;
    observations.push(new GripperObservation(detected, graspedState, timestampNs));
  }

  public function isOpen():Bool return openState;

  public function isGrasped():Bool return graspedState;

  function record(timestampNs:Int64):Void
    history.push(new GripperEvent(openState, graspedState, timestampNs));
}

/** One contact observation and the resulting grasp state. */
class GripperObservation {
  public final contact:Bool;
  public final grasped:Bool;
  public final timestampNs:Int64;

  public function new(contact:Bool, grasped:Bool, timestampNs:Int64) {
    this.contact = contact;
    this.grasped = grasped;
    this.timestampNs = timestampNs;
  }
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
