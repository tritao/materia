package motionkit.robot;

import robotkit.core.Robot;
import robotkit.core.SensorFrame;
import robotkit.runtime.JointSwitchFrame;
import motionkit.robot.HomingDriver.HomingObservation;
import motionkit.robot.HomingDriver.HomingSwitchObservation;
import RobotKitRuntime;

/** Reads homing observations from the same immutable snapshot as the robot's joint state.
 * Position snapshots use runtime logical coordinates, while switch captures use
 * physical endpoint coordinates. Explicit translations handle those two source
 * domains separately; velocities remain unchanged. */
class RuntimeHomingObserver {
  final robot:Robot;
  final axes:Map<Int, HomingAxis> = new Map();
  final translatePosition:Int -> Float -> Float;
  final translateEdge:Int -> Float -> Float;

  public function new(robot:Robot, axes:Array<HomingAxis>, translatePosition:Int -> Float -> Float,
      translateEdge:Int -> Float -> Float) {
    if (robot == null || axes == null || axes.length == 0 || translatePosition == null || translateEdge == null)
      throw "Runtime homing observation requires robot, axes and coordinate translation";
    this.robot = robot; this.translatePosition = translatePosition; this.translateEdge = translateEdge;
    var ids = new Map<String, Bool>();
    for (axis in axes) {
      if (axis == null || this.axes.exists(axis.joint)) throw "Duplicate or null homing axis";
      this.axes.set(axis.joint, axis);
      for (contact in axis.switches) {
        if (ids.exists(contact.id)) throw "A homing switch belongs to multiple axes";
        ids.set(contact.id, true);
      }
    }
  }

  public function observe(joint:Int):HomingObservation {
    var axis = axes.get(joint);
    if (axis == null) throw "Homing observation requested an unknown joint";
    var snapshot = robot.snapshot();
    if (snapshot == null || joint < 0 || joint >= snapshot.positions.length ||
        snapshot.velocities.length != snapshot.positions.length)
      throw "Homing runtime returned an invalid joint observation";
    if (snapshot.faultCode != 0 || snapshot.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT ||
        snapshot.safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP)
      throw 'Homing runtime has a safety fault ${snapshot.faultCode} on joint $joint';
    var frames = new Map<String, SensorFrame>();
    for (frame in snapshot.sensors.toArray()) {
      if (frame == null || frames.exists(frame.sensorId)) throw "Homing snapshot has duplicate or null sensor frames";
      frames.set(frame.sensorId, frame);
    }
    var signals:Array<HomingSwitchObservation> = [];
    for (contact in axis.switches) {
      var frame = frames.get(contact.id);
      if (frame == null || frame.frameId != contact.frameId)
        throw 'Homing switch "${contact.id}" has no matching mounted frame';
      var reading = new JointSwitchFrame(frame);
      var edge = reading.closingEdgePosition;
      var translatedEdge:Null<Float> = edge == null ? null : translateEdge(joint, edge);
      // A digital-only source has no edge identity; it uses sampled-position budgets.
      var count:Null<Int> = frame.values.length == 4 ? reading.closingEdges : null;
      signals.push(new HomingSwitchObservation(contact.id, reading.active,
        frame.sequence, frame.sourceTimestampNs, frame.sourceClockId, translatedEdge, count, reading.capturesEdges));
    }
    var ready = snapshot.sessionState == RobotKitRuntimeConstants.RK_SESSION_IDLE &&
      snapshot.trajectoryQueueDepth == 0 && !snapshot.trajectoryActive;
    for (velocity in snapshot.velocities.toArray()) if (Math.abs(velocity) > 1e-6) ready = false;
    return new HomingObservation(translatePosition(joint, snapshot.positions.get(joint)),
      snapshot.velocities.get(joint), signals, snapshot.sourceTimestampNs, snapshot.sourceClockId, ready);
  }
}
