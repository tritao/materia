package motionkit.robot;

import robotkit.core.Robot;
import robotkit.core.SensorFrame;
import robotkit.runtime.JointSwitchFrame;
import motionkit.robot.HomingDriver.HomingObservation;
import motionkit.robot.HomingDriver.HomingSwitchObservation;
import RobotKitRuntime;

/** Reads homing observations from the same immutable snapshot as the robot's joint state.
 * The translation callback establishes counter coordinates before latch and logical
 * coordinates afterward. It must apply the same translation to source positions
 * and captured edges; velocities remain unchanged. */
class RuntimeHomingObserver {
  final robot:Robot;
  final axes:Map<Int, HomingAxis> = new Map();
  final translate:Int -> Float -> Float;

  public function new(robot:Robot, axes:Array<HomingAxis>, translate:Int -> Float -> Float) {
    if (robot == null || axes == null || axes.length == 0 || translate == null)
      throw "Runtime homing observation requires robot, axes and coordinate translation";
    this.robot = robot; this.translate = translate;
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
      throw "Homing runtime has a safety fault";
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
      var translatedEdge:Null<Float> = edge == null ? null : translate(joint, edge);
      // A digital-only source has no edge identity; it uses sampled-position budgets.
      var count:Null<Int> = frame.values.length == 4 ? reading.closingEdges : null;
      signals.push(new HomingSwitchObservation(contact.id, reading.active,
        frame.sequence, frame.sourceTimestampNs, frame.sourceClockId, translatedEdge, count));
    }
    return new HomingObservation(translate(joint, snapshot.positions.get(joint)),
      snapshot.velocities.get(joint), signals, snapshot.sourceTimestampNs, snapshot.sourceClockId);
  }
}
