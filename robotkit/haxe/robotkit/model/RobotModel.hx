package robotkit.model;

/** Editable static definition of a robot's links, joints, and sensors. */
class RobotModel {
  public static inline var CURRENT_VERSION:Int = 3;
  public final schemaVersion:Int = CURRENT_VERSION;
  public final name:String;
  public final links:Array<Link> = [];
  public final joints:Array<Joint> = [];
  public final sensors:Array<Sensor> = [];
  public final frames:Array<Frame> = [];
  public var collisionApproximation:CollisionApproximation = CollisionApproximation.BoundsBox;
  /** Semantic mobile roles authored against stable joint IDs. */
  public var mobileBase:Null<RobotMobileConfiguration> = null;
  /** Semantic fork roles authored against stable joint IDs. */
  public var forkMechanism:Null<RobotForkConfiguration> = null;

  public function addFrame(frame:Frame):Frame {
    frames.push(frame);
    return frame;
  }

  public function new(name:String) {
    this.name = name;
  }

  public function addLink(link:Link):Link {
    links.push(link);
    return link;
  }

  public function addJoint(joint:Joint):Joint {
    joints.push(joint);
    return joint;
  }

  public function addSensor(sensor:Sensor):Sensor {
    sensors.push(sensor);
    return sensor;
  }

  /**
   * Performs the lightweight model checks used by editors.
   *
   * Runtime creation must use `RobotRuntimeCompiler.validate()` or `compile()`;
   * that pass also checks backend support and graph topology and returns
   * structured diagnostics.
   */
  public function validate():Array<String> {
    var errors:Array<String> = [];
    if (name.length == 0) errors.push("robot name is empty");
    if (links.length == 0) errors.push("robot has no links");
    if (links.length > 1024) errors.push("robot exceeds the native link limit");
    if (joints.length > 512) errors.push("robot exceeds the native joint limit");
    for (joint in joints) {
      var limitError = joint.limits.validate();
      if (limitError != null) errors.push('joint ${joint.name}: $limitError');
    }
    return errors;
  }
}
