package robotkit.model;

import robotkit.model.Transmission;

/** Editable static definition of a robot's links, joints, and sensors. */
class RobotModel {
  public static inline var CURRENT_VERSION:Int = 4;
  public final schemaVersion:Int = CURRENT_VERSION;
  public final name:String;
  public final links:Array<Link> = [];
  public final joints:Array<Joint> = [];
  /** Multiple actuators may address one joint through independent transmissions. */
  public final actuators:Array<Actuator> = [];
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

  public function addActuator(actuator:Actuator):Actuator {
    actuators.push(actuator);
    return actuator;
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
    var ids = new Map<String, Bool>();
    for (actuator in actuators) {
      if (actuator == null) {
        errors.push("robot has a null actuator");
        continue;
      }
      if (ids.exists(actuator.id)) errors.push('duplicate actuator ID ${actuator.id}');
      ids.set(actuator.id, true);
      if (!Math.isFinite(actuator.maxEffort) || actuator.maxEffort < 0.0 ||
          !Math.isFinite(actuator.maxRate) || actuator.maxRate < 0.0)
        errors.push('actuator ${actuator.id} has invalid limits');
      if (actuator.transmission == null) {
        errors.push('actuator ${actuator.id} has no transmission');
        continue;
      }
      switch actuator.transmission {
        case SimpleTransmission(jointId, ratio, offset):
          var found = false;
          for (joint in joints) if (joint.id == jointId) found = true;
          if (!found) errors.push('actuator ${actuator.id} references unknown joint $jointId');
          if (!Math.isFinite(ratio) || ratio == 0.0 || !Math.isFinite(offset))
            errors.push('actuator ${actuator.id} has an invalid transmission');
      }
    }
    return errors;
  }
}
