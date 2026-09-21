package robotkit.model;

/** Editable static definition of a robot's links, joints, and sensors. */
class RobotModel {
  public final name:String;
  public final links:Array<Link> = [];
  public final joints:Array<Joint> = [];
  public final sensors:Array<Sensor> = [];

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
    if (joints.length > 64) errors.push("robot exceeds the native joint limit");
    for (joint in joints) {
      var limitError = joint.limits.validate();
      if (limitError != null) errors.push('joint ${joint.name}: $limitError');
    }
    return errors;
  }
}
