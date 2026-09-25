package robotkit.runtime;

import RobotKitRuntime;

/**
 * Immutable-at-execution compiled robot description consumed by RobotRuntime.
 *
 * Keeping this boundary separate from the editable `robotkit.model.RobotModel`
 * model makes validation and native handle creation deterministic.
 */
class RobotRuntimeBlueprint {
  /** Semantic mappings are absent only for manually constructed native blueprints. */
  public final identity:Null<RobotRuntimeIdentity>;
  public final revision:Int;
  public final jointCount:Int;
  public final linkCount:Int;
  public final frameCount:Int;
  public final joints:Array<RobotRuntimeJointBlueprint> = [];
  public final sensors:Array<RobotRuntimeSensorBlueprint> = [];
  public final links:Array<RobotRuntimeLinkBlueprint> = [];
  public var collisionApproximation:Int = RobotKitRuntimeConstants.RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
  /** Compiled user-layer roles; null for manually assembled native blueprints. */
  public final configuration:Null<RobotRuntimeConfiguration>;

  public function sensorLayout():Array<RobotRuntimeSensorBlueprint> {
    if (sensors.length > 0) return sensors.copy();
    var children = [for (joint in joints) joint.childLink];
    var root = 0;
    for (i in 0...linkCount) if (children.indexOf(i) < 0) { root = i; break; }
    return RobotRuntimeSensorBlueprint.defaults(root, "base_link");
  }

  public function new(revision:Int, jointCount:Int, linkCount:Int,
      ?frameCount:Int = 0, ?identity:RobotRuntimeIdentity,
      ?configuration:RobotRuntimeConfiguration) {
    if (revision < 0 || jointCount < 0 || jointCount > RobotKitRuntimeConstants.RK_MAX_JOINTS ||
        linkCount < 1 || linkCount > RobotKitRuntimeConstants.RK_MAX_LINKS || frameCount < 0)
      throw "Invalid RobotKit runtime blueprint";
    this.identity = identity;
    this.revision = revision;
    this.jointCount = jointCount;
    this.linkCount = linkCount;
    this.frameCount = frameCount;
    for (_ in 0...linkCount)
      links.push(new RobotRuntimeLinkBlueprint(1.0, [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]));
    this.configuration = configuration;
  }

  public function addJoint(value:RobotRuntimeJointBlueprint):Void {
    if (joints.length >= jointCount)
      throw "RobotKit runtime blueprint has too many joints";
    joints.push(value);
  }

  @:allow(RobotRuntime, Simulation)
  function nativeValue():rk_robot_runtime_blueprint {
    if (joints.length != jointCount)
      throw "RobotKit runtime blueprint is missing joints";
    var value = new rk_robot_runtime_blueprint();
    value.set_struct_size(rk_robot_runtime_blueprint.size());
    value.set_revision(haxe.Int64.ofInt(revision));
    value.set_joint_count(jointCount);
    value.set_link_count(linkCount);
    value.set_frame_count(frameCount);
    value.set_collision_approximation(collisionApproximation);
    var layout = sensorLayout();
    if (layout.length > RobotKitRuntimeConstants.RK_MAX_SENSORS) throw "Too many sensors";
    value.set_sensor_count(layout.length);
    for (i in 0...layout.length) value.set_sensors(i, layout[i].nativeValue());
    for (index in 0...joints.length)
      value.set_joints(index, joints[index].nativeValue());
    if (links.length != linkCount) throw "RobotKit runtime blueprint is missing link physical properties";
    for (index in 0...links.length) value.set_links(index, links[index].nativeValue());
    return value;
  }

}
