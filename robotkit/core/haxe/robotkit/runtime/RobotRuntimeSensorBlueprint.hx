package robotkit.runtime;

import RobotKitRuntime;
import robotkit.core.ImmutableFloatArray;

/** Immutable lowering of one sensor and its semantic mount. */
class RobotRuntimeSensorBlueprint {
  public final id:String;
  public final kind:String;
  public final frameId:String;
  public final linkId:String;
  public final link:Int;
  public final mount:robotkit.core.SensorMount;
  public final position:ImmutableFloatArray;
  public final rotation:ImmutableFloatArray;
  public final updateRate:Float;
  public final rayCount:Int;
  public final maxRange:Float;
  public final startAngleRadians:Float;
  public final fieldOfViewRadians:Float;
  public final noiseStddev:Float;
  public final noiseSeed:Int;
  public final joint:Int;
  public final windowLower:Float;
  public final windowUpper:Float;
  public final hysteresis:Float;

  public function new(id:String, kind:String, frameId:String, linkId:String, link:Int,
      position:Array<Float>, rotation:Array<Float>, updateRate:Float = 0.0,
      rayCount:Int = 8, maxRange:Float = 10.0, noiseStddev:Float = 0.0, noiseSeed:Int = 1,
      startAngleRadians:Float = 0.0, fieldOfViewRadians:Float = Math.PI * 2.0,
      joint:Int = -1, windowLower:Float = 0.0, windowUpper:Float = 0.0, hysteresis:Float = 0.0) {
    this.id = id; this.kind = kind; this.frameId = frameId; this.linkId = linkId; this.link = link;
    mount = new robotkit.core.SensorMount(position, rotation);
    this.position = mount.position;
    this.rotation = mount.rotation;
    this.updateRate = updateRate; this.rayCount = rayCount; this.maxRange = maxRange;
    this.startAngleRadians = startAngleRadians; this.fieldOfViewRadians = fieldOfViewRadians;
    this.noiseStddev = noiseStddev; this.noiseSeed = noiseSeed;
    this.joint = joint; this.windowLower = windowLower; this.windowUpper = windowUpper;
    this.hysteresis = hysteresis;
  }

  /**
   * True for authored sensors whose observations come from outside the native
   * runtime (a camera driver, a GNSS receiver) and are published through
   * `RobotRuntime.publishSensorFrame` against their authored mount.
   */
  public static function isExternalKind(kind:String):Bool
    return kind == "camera" || kind == "gnss_pose" ||
      kind == "trip_switch" || kind == "tool_contact" || kind == "tool_vacuum_kpa" || kind == "tool_weld";

  /** True for sensors the native runtime samples itself. */
  public static function isNativeKind(kind:String):Bool
    return kind == "joint_encoder" || kind == "imu" || kind == "lidar" ||
      kind == "joint_switch" || kind == "at_speed" || kind == "presence";

  public var external(get, never):Bool;

  function get_external():Bool return isExternalKind(kind);

  public function nativeValue():rk_sensor_config {
    var value = new rk_sensor_config();
    value.set_kind(switch kind { case "joint_encoder": 1; case "imu": 2; case "lidar": 3;
      case "joint_switch": 4; case "at_speed": 5; case "presence": 6; default: throw "Unsupported sensor kind"; });
    value.set_link(link);
    for (i in 0...3) value.set_position(i, position.get(i));
    for (i in 0...4) value.set_rotation(i, rotation.get(i));
    value.set_update_rate(updateRate); value.set_ray_count(rayCount); value.set_max_range(maxRange);
    value.set_noise_stddev(noiseStddev); value.set_noise_seed(noiseSeed);
    value.set_start_angle(startAngleRadians); value.set_field_of_view(fieldOfViewRadians);
    return value;
  }

  public static function defaults(root:Int, linkId:String):Array<RobotRuntimeSensorBlueprint> {
    return [for (kind in ["joint_encoder", "imu", "lidar"])
      new RobotRuntimeSensorBlueprint(kind == "joint_encoder" ? "joint_encoders" : kind,
        kind, linkId, linkId, root, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])];
  }
}
