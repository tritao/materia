package humanoid;

import robotkit.model.RobotModel;

/** A named pose from robotkit_mjcf_import's poses.json: an MJCF keyframe. */
class Pose {
  public final name:String;
  public final rootPosition:Array<Float>;
  public final rootRotation:Array<Float>;
  final joints:Map<String, Float>;
  final actuatorTargets:Map<String, Float>;

  public function new(name:String, rootPosition:Array<Float>, rootRotation:Array<Float>,
      joints:Map<String, Float>, actuatorTargets:Map<String, Float>) {
    this.name = name;
    this.rootPosition = rootPosition;
    this.rootRotation = rootRotation;
    this.joints = joints;
    this.actuatorTargets = actuatorTargets;
  }

  /** Joint positions in model joint order; joints the pose omits stay at zero. */
  public function jointPositions(model:RobotModel):Array<Float>
    return [for (joint in model.joints) joints.exists(joint.id) ? joints.get(joint.id) : 0.0];

  /** The pose's control for an actuator: a position servo's target. */
  public function actuatorTarget(actuatorId:String):Float
    return actuatorTargets.exists(actuatorId) ? actuatorTargets.get(actuatorId) : 0.0;

  public static function load(path:String, name:String):Pose {
    var root:Dynamic = haxe.Json.parse(sys.io.File.getContent(path));
    var poses:Array<Dynamic> = cast Reflect.field(root, "poses");
    for (pose in poses) {
      if (Reflect.field(pose, "name") != name) continue;
      return new Pose(name, floats(Reflect.field(pose, "rootPosition")),
        floats(Reflect.field(pose, "rootRotation")), values(Reflect.field(pose, "joints")),
        values(Reflect.field(pose, "actuatorTargets")));
    }
    throw 'no pose $name in $path';
  }

  static function floats(value:Dynamic):Array<Float> {
    if (value == null) return [];
    var items:Array<Dynamic> = cast value;
    return [for (item in items) (item : Float)];
  }

  static function values(value:Dynamic):Map<String, Float> {
    var result = new Map<String, Float>();
    for (field in Reflect.fields(value)) result.set(field, Reflect.field(value, field));
    return result;
  }
}
