package app;

/** A document-owned, time-based position track for one simulated robot joint. */
class RobotMotionTrack {
  public final robotId:String;
  /** Joint index in the robot model; resolved from `jointId` when the track names its joint. */
  public final joint:Int;
  /** Joint name, for tracks authored against a generated model whose joint order is not fixed. */
  public final jointId:Null<String>;
  public final loop:Bool;
  public final keys:Array<{time:Float, position:Float}>;

  public function new(robotId:String, joint:Int, loop:Bool,
      keys:Array<{time:Float, position:Float}>, ?jointId:String) {
    this.robotId = robotId;
    this.joint = joint;
    this.jointId = jointId;
    this.loop = loop;
    this.keys = keys;
  }

  public function sample(seconds:Float):Float {
    var duration = keys[keys.length - 1].time;
    var time = loop && duration > 0 ? seconds % duration : Math.min(seconds, duration);
    if (time <= keys[0].time) return keys[0].position;
    for (i in 1...keys.length) if (time <= keys[i].time) {
      var left = keys[i - 1], right = keys[i];
      var fraction = (time - left.time) / (right.time - left.time);
      return left.position + fraction * (right.position - left.position);
    }
    return keys[keys.length - 1].position;
  }

  public function record():Dynamic {
    var result:Dynamic = {version:1, robotId:robotId, joint:joint,
      loop:loop, keys:[for (key in keys) {time:key.time, position:key.position}]};
    if (jointId != null) Reflect.setField(result, "jointId", jointId);
    return result;
  }

  public static function decode(raw:Dynamic):Array<RobotMotionTrack> {
    if (raw == null) return [];
    if (!Std.isOfType(raw, Array)) throw "Robot motions must be an array";
    var result:Array<RobotMotionTrack> = [];
    var seen = new Map<String, Bool>();
    for (item in (cast raw:Array<Dynamic>)) {
      fields(item, ["version", "robotId", "joint", "jointId", "loop", "keys"]);
      if (Reflect.field(item, "version") != 1) throw "Unsupported robot motion version";
      var robotId:Dynamic = Reflect.field(item, "robotId");
      var jointId:Dynamic = Reflect.field(item, "jointId");
      var joint:Dynamic = Reflect.field(item, "joint");
      if (joint == null && jointId != null) joint = 0;
      var loop:Dynamic = Reflect.field(item, "loop");
      var rawKeys:Dynamic = Reflect.field(item, "keys");
      if (!Std.isOfType(robotId, String) || robotId == "" || !Std.isOfType(joint, Int) ||
          joint < 0 || joint >= 64 || !Std.isOfType(loop, Bool) ||
          !Std.isOfType(rawKeys, Array) ||
          (jointId != null && (!Std.isOfType(jointId, String) || jointId == "")))
        throw "Invalid robot motion header";
      var identity = robotId + ":" + (jointId != null ? jointId : Std.string(joint));
      if (seen.exists(identity)) throw 'Duplicate robot motion "$identity"';
      seen.set(identity, true);
      var keys:Array<{time:Float, position:Float}> = [];
      for (rawKey in (cast rawKeys:Array<Dynamic>)) {
        fields(rawKey, ["time", "position"]);
        var rawTime:Dynamic = Reflect.field(rawKey, "time");
        var rawPosition:Dynamic = Reflect.field(rawKey, "position");
        if ((!Std.isOfType(rawTime, Int) && !Std.isOfType(rawTime, Float)) ||
            (!Std.isOfType(rawPosition, Int) && !Std.isOfType(rawPosition, Float)))
          throw "Robot motion keys need numeric time and position";
        var time:Float = cast rawTime, position:Float = cast rawPosition;
        if (
            !Math.isFinite(time) || !Math.isFinite(position) || time < 0 ||
            (keys.length == 0 && time != 0) ||
            (keys.length > 0 && time <= keys[keys.length - 1].time))
          throw "Robot motion keys need finite positions and increasing times starting at zero";
        keys.push({time:time, position:position});
      }
      if (keys.length < 2 || keys.length > 10000 || keys[keys.length - 1].time <= 0)
        throw "Robot motion needs 2..10000 keys and positive duration";
      if (loop && Math.abs(keys[0].position - keys[keys.length - 1].position) > 1e-9)
        throw "Looping robot motion must end at its starting position";
      result.push(new RobotMotionTrack(robotId, joint, loop, keys, jointId));
    }
    return result;
  }

  static function fields(value:Dynamic, allowed:Array<String>):Void {
    if (value == null || Std.isOfType(value, Array) || Std.isOfType(value, String))
      throw "Robot motion needs an object";
    for (name in Reflect.fields(value)) if (allowed.indexOf(name) < 0)
      throw 'Unknown robot motion field "$name"';
  }
}
