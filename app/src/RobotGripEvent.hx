package app;

/**
 * A vacuum command at a simulation time: the tool link grips the object it is touching, or lets go.
 * A project lists these beside its joint motion; they repeat with the motion's loop.
 */
class RobotGripEvent {
  public final time:Float;
  /** The assembly link that carries the gripper, such as the suction cup. */
  public final link:String;
  public final grip:Bool;

  public function new(time:Float, link:String, grip:Bool) {
    this.time = time;
    this.link = link;
    this.grip = grip;
  }

  /**
   * Reads `[{"time": 1.4, "link": "tool/cup", "action": "grip"}, ...]`. Times never decrease, and
   * each link alternates grip and release and ends released, so a repeating cycle starts free.
   */
  public static function decode(raw:Dynamic):Array<RobotGripEvent> {
    if (raw == null) return [];
    if (!Std.isOfType(raw, Array)) throw "Robot grips must be an array";
    var result:Array<RobotGripEvent> = [];
    var holding = new Map<String, Bool>();
    var previous = 0.0;
    for (item in (cast raw:Array<Dynamic>)) {
      for (name in Reflect.fields(item))
        if (name != "time" && name != "link" && name != "action") throw 'Unknown robot grip field "$name"';
      var time:Dynamic = Reflect.field(item, "time");
      var link:Dynamic = Reflect.field(item, "link");
      var action:Dynamic = Reflect.field(item, "action");
      if ((!Std.isOfType(time, Int) && !Std.isOfType(time, Float)) || !Std.isOfType(link, String) || link == "" ||
          (action != "grip" && action != "release"))
        throw "A robot grip needs a time, a link and an action of grip or release";
      var at:Float = cast time;
      if (!Math.isFinite(at) || at < previous) throw "Robot grips need finite times that never decrease";
      previous = at;
      var name:String = cast link;
      var grip = action == "grip";
      if (grip == holding.exists(name)) throw 'Robot grip events for "$name" must alternate grip and release';
      if (grip) holding.set(name, true);
      else holding.remove(name);
      result.push(new RobotGripEvent(at, name, grip));
    }
    for (name in holding.keys()) throw 'Robot grip events for "$name" must end with a release';
    return result;
  }

  public function record():Dynamic return {time: time, link: link, action: grip ? "grip" : "release"};
}
