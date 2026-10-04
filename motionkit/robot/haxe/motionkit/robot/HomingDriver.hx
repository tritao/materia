package motionkit.robot;

/** The runtime boundary used by homing; all commands are classified as homing motion. */
interface HomingDriver {
  function observe(joint:Int):HomingObservation;
  function velocity(joint:Int, velocity:Float, acceleration:Float):Void;
  function stop(joint:Int, acceleration:Float):Void;
  /** Establish the runtime zero and reset encoder/slip monitors at the captured latch. */
  function latch(switchId:String, counterPosition:Float):Void;
  function returnHome(joint:Int, position:Float, velocity:Float, acceleration:Float):Void;
}

/** One fresh digital home signal with an optional captured closing-edge counter coordinate. */
class HomingSwitchObservation {
  public final id:String;
  public final active:Bool;
  public final edgePosition:Null<Float>;
  public function new(id:String, active:Bool, edgePosition:Null<Float> = null) {
    if (id == null || id.length == 0 || (edgePosition != null && !Math.isFinite(edgePosition)))
      throw "Home signal requires an ID and finite edge coordinate";
    this.id = id; this.active = active; this.edgePosition = edgePosition;
  }
}

/** Counter position before latch, calibrated logical position after latch. */
class HomingObservation {
  public final position:Float;
  public final velocity:Float;
  public final switches:Array<HomingSwitchObservation>;
  public function new(position:Float, velocity:Float, switches:Array<HomingSwitchObservation>) {
    if (!Math.isFinite(position) || !Math.isFinite(velocity) || switches == null)
      throw "Homing observation requires finite position/velocity and switch readings";
    this.position = position; this.velocity = velocity; this.switches = switches.copy();
  }
}
