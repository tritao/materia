package motionkit.robot;

import haxe.Int64;

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
  public final sequence:Int64;
  public final timestampNs:Int64;
  public final clockId:String;
  public final closingEdges:Null<Int>;
  public function new(id:String, active:Bool, sequence:Int64, timestampNs:Int64,
      clockId:String, edgePosition:Null<Float> = null, closingEdges:Null<Int> = null) {
    if (id == null || id.length == 0 || (edgePosition != null && !Math.isFinite(edgePosition)))
      throw "Home signal requires an ID and finite edge coordinate";
    if (sequence == null || timestampNs == null || Int64.compare(sequence, Int64.ofInt(0)) <= 0 ||
        Int64.compare(timestampNs, Int64.ofInt(0)) < 0 || clockId == null || clockId.length == 0 ||
        (closingEdges != null && closingEdges < 0) ||
        (edgePosition != null && (closingEdges == null || closingEdges == 0)))
      throw "Home signal requires source freshness and captured-edge identity";
    this.id = id; this.active = active; this.edgePosition = edgePosition;
    this.sequence = sequence; this.timestampNs = timestampNs; this.clockId = clockId;
    this.closingEdges = closingEdges;
  }
}

/** Counter position before latch, calibrated logical position after latch. */
class HomingObservation {
  public final position:Float;
  public final velocity:Float;
  public final switches:Array<HomingSwitchObservation>;
  public final timestampNs:Int64;
  public final clockId:String;
  public function new(position:Float, velocity:Float, switches:Array<HomingSwitchObservation>,
      timestampNs:Int64, clockId:String) {
    if (!Math.isFinite(position) || !Math.isFinite(velocity) || switches == null)
      throw "Homing observation requires finite position/velocity and switch readings";
    if (timestampNs == null || Int64.compare(timestampNs, Int64.ofInt(0)) < 0 ||
        clockId == null || clockId.length == 0) throw "Homing position requires a source clock";
    var ids = new Map<String, Bool>();
    for (signal in switches) {
      if (signal == null || ids.exists(signal.id)) throw "Homing switch observations must be distinct";
      ids.set(signal.id, true);
    }
    this.position = position; this.velocity = velocity; this.switches = switches.copy();
    this.timestampNs = timestampNs; this.clockId = clockId;
  }
}
