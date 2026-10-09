package toolpathkit.motion;

import toolpathkit.setup.TravelEnvelope;

/** Machine axes and execution limits, independent of a G-code controller. */
class MachineBinding {
  public final channels:ToolpathChannels;
  public final frameId:String;
  public final xAxisId:String;
  public final yAxisId:String;
  public final zAxisId:String;
  public final rapidSpeed:Float;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;
  public final maxBlendTurnAngleRadians:Float;
  public var travel(default, null):Null<TravelEnvelope> = null;
  /**
    Whether `Rapid`, `Link` and `Retract` moves run at their own feed (bounded by `rapidSpeed`) and round
    their corners within their tolerance, as a printer's travels do, instead of at `rapidSpeed` with
    exact stops, as a mill's rapids do.
  **/
  public var rapidsAtFeed(default, null):Bool = false;
  /** The extruder and heaters, when the machine prints. */
  public var printer(default, null):Null<PrinterBinding> = null;

  public function new(frameId:String, xAxisId:String, yAxisId:String,
      zAxisId:String, rapidSpeed:Float, channels:ToolpathChannels,
      positionTolerance:Float = 0.0005,
      orientationTolerance:Float = 0.02,
      maxBlendTurnAngleRadians:Float = Math.PI * 5.0 / 6.0) {
    if (frameId == null || frameId.length == 0 || xAxisId == null ||
        yAxisId == null || zAxisId == null || xAxisId.length == 0 ||
        yAxisId.length == 0 || zAxisId.length == 0 ||
        xAxisId == yAxisId || xAxisId == zAxisId || yAxisId == zAxisId)
      throw "machine binding needs a frame and three distinct axes";
    if (!Math.isFinite(rapidSpeed) || rapidSpeed <= 0.0 ||
        !Math.isFinite(positionTolerance) || positionTolerance <= 0.0 ||
        !Math.isFinite(orientationTolerance) || orientationTolerance <= 0.0 ||
        !Math.isFinite(maxBlendTurnAngleRadians) ||
        maxBlendTurnAngleRadians <= 0.0 || maxBlendTurnAngleRadians >= Math.PI)
      throw "machine binding needs positive speed and tolerances";
    if (channels == null) throw "machine binding needs its controller channels";
    this.channels = channels;
    this.frameId = frameId; this.xAxisId = xAxisId; this.yAxisId = yAxisId;
    this.zAxisId = zAxisId; this.rapidSpeed = rapidSpeed;
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.maxBlendTurnAngleRadians = maxBlendTurnAngleRadians;
  }

  public function setTravel(value:Null<TravelEnvelope>):Void travel = value;

  public function setRapidsAtFeed(value:Bool):Void rapidsAtFeed = value;

  public function setPrinter(value:Null<PrinterBinding>):Void printer = value;
}
