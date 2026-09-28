package cnckit;

import toolpathkit.tool.Tool;
import toolpathkit.tool.ToolLibrary;

/** CNC machine limits and controller configuration, all in metres. */
class CncMachine {
  public final frameId:String;
  public final xAxisId:String;
  public final yAxisId:String;
  public final zAxisId:String;
  public final rapidSpeed:Float;
  public final dialect:CncDialect;
  public final maxBlendTurnAngleRadians:Float;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;
  public final initialPosition:Array<Float>;
  public final toolLibrary:ToolLibrary = new ToolLibrary();
  public final controller:CncControllerSetup;
  public var travelLower(default, null):Null<Array<Float>> = null;
  public var travelUpper(default, null):Null<Array<Float>> = null;

  public function new(frameId:String, xAxisId:String, yAxisId:String,
      zAxisId:String, rapidSpeed:Float, ?initialPosition:Array<Float>,
      ?positionTolerance:Float = 0.0005,
      ?orientationTolerance:Float = 0.02,
      ?dialect:CncDialect = LinuxCnc,
      ?maxBlendTurnAngleRadians:Float = Math.PI * 5.0 / 6.0) {
    if (frameId == null || frameId.length == 0 || xAxisId == null ||
        yAxisId == null || zAxisId == null || xAxisId.length == 0 ||
        yAxisId.length == 0 || zAxisId.length == 0 ||
        xAxisId == yAxisId || xAxisId == zAxisId || yAxisId == zAxisId)
      throw "CNC machine needs a frame and three distinct logical axes";
    if (!Math.isFinite(rapidSpeed) || rapidSpeed <= 0.0 ||
        !Math.isFinite(positionTolerance) || positionTolerance <= 0.0 ||
        !Math.isFinite(orientationTolerance) || orientationTolerance <= 0.0 ||
        !Math.isFinite(maxBlendTurnAngleRadians) ||
        maxBlendTurnAngleRadians <= 0.0 || maxBlendTurnAngleRadians >= Math.PI)
      throw "CNC machine needs positive rapid speed and tolerances";
    var initial = initialPosition == null ? [0.0, 0.0, 0.0] : initialPosition;
    if (initial.length != 3) throw "CNC machine needs three initial coordinates";
    for (value in initial) if (!Math.isFinite(value))
      throw "CNC initial coordinates must be finite";
    this.frameId = frameId;
    this.xAxisId = xAxisId;
    this.yAxisId = yAxisId;
    this.zAxisId = zAxisId;
    this.rapidSpeed = rapidSpeed;
    this.dialect = dialect;
    this.maxBlendTurnAngleRadians = maxBlendTurnAngleRadians;
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.initialPosition = initial.copy();
    controller = new CncControllerSetup(toolLibrary);
  }

  public function setTravelEnvelope(lower:Array<Float>, upper:Array<Float>):Void {
    if (lower == null || upper == null || lower.length != 3 || upper.length != 3)
      throw "CNC travel envelope needs three lower and upper coordinates";
    for (axis in 0...3)
      if (!Math.isFinite(lower[axis]) || !Math.isFinite(upper[axis]) ||
          lower[axis] >= upper[axis])
        throw "CNC travel envelope needs finite ordered bounds";
    travelLower = lower.copy();
    travelUpper = upper.copy();
  }

}
