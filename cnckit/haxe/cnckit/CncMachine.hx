package cnckit;

/** CNC coordinates, machine-axis binding, and stored offsets, all in metres. */
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
  final offsets:Map<Int, Array<Float>> = new Map();
  final lengths:Map<Int, Float> = new Map();
  final homes:Map<Int, Array<Float>> = new Map();

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
    for (code in 54...60) offsets.set(code, [0.0, 0.0, 0.0]);
    homes.set(28, [0.0, 0.0, 0.0]);
    homes.set(30, [0.0, 0.0, 0.0]);
  }

  public function setWorkOffset(code:Int, x:Float, y:Float, z:Float):Void {
    if (code < 54 || code > 59 || !Math.isFinite(x) || !Math.isFinite(y) ||
        !Math.isFinite(z)) throw "CNC work offset needs G54-G59 and finite XYZ";
    offsets.set(code, [x, y, z]);
  }

  public function workOffset(code:Int):Array<Float> {
    var result = offsets.get(code);
    if (result == null) throw 'Unknown CNC work offset G$code';
    return result.copy();
  }

  public function setToolLength(h:Int, length:Float):Void {
    if (h < 0 || !Math.isFinite(length))
      throw "CNC tool length needs a non-negative H and finite metres";
    lengths.set(h, length);
  }

  public function toolLength(h:Int):Float {
    var result = lengths.get(h);
    if (result == null) throw 'Unknown CNC tool length H$h';
    return result;
  }

  /** Stored G28/G30 positions are absolute machine coordinates in metres. */
  public function setHomePosition(code:Int, x:Float, y:Float, z:Float):Void {
    if ((code != 28 && code != 30) || !Math.isFinite(x) ||
        !Math.isFinite(y) || !Math.isFinite(z))
      throw "CNC home needs G28 or G30 and finite XYZ metres";
    homes.set(code, [x, y, z]);
  }

  public function homePosition(code:Int):Array<Float> {
    var result = homes.get(code);
    if (result == null) throw 'Unknown CNC home G$code';
    return result.copy();
  }
}
