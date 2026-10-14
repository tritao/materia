package robotkit.model;

class Sensor {
  public final id:SensorId;
  public var name:String;
  public final kind:String;
  public var updateRate:Float;
  /** Null mounts at the root link origin; otherwise references a model frame. */
  public var frame:Null<Frame>;
  public var rayCount:Int = 8;
  public var maxRange:Float = 10.0;
  /** A `zone_input`'s boxes in its frame: six numbers per box, its centre and half extents, in metres. */
  public var zoneBoxes:Null<Array<Float>>;
  /** Joint read by a native switch or speed sensor. */
  public var joint:Null<String>;
	/** Switch position window in SI units; for `at_speed`, lower is fractional speed tolerance. */
  public var windowLower:Float = 0.0;
  public var windowUpper:Float = 0.0;
	/** Switch position hysteresis in SI units; for `at_speed`, fractional speed hysteresis. */
  public var hysteresis:Float = 0.0;
  /** Bearing of the first LiDAR ray in the sensor frame. */
  public var startAngleRadians:Float = 0.0;
  /** Angular coverage. The default is one full revolution. */
  public var fieldOfViewRadians:Float = Math.PI * 2.0;
  public var noiseStddev:Float = 0.0;
  public var noiseSeed:Int = 1;

  public function new(name:String, kind:String, ?updateRate:Float = 0.0, ?id:SensorId) {
    // Authors can use the initial name once; imports pass the stored ID.
    this.id = id == null ? name : id;
    this.name = name;
    this.kind = kind;
    this.updateRate = updateRate;
  }

  /** Whether `boxes` holds one or more boxes of a finite centre and three positive half extents. */
  public static function validZoneBoxes(boxes:Null<Array<Float>>):Bool {
    if (boxes == null || boxes.length == 0 || boxes.length % 6 != 0) return false;
    for (index in 0...boxes.length)
      if (!Math.isFinite(boxes[index]) || (index % 6 >= 3 && !(boxes[index] > 0.0))) return false;
    return true;
  }
}
