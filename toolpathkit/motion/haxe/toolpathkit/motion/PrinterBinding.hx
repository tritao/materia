package toolpathkit.motion;

/**
 * What lowering needs to know of a printer beyond its axes: the extruder joint and how it turns per metre of
 * filament, the hotend's melt-rate limit, and how the heaters and fans are reached. A bead of cross-section `area`
 * laid along a path takes `area / filamentArea` of filament per metre, and the extruder's gear turns
 * `1 / gearRadius` radians per metre of filament.
 */
class PrinterBinding {
  /** The robot-model joint of the filament drive gear. */
  public final extruderJoint:String;
  public final filamentDiameter:Float;
  /** The effective radius at which the drive gear grips the filament, metres. */
  public final gearRadius:Float;
  /** The hotend's greatest volumetric flow, m^3/s: a bead is laid no faster than this melts. */
  public final maxFlow:Float;
  /** How far along the path the extruder's rate ramps between runs of bead, metres. */
  public final rampLength:Float;
  /** Pressure advance: the extruder runs `pressureAdvance` seconds ahead of the flow it feeds, seconds (0 is off). */
  public final pressureAdvance:Float;
  /** The window over which the extruder's velocity is averaged for the advance, seconds: it keeps the advance within the drive's limits. */
  public final pressureAdvanceWindow:Float;
  /** The extruder joint's index in the machine, set by the `ToolpathMotionBinding` that finds it. */
  public var extruderIndex(default, null):Int = -1;

  public function new(extruderJoint:String, filamentDiameter:Float, gearRadius:Float, maxFlow:Float,
      ?rampLength:Float = 0.0005, ?pressureAdvance:Float = 0.0, ?pressureAdvanceWindow:Float = 0.04) {
    if (extruderJoint == null || extruderJoint.length == 0) throw "A printer binding names its extruder joint";
    for (value in [filamentDiameter, gearRadius, maxFlow, rampLength])
      if (!Math.isFinite(value) || value <= 0.0) throw "A printer binding needs positive filament, gear, flow and ramp sizes";
    if (!Math.isFinite(pressureAdvance) || pressureAdvance < 0.0 || pressureAdvance > 1.0)
      throw "Pressure advance is from 0 to 1 second";
    this.extruderJoint = extruderJoint; this.filamentDiameter = filamentDiameter; this.gearRadius = gearRadius;
    if (!Math.isFinite(pressureAdvanceWindow) || pressureAdvanceWindow <= 0.0) throw "The pressure-advance window must be positive";
    this.maxFlow = maxFlow; this.rampLength = rampLength; this.pressureAdvance = pressureAdvance;
    this.pressureAdvanceWindow = pressureAdvanceWindow;
  }

  public function filamentArea():Float return Math.PI * filamentDiameter * filamentDiameter / 4.0;

  /** Radians of the extruder gear per metre of path laying a bead of `area`. */
  public function radiansPerMetre(area:Float):Float return area / filamentArea() / gearRadius;

  /** Radians of the extruder gear for `length` metres of filament. */
  public function radians(length:Float):Float return length / gearRadius;

  public function bind(index:Int):Void {
    if (index < 0) throw "The extruder joint is not in the machine";
    extruderIndex = index;
  }
}
