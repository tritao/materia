package camkit.additive;

/**
 * What the slicer is told, in metres and metres per second. The defaults are an ordinary 0.4 mm nozzle
 * at 0.2 mm layers; every value is checked, so a bad one fails before anything is sliced.
 */
class SlicerSettings {
  public var nozzleDiameter:Float = 0.0004;
  public var layerHeight:Float = 0.0002;
  public var firstLayerHeight:Float = 0.0002;
  /** Width of a bead, centre to edge to edge; a quarter wider than the nozzle fills its corners. */
  public var lineWidth:Float = 0.00045;
  public var perimeters:Int = 2;
  public var topLayers:Int = 4;
  public var bottomLayers:Int = 4;
  /** Fraction of the sparse infill region filled: 0 to 1. At 1 every layer is solid. */
  public var infillDensity:Float = 0.15;
  /** Rectilinear infill alternates between these angles (radians, from +x) layer by layer. */
  public var infillAngles:Array<Float> = [Math.PI / 4.0, 3.0 * Math.PI / 4.0];
  public var externalPerimeterSpeed:Float = 0.03;
  public var perimeterSpeed:Float = 0.045;
  public var infillSpeed:Float = 0.08;
  public var solidInfillSpeed:Float = 0.06;
  public var topSolidInfillSpeed:Float = 0.04;
  public var skirtSpeed:Float = 0.04;
  public var firstLayerSpeed:Float = 0.02;
  public var travelSpeed:Float = 0.15;
  /** Retraction: pull back this much filament, at this speed, before a travel longer than `retractMinTravel` that leaves the part. */
  public var retractLength:Float = 0.0008;
  public var retractSpeed:Float = 0.035;
  public var retractMinTravel:Float = 0.002;
  /** Lift the nozzle this much during retracted travels; off by default. */
  public var zHop:Float = 0.0;
  public var hotendTemperature:Float = 210.0;
  public var bedTemperature:Float = 60.0;
  /** Part-cooling fan duty from this layer on (layer indices count from 0); the first layers print without it. */
  public var fanDuty:Float = 1.0;
  public var fanFromLayer:Int = 2;
  public var skirtLoops:Int = 1;
  public var skirtDistance:Float = 0.003;
  /** Corner rounding allowed to deposited moves. */
  public var pathTolerance:Float = 0.0001;

  public function new() {}

  /** The distance between neighbouring beads that fills a layer exactly: a rounded bead of this width and height holds `height * spacing`. */
  public function beadSpacing(height:Float):Float return lineWidth - height * (1.0 - Math.PI / 4.0);

  /** Cross-section of a bead `height` tall: a rectangle with semicircular sides, `lineWidth` wide over all. */
  public function beadArea(height:Float):Float return height * (lineWidth - height) + Math.PI * height * height / 4.0;

  public function validate():Void {
    for (value in [nozzleDiameter, layerHeight, firstLayerHeight, lineWidth, externalPerimeterSpeed, perimeterSpeed, infillSpeed,
        solidInfillSpeed, topSolidInfillSpeed, skirtSpeed, firstLayerSpeed, travelSpeed, retractSpeed, hotendTemperature])
      if (!Math.isFinite(value) || value <= 0.0) throw "slicer settings need positive finite sizes, speeds and temperatures";
    for (value in [retractLength, retractMinTravel, zHop, bedTemperature, skirtDistance, pathTolerance])
      if (!Math.isFinite(value) || value < 0.0) throw "slicer settings need non-negative finite lengths and temperatures";
    if (perimeters < 0 || topLayers < 0 || bottomLayers < 0 || skirtLoops < 0 || fanFromLayer < 0)
      throw "slicer settings need non-negative counts";
    if (!(infillDensity >= 0.0 && infillDensity <= 1.0)) throw "infill density is from 0 to 1";
    if (!(fanDuty >= 0.0 && fanDuty <= 1.0)) throw "fan duty is from 0 to 1";
    if (infillAngles.length == 0) throw "infill needs at least one angle";
    if (lineWidth < layerHeight || lineWidth < firstLayerHeight) throw "a bead must be at least as wide as it is tall";
    if (lineWidth > 2.0 * nozzleDiameter) throw "line width is more than twice the nozzle";
  }
}
