package cnckit;

/**
  What a printer controller knows that a G-code file does not say: the filament, which turns an `E`
  distance into the cross-section laid along the path, and the firmware's own retract. Lengths are in
  metres and speeds in metres per second.
**/
class PrinterSettings {
  /** Filament diameter; 1.75 mm is the usual size. */
  public final filamentDiameter:Float;
  /** The firmware retract `G10` pulls back and `G11` pushes forward again. */
  public final retractLength:Float;
  public final retractSpeed:Float;
  /** Extra filament `G11` pushes, which `G10` did not pull. */
  public final unretractExtraLength:Float;
  public final unretractSpeed:Float;
  /**
    Change of the extrusion-per-distance ratio, relative to the bead in force, that starts a new
    `Deposit`. Slicers round `E` to a few decimals, so exact comparison would restart it constantly.
  **/
  public final ratioTolerance:Float;
  /** Corner rounding allowed to deposited moves; the file says nothing of it, a slicer or the user does. */
  public final pathTolerance:Float;
  /** Layer height assumed before a file says one. */
  public final defaultLayerHeight:Float;
  /** May the controller draw `G2`/`G3` arcs? Otherwise the writer draws them as short lines. */
  public final arcs:Bool;

  public function new(?filamentDiameter:Float = 0.00175, ?retractLength:Float = 0.0008, ?retractSpeed:Float = 0.035,
      ?unretractExtraLength:Float = 0.0, ?unretractSpeed:Float = 0.035, ?ratioTolerance:Float = 0.005,
      ?pathTolerance:Float = 0.0001, ?defaultLayerHeight:Float = 0.0002, ?arcs:Bool = true) {
    for (value in [filamentDiameter, retractLength, retractSpeed, unretractSpeed, defaultLayerHeight])
      if (!Math.isFinite(value) || value <= 0.0) throw "printer settings need positive finite lengths and speeds";
    for (value in [unretractExtraLength, ratioTolerance, pathTolerance])
      if (!Math.isFinite(value) || value < 0.0) throw "printer settings need non-negative finite tolerances";
    this.filamentDiameter = filamentDiameter;
    this.retractLength = retractLength;
    this.retractSpeed = retractSpeed;
    this.unretractExtraLength = unretractExtraLength;
    this.unretractSpeed = unretractSpeed;
    this.ratioTolerance = ratioTolerance;
    this.pathTolerance = pathTolerance;
    this.defaultLayerHeight = defaultLayerHeight;
    this.arcs = arcs;
  }

  /** Cross-section of the filament, in m^2. */
  public function filamentArea():Float return toolpathkit.path.BeadSection.filamentArea(filamentDiameter);
}
