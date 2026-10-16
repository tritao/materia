package camkit.additive;

import toolpathkit.path.Point3;

/**
 * The machine the slicer writes for: where the bed is and the start and end sequences. Positions are
 * machine metres. A model is centred on `bedCenterX/Y` and sits on the bed at z = 0.
 */
class PrinterProfile {
  public final bedCenterX:Float;
  public final bedCenterY:Float;
  /** The purge line along the front of the bed, printed before the first layer. */
  public final purgeStart:Point3;
  public final purgeEnd:Point3;
  /** Height and width of the purge bead. */
  public final purgeHeight:Float;
  public final purgeWidth:Float;
  public final purgeSpeed:Float;
  /** Stationary filament prime before the purge bead, metres; zero disables it. */
  public final purgePrimeLength:Float;
  /** Where homing leaves the nozzle, and where it goes when the print is done (after lifting `parkLift`). */
  public final home:Point3;
  public final parkLift:Float;

  public function new(bedCenterX:Float, bedCenterY:Float, purgeStart:Point3, purgeEnd:Point3,
      ?purgeHeight:Float = 0.0003, ?purgeWidth:Float = 0.00045, ?purgeSpeed:Float = 0.02, ?home:Point3, ?parkLift:Float = 0.02, ?purgePrimeLength:Float = 0.0015) {
    if (!Math.isFinite(bedCenterX) || !Math.isFinite(bedCenterY)) throw "the bed centre must be finite";
    if (purgeStart == null || purgeEnd == null || purgeStart.distanceTo(purgeEnd) < 0.001)
      throw "the purge line needs two distinct ends";
    for (value in [purgeHeight, purgeWidth, purgeSpeed, parkLift])
      if (!Math.isFinite(value) || value <= 0.0) throw "profile heights, widths and speeds must be positive";
    if (!Math.isFinite(purgePrimeLength) || purgePrimeLength < 0.0) throw "purge prime must be a non-negative finite filament length";
    this.purgePrimeLength = purgePrimeLength;
    this.bedCenterX = bedCenterX;
    this.bedCenterY = bedCenterY;
    this.purgeStart = purgeStart;
    this.purgeEnd = purgeEnd;
    this.purgeHeight = purgeHeight;
    this.purgeWidth = purgeWidth;
    this.purgeSpeed = purgeSpeed;
    this.home = home == null ? new Point3(0.0, 0.0, 0.01) : home;
    this.parkLift = parkLift;
  }

  /** A 220 x 220 mm bed with the purge line 5 mm from its front edge. */
  public static function bed220():PrinterProfile
    return new PrinterProfile(0.11, 0.11, new Point3(0.03, 0.005, 0.0003), new Point3(0.11, 0.005, 0.0003));
}
