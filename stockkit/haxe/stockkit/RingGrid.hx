package stockkit;

/** Cell-centred circumferential grid. Millimetres; angles about +Z from +X.
    Bounds are exact; cell sizes divide the bounds and never exceed requested spacing. */
class RingGrid {
  public final radiusMin:Float;
  public final radiusMax:Float;
  public final zMin:Float;
  public final zMax:Float;
  public final radialCount:Int;
  public final axialCount:Int;
  public final radialStep:Float;
  public final axialStep:Float;
  public function new(radiusMin:Float, radiusMax:Float, zMin:Float, zMax:Float,
      radialSpacing:Float, axialSpacing:Float) {
    if (!Math.isFinite(radiusMin) || !Math.isFinite(radiusMax) || radiusMin <= 0 || radiusMax <= radiusMin
      || !Math.isFinite(zMin) || !Math.isFinite(zMax) || zMax <= zMin)
      throw "ring grid needs finite increasing bounds and positive inner radius";
    if (!Math.isFinite(radialSpacing) || !Math.isFinite(axialSpacing) || radialSpacing <= 0 || axialSpacing <= 0)
      throw "ring spacing must be finite and positive";
    var nr = Math.max(1, Math.ceil((radiusMax - radiusMin) / radialSpacing - 1e-12));
    var nz = Math.max(1, Math.ceil((zMax - zMin) / axialSpacing - 1e-12));
    if (nr * nz > 10000000 || !Math.isFinite(nr * nz)) throw "ring grid exceeds ten million circles";
    this.radiusMin = radiusMin;
    this.radiusMax = radiusMax;
    this.zMin = zMin;
    this.zMax = zMax;
    radialCount = Std.int(nr);
    axialCount = Std.int(nz);
    radialStep = (radiusMax - radiusMin) / radialCount;
    axialStep = (zMax - zMin) / axialCount;
  }
  public function radius(index:Int):Float {
    if (index < 0 || index >= radialCount) throw "ring radial index out of range";
    return radiusMin + (index + 0.5) * radialStep;
  }
  public function z(index:Int):Float {
    if (index < 0 || index >= axialCount) throw "ring axial index out of range";
    return zMin + (index + 0.5) * axialStep;
  }
  /** Volume of a one-radian sector of a cell, in cubic mm. */
  public function sectorVolume(radialIndex:Int):Float return radius(radialIndex) * radialStep * axialStep;
}
