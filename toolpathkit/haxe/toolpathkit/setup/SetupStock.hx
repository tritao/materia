package toolpathkit.setup;

/** Optional stock and clamp geometry for validating a placed setup. */
class SetupStock {
  public final minX:Float;
  public final maxX:Float;
  public final minY:Float;
  public final maxY:Float;
  public final top:Float;
  public final bottom:Float;
  public final safeZ:Float;
  public final fixtures:Array<Fixture>;
  /** Top plane of the sacrificial spoilboard, coincident with the stock bottom. */
  public final spoilboardPlane:Float;
  public final breakthroughAllowance:Float;

  public function new(minX:Float, maxX:Float, minY:Float, maxY:Float,
      top:Float, bottom:Float, safeZ:Float, ?fixtures:Array<Fixture>, ?spoilboardPlane:Float, breakthroughAllowance:Float = 0.0) {
    for (value in [minX, maxX, minY, maxY, top, bottom, safeZ])
      if (!Math.isFinite(value)) throw "CAM setup needs finite bounds";
    if (minX >= maxX || minY >= maxY || bottom >= top || safeZ < top)
      throw "CAM setup needs ordered stock bounds and safe Z above stock";
    var plane = spoilboardPlane == null ? bottom : spoilboardPlane;
    if (!Math.isFinite(plane) || !Math.isFinite(breakthroughAllowance) ||
        breakthroughAllowance < 0 || Math.abs(plane - bottom) > 1e-9 ||
        (breakthroughAllowance > 0 && spoilboardPlane == null))
      throw "CAM breakthrough must remain above the declared spoilboard plane";
    this.spoilboardPlane = plane;
    this.breakthroughAllowance = breakthroughAllowance;
    this.minX = minX; this.maxX = maxX;
    this.minY = minY; this.maxY = maxY;
    this.top = top; this.bottom = bottom; this.safeZ = safeZ;
    this.fixtures = fixtures == null ? [] : fixtures.copy();
    for (fixture in this.fixtures)
      if (fixture == null || safeZ <= fixture.maxZ)
        throw "CAM safe Z must clear every fixture";
  }
}
