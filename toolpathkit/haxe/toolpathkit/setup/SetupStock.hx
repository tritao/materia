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

  public function new(minX:Float, maxX:Float, minY:Float, maxY:Float,
      top:Float, bottom:Float, safeZ:Float, ?fixtures:Array<Fixture>) {
    for (value in [minX, maxX, minY, maxY, top, bottom, safeZ])
      if (!Math.isFinite(value)) throw "CAM setup needs finite bounds";
    if (minX >= maxX || minY >= maxY || bottom >= top || safeZ < top)
      throw "CAM setup needs ordered stock bounds and safe Z above stock";
    this.minX = minX; this.maxX = maxX;
    this.minY = minY; this.maxY = maxY;
    this.top = top; this.bottom = bottom; this.safeZ = safeZ;
    this.fixtures = fixtures == null ? [] : fixtures.copy();
    for (fixture in this.fixtures)
      if (fixture == null || safeZ <= fixture.maxZ)
        throw "CAM safe Z must clear every fixture";
  }
}
