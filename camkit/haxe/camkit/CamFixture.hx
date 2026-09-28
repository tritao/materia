package camkit;

/** A rectangular clamp or other forbidden volume in work coordinates, metres. */
class CamFixture {
  public final name:String;
  public final minX:Float;
  public final maxX:Float;
  public final minY:Float;
  public final maxY:Float;
  public final minZ:Float;
  public final maxZ:Float;

  public function new(name:String, minX:Float, maxX:Float, minY:Float,
      maxY:Float, minZ:Float, maxZ:Float) {
    if (name == null || name.length == 0 || !Math.isFinite(minX) ||
        !Math.isFinite(maxX) || !Math.isFinite(minY) ||
        !Math.isFinite(maxY) || !Math.isFinite(minZ) ||
        !Math.isFinite(maxZ) || minX >= maxX || minY >= maxY ||
        minZ >= maxZ) throw "CAM fixture needs a name and ordered finite bounds";
    this.name = name;
    this.minX = minX; this.maxX = maxX;
    this.minY = minY; this.maxY = maxY;
    this.minZ = minZ; this.maxZ = maxZ;
  }
}
