package robotkit.work;

/** Cut/fill volume in cubic meters, e.g. from `HeightMap.volumeBetween`. */
class VolumeResult {
  public final cut:Float;
  public final fill:Float;

  public function new(cut:Float, fill:Float) {
    if (!Math.isFinite(cut) || cut < 0.0) throw "Volume result cut must be finite and non-negative";
    if (!Math.isFinite(fill) || fill < 0.0) throw "Volume result fill must be finite and non-negative";
    this.cut = cut;
    this.fill = fill;
  }

  /** Net volume: positive means net cut (material to remove), negative means net fill. */
  public function net():Float return cut - fill;
}
