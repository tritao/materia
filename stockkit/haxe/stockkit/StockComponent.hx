package stockkit;

/** One material component on the dexel lattice. Areas/volumes are sampled,
 * just as Stock.volume is; supportArea is material crossing supportPlaneZ.
 */
class StockComponent {
  public final seedRay:Int;
  public final seedZ:Float;
  public final supportArea:Float;
  public final volume:Float;
  public final min:Array<Float>;
  public final max:Array<Float>;
  public final intervalSamples:Int;
  public final supportByRegion:Array<Float>;
  public function new(
    seedRay:Int,
    seedZ:Float,
    supportArea:Float,
    volume:Float,
    min:Array<Float>,
    max:Array<Float>,
    samples:Int,
    regions:Array<Float>
  ) {
    this.seedRay = seedRay;
    this.seedZ = seedZ;
    this.supportArea = supportArea;
    this.volume = volume;
    this.min = min;
    this.max = max;
    intervalSamples = samples;
    supportByRegion = regions;
  }
}
