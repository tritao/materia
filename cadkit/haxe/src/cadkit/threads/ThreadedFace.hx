package cadkit.threads;

/** The end from which a thread proceeds along the face's native axis. */
enum ThreadStart {
  AxisMinimum;
  AxisMaximum;
}

/** Immutable manufacturing interval on a strong face name, in native axial parameters. */
class ThreadedFace {
  public final faceName:String;
  public final spec:ThreadSpec;
  public final start:Float;
  public final direction:Int;
  public final length:Null<Float>;
  /** Set when this is a cosmetic surface detail rather than a thread (the spec is then a placeholder). */
  public final detail:Null<SurfaceDetail>;

  public function new(
    faceName:String,
    spec:ThreadSpec,
    start:Float,
    direction:Int,
    length:Null<Float>,
    ?detail:SurfaceDetail
  ) {
    if (!Math.isFinite(start) || (direction != 1 && direction != -1)
      || (length != null && (!Math.isFinite(length) || length <= 0))) throw 'Invalid thread interval on $faceName';
    this.faceName = faceName;
    this.spec = spec;
    this.start = start;
    this.direction = direction;
    this.length = length;
    this.detail = detail;
  }

  /** A surface detail over an axial interval of a face, in the same terms as a thread. */
  public static function decoration(
    faceName:String,
    detail:SurfaceDetail,
    start:Float,
    direction:Int,
    length:Null<Float>
  ):ThreadedFace return new ThreadedFace(
    faceName,
    new ThreadSpec("DETAIL", 1.0, 1.0),
    start,
    direction,
    length,
    detail
  );

  public function renamed(name:String):ThreadedFace return new ThreadedFace(
    name,
    spec,
    start,
    direction,
    length,
    detail
  );
}
