package robotkit.tool;

/**
 * The weld metal laid along one straight seam, as the wire actually deposited it. It is told every step whether the arc
 * is established, how fast the wire feeds and where the wire tip is, and keeps the metal deposited at each station along
 * the seam. Nothing here is a plan: the bead is what the torch did, so a stop, a slow-down, a restart's overlap or an
 * arc that went out show in it.
 *
 * **Deposition.** The wire melts into the joint as it is fed: in `dt` seconds the arc melts `wireSpeed · A · dt` of wire
 * (`A` the wire's cross-section), and `depositionEfficiency` of it reaches the bead, the rest being spatter and fume. The
 * efficiency is a property of the wire and its gas, given with the wire (the scene's torch block).
 * That metal goes to the station of the seam the wire tip is over. Moving at travel speed `v`, a station receives the
 * metal of the time the tip is over it, so the cross-section is the deposition rate over travel speed:
 * `area = wireSpeed · A · efficiency / v`. Standing still piles it into one station, as a crater does.
 *
 * **Section.** A fillet's section is a triangle against the two faces, an equal-leg one here: legs of
 * `sqrt(2 · area)`, a straight face (a concave or convex profile would shift the leg for the same area, by a few
 * percent). `leg(i)` is that leg at station `i`, and the achieved leg of a weld is a result compared with the leg the
 * weldment asked for.
 *
 * The seam is cut into stations of `BIN` metres. The tip counts as welding it when it is within `REACH` of the seam
 * line and not more than `OVERRUN` past either end; metal deposited while it is elsewhere (an arc held in the air above
 * the work, say) is `stray`. Lengths are metres, wire speed metres per minute, the wire's diameter millimetres.
 */
class WeldBead {
  /** Length of one station along the seam, in metres. */
  public static inline var BIN:Float = 0.001;
  /** How far from the seam line the wire tip may be and still deposit into the seam, in metres. */
  public static inline var REACH:Float = 0.008;
  /** How far past an end of the seam the tip may be and still deposit into its end station, in metres. */
  public static inline var OVERRUN:Float = 0.004;

  public final start:Array<Float>;
  public final stop:Array<Float>;
  public final normalA:Array<Float>;
  public final normalB:Array<Float>;
  /** The seam's direction, a unit vector from `start` to `stop`. */
  public final tangent:Array<Float>;
  /** Direction along face A, away from the corner into the open side: the way the leg on face A runs. */
  public final legA:Array<Float>;
  /** Direction along face B, away from the corner. */
  public final legB:Array<Float>;
  public final length:Float;
  public final wireArea:Float;
  /** Fraction of the melted wire that ends up in the bead. */
  public final depositionEfficiency:Float;
  /** Number of stations. */
  public final count:Int;
  /** Metal deposited outside the seam, in cubic metres. */
  public var stray(default, null):Float = 0.0;
  /** Metal in the bead, in cubic metres. */
  public var deposited(default, null):Float = 0.0;

  final volume:Array<Float>;
  /** The deposition episode (one arc run) that last touched each station, and how many distinct ones did. */
  final lastEpisode:Array<Int>;
  final episodes:Array<Int>;
  var episode:Int = 0;
  var burning:Bool = false;
  var changedFrom:Int;
  var changedTo:Int = -1;

  /**
   * `start` and `stop` bound the seam and `normalA`, `normalB` are the faces' outward unit normals, all in one frame.
   * `wireDiameterMm` is the wire's diameter and `depositionEfficiency` the fraction of its melted metal that reaches the bead.
   */
  public function new(start:Array<Float>, stop:Array<Float>, normalA:Array<Float>, normalB:Array<Float>, wireDiameterMm:Float,
      depositionEfficiency:Float) {
    if (start == null || stop == null || normalA == null || normalB == null || start.length != 3 || stop.length != 3 ||
        normalA.length != 3 || normalB.length != 3 || !(wireDiameterMm > 0) ||
        !(depositionEfficiency > 0 && depositionEfficiency <= 1))
      throw "A weld bead needs a seam, the faces' normals, a wire diameter and a deposition efficiency in (0, 1]";
    this.depositionEfficiency = depositionEfficiency;
    var along = [for (axis in 0...3) stop[axis] - start[axis]];
    var seamLength = norm(along);
    if (!(seamLength > 1e-6)) throw "A weld bead needs a seam with a length";
    this.start = start.copy();
    this.stop = stop.copy();
    this.length = seamLength;
    this.tangent = [for (axis in 0...3) along[axis] / seamLength];
    this.normalA = unit(normalA);
    this.normalB = unit(normalB);
    this.legA = unit(minus(this.normalB, scale(this.normalA, dot(this.normalB, this.normalA))));
    this.legB = unit(minus(this.normalA, scale(this.normalB, dot(this.normalA, this.normalB))));
    this.wireArea = Math.PI * wireDiameterMm * wireDiameterMm / 4.0 * 1.0e-6;
    this.count = Std.int(Math.ceil(seamLength / BIN - 1e-9));
    this.volume = [for (_ in 0...count) 0.0];
    this.lastEpisode = [for (_ in 0...count) 0];
    this.episodes = [for (_ in 0...count) 0];
    this.changedFrom = count;
  }

  /** Forgets all the metal, as a fresh workpiece does. */
  public function reset():Void {
    for (i in 0...count) {
      volume[i] = 0.0;
      lastEpisode[i] = 0;
      episodes[i] = 0;
    }
    stray = 0.0;
    deposited = 0.0;
    episode = 0;
    burning = false;
    changedFrom = 0;
    changedTo = count - 1;
  }

  /** Advances `dt` seconds with the arc `established` (or not), the wire fed at `wireSpeed` m/min, its tip at `tip`. */
  public function step(dt:Float, established:Bool, wireSpeed:Float, tip:Array<Float>):Void {
    if (!(dt >= 0)) throw "A weld bead step needs a non-negative time step";
    if (!established) {
      burning = false;
      return;
    }
    if (!burning) {
      burning = true;
      episode++;
    }
    var melted = Math.max(0.0, wireSpeed) / 60.0 * wireArea * dt * depositionEfficiency;
    if (!(melted > 0)) return;
    var relative = [for (axis in 0...3) tip[axis] - start[axis]];
    var s = dot(relative, tangent);
    if (over(tip) < 0.0) {
      stray += melted;
      return;
    }
    var index = Std.int(Math.min(count - 1, Math.max(0, Math.floor(s / BIN))));
    volume[index] += melted;
    deposited += melted;
    if (lastEpisode[index] != episode) {
      lastEpisode[index] = episode;
      episodes[index]++;
    }
    if (index < changedFrom) changedFrom = index;
    if (index > changedTo) changedTo = index;
  }

  /**
   * How far `tip` is from the seam line, in metres, when the tip is over the seam (within `REACH` of the line and no more
   * than `OVERRUN` past either end), and -1 when it is not.
   */
  public function over(tip:Array<Float>):Float {
    var relative = [for (axis in 0...3) tip[axis] - start[axis]];
    var s = dot(relative, tangent);
    var off = norm(minus(relative, scale(tangent, s)));
    return off > REACH || s < -OVERRUN || s > length + OVERRUN ? -1.0 : off;
  }

  /** Length of station `i`, in metres: all `BIN` but perhaps the last. */
  public function binLength(i:Int):Float return Math.min(BIN, length - i * BIN);

  /** Cross-section of the bead at station `i`, in square metres. */
  public function area(i:Int):Float return volume[i] / binLength(i);

  /** The equal-leg fillet's leg at station `i`, in metres; zero where there is no metal. */
  public function leg(i:Int):Float return Math.sqrt(2.0 * area(i));

  /** How many separate arc runs deposited at station `i`: two where a restart overlaps. */
  public function runs(i:Int):Int return episodes[i];

  /** Station `i`'s start along the seam, in metres. */
  public function stationAt(i:Int):Float return i * BIN;

  /** Point on the seam line at `distance` metres along it. */
  public function pointAt(distance:Float):Array<Float> return [for (axis in 0...3) start[axis] + tangent[axis] * distance];

  /** Stations with metal in them. */
  public function covered():Int {
    var n = 0;
    for (i in 0...count) if (volume[i] > 0.0) n++;
    return n;
  }

  /** The first and last stations with metal, or null for a bare seam. */
  function span():Null<{first:Int, last:Int}> {
    var first = -1, last = -1;
    for (i in 0...count) if (volume[i] > 0.0) {
      if (first < 0) first = i;
      last = i;
    }
    return first < 0 ? null : {first: first, last: last};
  }

  /** Metres of seam from the first station with metal to the last, however many between have none. */
  public function extent():Float {
    var found = span();
    return found == null ? 0.0 : stationAt(found.last) + binLength(found.last) - stationAt(found.first);
  }

  /** Stations without metal between the first and the last that have some: a gap in the bead. */
  public function gaps():Int {
    var found = span();
    if (found == null) return 0;
    var n = 0;
    for (i in found.first...found.last + 1) if (volume[i] <= 0.0) n++;
    return n;
  }

  /** Mean leg over the stretch from `from` to `to` (fractions of the seam), in metres: the achieved leg of a weld. */
  public function meanLeg(from:Float, to:Float):Float {
    var low = Std.int(Math.max(0, Math.floor(from * count))), high = Std.int(Math.min(count, Math.ceil(to * count)));
    if (high <= low) return 0.0;
    var sum = 0.0;
    for (i in low...high) sum += leg(i);
    return sum / (high - low);
  }

  /** Smallest and largest leg over the same stretch, in metres. */
  public function legRange(from:Float, to:Float):{min:Float, max:Float} {
    var low = Std.int(Math.max(0, Math.floor(from * count))), high = Std.int(Math.min(count, Math.ceil(to * count)));
    var smallest = Math.POSITIVE_INFINITY, largest = 0.0;
    for (i in low...high) {
      smallest = Math.min(smallest, leg(i));
      largest = Math.max(largest, leg(i));
    }
    return {min: high <= low ? 0.0 : smallest, max: largest};
  }

  /**
   * The stations changed since the last call, as a first and a last index, or null when none. The viewer re-meshes
   * only those.
   */
  public function takeChanges():Null<{first:Int, last:Int}> {
    if (changedTo < changedFrom) return null;
    var result = {first: changedFrom, last: changedTo};
    changedFrom = count;
    changedTo = -1;
    return result;
  }

  static function dot(a:Array<Float>, b:Array<Float>):Float return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  static function norm(a:Array<Float>):Float return Math.sqrt(dot(a, a));
  static function scale(a:Array<Float>, k:Float):Array<Float> return [a[0] * k, a[1] * k, a[2] * k];
  static function minus(a:Array<Float>, b:Array<Float>):Array<Float> return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  static function unit(a:Array<Float>):Array<Float> {
    var n = norm(a);
    if (!(n > 1e-9)) throw "A weld bead needs faces whose normals are not parallel";
    return scale(a, 1.0 / n);
  }
}
