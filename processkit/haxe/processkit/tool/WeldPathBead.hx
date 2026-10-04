package processkit.tool;

import robotkit.tool.*;

/**
 * The weld metal laid along a path of straight seams, as the wire deposited it: one `WeldBead` per segment, each with
 * the faces' normals of its own seam (the sides of a tube lie between different faces). Each step the metal goes to
 * the segment whose line the wire tip is nearest, among those the tip is over (`WeldBead.over`); at a corner the tip is
 * on both lines and the earlier segment keeps it until the tip has left that one's end. Metal the tip puts down
 * while it is over no segment is `stray`.
 */
class WeldPathBead {
  public final beads:Array<WeldBead>;

  final wireArea:Float;
  final depositionEfficiency:Float;
  var strayAway:Float = 0.0;

  /** `starts`, `stops` and `normals` (two per segment) give the segments in order, in one frame. */
  public function new(starts:Array<Array<Float>>, stops:Array<Array<Float>>, normals:Array<Array<Array<Float>>>, wireDiameterMm:Float,
      depositionEfficiency:Float) {
    if (starts == null || starts.length == 0 || stops == null || stops.length != starts.length || normals == null ||
        normals.length != starts.length)
      throw "A weld path bead needs a start, a stop and two normals for every segment";
    beads = [for (index in 0...starts.length) new WeldBead(starts[index], stops[index], normals[index][0], normals[index][1], wireDiameterMm,
      depositionEfficiency)];
    wireArea = beads[0].wireArea;
    this.depositionEfficiency = depositionEfficiency;
  }

  /** Metal in all the beads, in cubic metres. */
  public var deposited(get, never):Float;
  function get_deposited():Float {
    var sum = 0.0;
    for (bead in beads) sum += bead.deposited;
    return sum;
  }

  /** Metal that went anywhere but into a seam, in cubic metres. */
  public var stray(get, never):Float;
  function get_stray():Float {
    var sum = strayAway;
    for (bead in beads) sum += bead.stray;
    return sum;
  }

  public function reset():Void {
    for (bead in beads) bead.reset();
    strayAway = 0.0;
  }

  /** Advances `dt` seconds with the arc `established` (or not), the wire fed at `wireSpeed` m/min, its tip at `tip`. */
  public function step(dt:Float, established:Bool, wireSpeed:Float, tip:Array<Float>):Void {
    var chosen = -1;
    var nearest = Math.POSITIVE_INFINITY;
    for (index in 0...beads.length) {
      var off = beads[index].over(tip);
      // A segment the tip is not over is no candidate; of those it is over, the nearest line wins (the earlier on a tie).
      if (off >= 0.0 && off < nearest - 1e-9) {
        nearest = off;
        chosen = index;
      }
    }
    if (chosen < 0 && established)
      strayAway += Math.max(0.0, wireSpeed) / 60.0 * wireArea * dt * depositionEfficiency;
    for (index in 0...beads.length) beads[index].step(dt, established && index == chosen, wireSpeed, tip);
  }

  /** Stations with metal, over all the segments. */
  public function covered():Int {
    var sum = 0;
    for (bead in beads) sum += bead.covered();
    return sum;
  }

  /** The length of the seams together, in metres. */
  public function length():Float {
    var sum = 0.0;
    for (bead in beads) sum += bead.length;
    return sum;
  }

  /** Bare stretches inside a segment's metal, and a segment left with none when a later one has some. */
  public function gaps():Int {
    var sum = 0;
    var last = -1;
    for (index in 0...beads.length) if (beads[index].covered() > 0) last = index;
    for (index in 0...beads.length) {
      sum += beads[index].gaps();
      if (index < last && beads[index].covered() == 0) sum++;
    }
    return sum;
  }
}
