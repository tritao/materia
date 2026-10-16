package toolpathkit.path;

/** Rounded bead geometry shared by slicing, G-code and motion lowering.
 * Lengths are metres and areas m^2. These arithmetic helpers expect validated
 * positive inputs; a stadium's overall width is at least its height. Input
 * validation stays at each caller's settings/program boundary.
 */
class BeadSection {
  /** Rectangle with semicircular sides, width measured edge to edge. */
  public static inline function area(width:Float,height:Float):Float
    return height*(width-height)+Math.PI*height*height/4.0;

  public static inline function width(area:Float,height:Float):Float
    return area/height+height*(1.0-Math.PI/4.0);

  /** Neighbouring beads at this spacing fill a layer of the given height. */
  public static inline function spacing(width:Float,height:Float):Float
    return width-height*(1.0-Math.PI/4.0);

  /** Reduce the nominal layer height when the supplied area only admits a circular bead. */
  public static inline function heightForArea(area:Float,nominalHeight:Float):Float
    return Math.min(nominalHeight,Math.sqrt(4.0*area/Math.PI));

  public static inline function filamentArea(diameter:Float):Float
    return Math.PI*diameter*diameter/4.0;
}
