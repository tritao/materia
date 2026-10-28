package machinekit.component;

import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;

/**
 * How a part is fixed to a panel and what it takes of it: the opening it needs, the area it occupies, extra
 * holes (studs), how far it stands out and reaches back, its rear body, and the thickest panel its fixing
 * allows. The part's frame has its origin at the centre of the panel's front face, +z toward the operator.
 * Any component can state this, so a layout works for every panel-mounted part, whichever package it is in.
 */
class PanelMountFacet implements ComponentFacet {
  /** The opening the panel needs for it. */
  public final cutout:Footprint;
  /** The panel area it occupies (knob, cap, flange), kept clear of other parts. */
  public final keepOut:Footprint;
  /** Extra round holes (studs), as offsets from its centre. */
  public final holes:Array<{x:Float, y:Float, diameter:Float}>;
  public final above:Float;
  public final behind:Float;
  /** What it occupies behind the panel, seen from the front. */
  public final body:Footprint;
  /** The thickest panel it can be fixed to; large when the part states no limit. */
  public final maxPanelThickness:Float;
  /** Bosses it needs on the panel's back to be screwed on; none for a part held by its own nut. */
  public final bosses:Array<PanelBoss>;
  /** "vendor-drawing", "listing", "measured" or "assumed". */
  public final source:String;

  public function new(
    cutout:Footprint,
    keepOut:Footprint,
    holes:Array<{x:Float, y:Float, diameter:Float}>,
    above:Float,
    behind:Float,
    body:Footprint,
    maxPanelThickness:Float,
    source:String,
    ?bosses:Array<PanelBoss>
  ) {
    this.cutout = cutout;
    this.keepOut = keepOut;
    this.holes = holes.copy();
    this.above = above;
    this.behind = behind;
    this.body = body;
    this.maxPanelThickness = maxPanelThickness;
    this.source = source;
    this.bosses = bosses == null ? [] : bosses.copy();
  }

  public function check(component:MachineComponent):Void {
    var name = component.designation;
    for (v in [above, behind, maxPanelThickness]) if (!Math.isFinite(v)
      && v != maxPanelThickness) throw '$name has a non-finite panel mount value';
    if (above < 0 || !(behind > 0) || !(maxPanelThickness
      > 0)) throw '$name needs a height above of zero or more and a positive depth and panel limit';
    for (f in [cutout, keepOut, body]) if (!(FootprintGeometry.area(f) > 0)) throw '$name has an empty panel footprint';
    for (h in holes) if (!(h.diameter > 0) || !Math.isFinite(h.x + h.y)) throw '$name has a bad panel hole';
    for (b in bosses) if (!(b.diameter > b.pilotDiameter) || !(b.pilotDiameter > 0)
      || !(b.height > 0) || !(b.pilotDepth > 0) || !Math.isFinite(b.x + b.y)) throw '$name has a bad panel boss';
    if (["vendor-drawing",
      "listing", "measured", "assumed"].indexOf(source) < 0) throw '$name has an unknown data source $source';
  }

  public function describe():String return 'panel mount, ${above} above, ${behind} behind, from $source';

  /** The facet of this kind on `component`, or null. */
  public static function of(component:MachineComponent):Null<PanelMountFacet> {
    for (facet in component.facets()) if (Std.isOfType(facet, PanelMountFacet)) return cast facet;
    return null;
  }
}
