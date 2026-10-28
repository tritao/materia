package machinekit.hmi;

/**
 * A part at a position on a panel. The panel layout asks the part for everything else (footprints, heights,
 * mass, provenance), so there is one source for each number. Coordinates are millimetres from the panel
 * centre, x to the right, y up.
 */
class PlacedPart {
  public final id:String;
  public final part:HmiPart;
  public final x:Float;
  public final y:Float;

  public function new(id:String, part:HmiPart, x:Float, y:Float) {
    if (id == null || id == "") throw "A placed part needs an id";
    if (part == null) throw 'Placed part $id has no part';
    if (!Math.isFinite(x) || !Math.isFinite(y)) throw 'Placed part $id has a non-finite position';
    this.id = id; this.part = part; this.x = x; this.y = y;
  }

  /** The same part at another position. */
  public function movedTo(newX:Float, newY:Float):PlacedPart return new PlacedPart(id, part, newX, newY);

  /** What the panel must open for it. */
  public function cutout():Footprint return part.panelCutout();
  /** The panel area it occupies, kept clear of other parts. */
  public function keepOut():Footprint return part.panelKeepOut();
  /** Extra round holes (studs), as offsets from its centre. */
  public function holes():Array<{x:Float, y:Float, diameter:Float}> return part.panelHoles();
  /** What it occupies behind the panel, seen from the front. */
  public function body():Footprint return part.rearBody();
  public function above():Float return part.aboveFace();
  public function behind():Float return part.behindFace();
  public function massGrams():Float return part.massGrams();
  /** "vendor-drawing", "listing", "measured" or "assumed". */
  public function source():String return part.dataSource();
}
