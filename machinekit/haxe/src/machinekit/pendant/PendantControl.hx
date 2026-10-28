package machinekit.pendant;

/**
 * One part on the pendant face. `cutout` is what the face plate must open for it, `keepOut` is the
 * face area the part occupies (knob, cap, flange), so nothing else may overlap it. Heights are
 * from the face plate: `above` stands on the outside, `behind` reaches into the case. Every value
 * says where it came from in `source`, so an assumed number cannot pass for a measured one.
 */
class PendantControl {
  public final id:String;
  public final kind:ControlKind;
  public final x:Float;
  public final y:Float;
  public final cutout:Footprint;
  public final keepOut:Footprint;
  /** Extra round holes in the face for this part (stud holes, anti-rotation pins), at offsets from its centre. */
  public final extraHoles:Array<{x:Float, y:Float, diameter:Float}>;
  public final above:Float;
  public final behind:Float;
  public final massGrams:Float;
  /** "vendor-drawing", "measured", "datasheet" or "assumed". */
  public final source:String;

  public function new(id:String, kind:ControlKind, x:Float, y:Float, cutout:Footprint, keepOut:Footprint,
      above:Float, behind:Float, massGrams:Float, source:String,
      ?extraHoles:Array<{x:Float, y:Float, diameter:Float}>) {
    if (id == null || id == "") throw "A control needs an id";
    for (v in [x, y, above, behind, massGrams]) if (!Math.isFinite(v)) throw 'Control $id has a non-finite value';
    if (above < 0 || behind < 0 || massGrams < 0) throw 'Control $id has a negative size or mass';
    checkFootprint(cutout, id);
    checkFootprint(keepOut, id);
    if (["vendor-drawing", "measured", "datasheet", "assumed"].indexOf(source) < 0) throw 'Control $id has an unknown source $source';
    this.id = id; this.kind = kind; this.x = x; this.y = y; this.cutout = cutout; this.keepOut = keepOut;
    this.above = above; this.behind = behind; this.massGrams = massGrams; this.source = source;
    this.extraHoles = extraHoles == null ? [] : extraHoles.copy();
  }

  static function checkFootprint(f:Footprint, id:String):Void switch f {
    case Circle(d): if (!(d > 0) || !Math.isFinite(d)) throw 'Control $id has a bad circle';
    case Rect(w, h): if (!(w > 0) || !(h > 0) || !Math.isFinite(w + h)) throw 'Control $id has a bad rectangle';
  };

  /** The same control at another position. */
  public function movedTo(newX:Float, newY:Float):PendantControl
    return new PendantControl(id, kind, newX, newY, cutout, keepOut, above, behind, massGrams, source, extraHoles);
}
