package cadkit.sketch;

/** Generic stable constraint record. Factory methods document the supported reference combinations. */
class SketchConstraint {
  public final id:String;
  public final kind:String;
  public final first:String;
  public final second:Null<String>;
  public final third:Null<String>;
  public final value:Float;
  /**
		A reference (measured) dimension: a distance, radius or angle that is not solved for, only measured after
		the solve (`SolvedSketch.measured`); `value` is then ignored. It never over-constrains a sketch.
	*/
  public final reference:Bool;

  private function new(
    id:String,
    kind:String,
    first:String,
    second:Null<String>,
    third:Null<String>,
    value:Float,
    reference:Bool = false
  ) {
    this.id = id;
    this.kind = kind;
    this.first = first;
    this.second = second;
    this.third = third;
    this.value = value;
    this.reference = reference;
  }

  /** This constraint with another value (a dimension edit); everything else, its reference flag included, stays. */
  public function withValue(value:Float):SketchConstraint return new SketchConstraint(
    id,
    kind,
    first,
    second,
    third,
    value,
    reference
  );

  /** This constraint as a reference (measured) dimension, or back as a driving one. */
  public function asReference(reference:Bool = true):SketchConstraint return new SketchConstraint(
    id,
    kind,
    first,
    second,
    third,
    value,
    reference
  );

  /** Distances, radii and angles: the kinds that have a value to drive or to measure. */
  public static function isDimension(kind:String):Bool return switch kind {
    case "distance", "radius", "angle", "horizontalDistance", "verticalDistance": true;
    default: false;
  };

  /** Dimensions measured in the sketch's length unit: every dimension but the angle. */
  public static function isLengthDimension(kind:String):Bool return isDimension(kind) && kind != "angle";

  /** Dimensions that must be positive. Horizontal and vertical distances are signed: the sign says which side. */
  public static function mustBePositive(kind:String):Bool return kind == "distance" || kind == "radius";

  /** Whether the constraint's reference in `slot` (0 first, 1 second, 2 third) names a point rather than an entity. */
  public static function referencesPoint(kind:String, slot:Int):Bool return switch kind {
    case "fixed": slot == 0;
    case "coincident", "distance", "horizontalDistance", "verticalDistance", "symmetric": slot < 2;
    case "pointOn": slot == 0;
    default: false;
  };
  public static function raw(id:String, kind:String,
    first:String, second:Null<String>, third:Null<String>, value:Float):SketchConstraint return new SketchConstraint(
      id,
      kind,
      first,
      second,
      third,
      value
    );

  public static function fixed(id:String, point:String):SketchConstraint return new SketchConstraint(
    id,
    "fixed",
    point,
    null,
    null,
    0
  );
  public static function coincident(id:String,
    first:String, second:String):SketchConstraint return new SketchConstraint(
      id,
      "coincident",
      first,
      second,
      null,
      0
    );
  public static function horizontal(id:String, line:String):SketchConstraint return new SketchConstraint(
    id,
    "horizontal",
    line,
    null,
    null,
    0
  );
  public static function vertical(id:String, line:String):SketchConstraint return new SketchConstraint(
    id,
    "vertical",
    line,
    null,
    null,
    0
  );
  public static function distance(id:String,
    first:String, second:String, value:Float):SketchConstraint return new SketchConstraint(
      id,
      "distance",
      first,
      second,
      null,
      value
    );
  public static function radius(id:String, entity:String, value:Float):SketchConstraint return new SketchConstraint(
    id,
    "radius",
    entity,
    null,
    null,
    value
  );
  public static function equal(id:String, first:String, second:String):SketchConstraint return new SketchConstraint(
    id,
    "equal",
    first,
    second,
    null,
    0
  );
  public static function parallel(id:String, first:String, second:String):SketchConstraint return new SketchConstraint(
    id,
    "parallel",
    first,
    second,
    null,
    0
  );
  public static function perpendicular(id:String,
    first:String, second:String):SketchConstraint return new SketchConstraint(
      id,
      "perpendicular",
      first,
      second,
      null,
      0
    );
  public static function angle(id:String,
    first:String, second:String, radians:Float):SketchConstraint return new SketchConstraint(
      id,
      "angle",
      first,
      second,
      null,
      radians
    );
  public static function concentric(id:String,
    first:String, second:String):SketchConstraint return new SketchConstraint(
      id,
      "concentric",
      first,
      second,
      null,
      0
    );
  public static function pointOn(id:String, point:String, entity:String):SketchConstraint return new SketchConstraint(
    id,
    "pointOn",
    point,
    entity,
    null,
    0
  );
  /** Tangency supports line-circle, line-arc, circle-circle, circle-arc, and arc-arc. */
  public static function tangent(id:String, first:String, second:String):SketchConstraint return new SketchConstraint(
    id,
    "tangent",
    first,
    second,
    null,
    0
  );
  /** Point `second` lies `value` to the right of point `first` (signed: negative to the left). */
  public static function horizontalDistance(id:String, first:String, second:String, value:Float):SketchConstraint
    return new SketchConstraint(id, "horizontalDistance", first, second, null, value);

  /** Point `second` lies `value` above point `first` (signed: negative below). */
  public static function verticalDistance(id:String, first:String, second:String, value:Float):SketchConstraint
    return new SketchConstraint(id, "verticalDistance", first, second, null, value);

  /** Reflect first point to second point across the referenced line. */
  public static function symmetric(id:String,
    first:String, second:String, axis:String):SketchConstraint return new SketchConstraint(
      id,
      "symmetric",
      first,
      second,
      axis,
      0
    );
}
