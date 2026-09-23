package app;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SolverSettings;
import haxe.Json;

/** Versioned persistence for the editor's isolated, possibly unfinished sketch draft. */
class SketchDraftCodec {
  public static inline var FORMAT:String = "cadkit.sketch-draft";
  public static inline var VERSION:Int = 1;
  static inline var MAX_ITEMS:Int = 10000;

  public static function encode(sketch:ConstrainedSketch, featureIndex:Int):String {
    if (sketch == null || featureIndex < -1)
      throw "invalid sketch draft state";
    var authoredPoints = sketch.points();
    var authoredEntities = sketch.entities();
    var authoredConstraints = sketch.constraints();
    if (authoredPoints.length > MAX_ITEMS || authoredEntities.length > MAX_ITEMS ||
        authoredConstraints.length > MAX_ITEMS)
      throw "sketch draft exceeds the persistence item limit";
    var points:Array<Dynamic> = [];
    for (point in authoredPoints) points.push({id:point.id, x:point.x, y:point.y});
    var entities:Array<Dynamic> = [];
    for (entity in authoredEntities) entities.push({id:entity.id, kind:entity.kind,
      first:entity.first, second:entity.second, radius:entity.radius,
      startAngle:entity.startAngle, endAngle:entity.endAngle,
      clockwise:entity.clockwise, construction:entity.construction});
    var constraints:Array<Dynamic> = [];
    for (constraint in authoredConstraints) constraints.push({id:constraint.id, kind:constraint.kind,
      first:constraint.first, second:constraint.second, third:constraint.third, value:constraint.value});
    var encoded = Json.stringify({format:FORMAT, version:VERSION, featureIndex:featureIndex,
      sketch:{units:sketch.units,
        plane:{origin:vector(sketch.plane.origin), xDirection:vector(sketch.plane.xDirection),
          normal:vector(sketch.plane.normal)},
        settings:{tolerance:sketch.settings.tolerance, rankTolerance:sketch.settings.rankTolerance,
          maxIterations:sketch.settings.maxIterations, initialDamping:sketch.settings.initialDamping},
        points:points, entities:entities, constraints:constraints}});
    if (encoded.length > 10000000)
      throw "sketch draft exceeds the persistence size limit";
    return encoded;
  }

  public static function decode(text:String):SketchDraftRecord {
    if (text == null || text.length == 0 || text.length > 10000000 || containsNul(text))
      throw "invalid sketch draft data";
    var root:Dynamic;
    try root = Json.parse(text) catch (_:Dynamic) throw "invalid sketch draft JSON";
    if (stringField(root, "format") != FORMAT || integerField(root, "version") != VERSION)
      throw "unsupported sketch draft format";
    var featureIndex = integerField(root, "featureIndex");
    if (featureIndex < -1)
      throw "invalid sketch draft feature index";

    var source:Dynamic = requiredField(root, "sketch");
    var units = stringField(source, "units");
    var planeRecord:Dynamic = requiredField(source, "plane");
    var settingsRecord:Dynamic = requiredField(source, "settings");
    var origin = vectorField(planeRecord, "origin");
    var xDirection = vectorField(planeRecord, "xDirection");
    var normal = vectorField(planeRecord, "normal");
    if (xDirection.length() < 1e-10 || normal.length() < 1e-10 ||
        Math.abs(xDirection.normalized().dot(normal.normalized())) > 1e-10)
      throw "invalid sketch workplane axes";
    var plane = new Plane(origin, xDirection, normal);
    var settings = new SolverSettings(positiveField(settingsRecord, "tolerance"),
      positiveField(settingsRecord, "rankTolerance"), boundedInteger(settingsRecord, "maxIterations", 1, 10000),
      positiveField(settingsRecord, "initialDamping"));
    var sketch = new ConstrainedSketch(plane, units, settings);
    for (record in arrayField(source, "points"))
      sketch.addPoint(new SketchPoint(stringField(record, "id"),
        numberField(record, "x"), numberField(record, "y")));
    for (record in arrayField(source, "entities")) {
      var id = stringField(record, "id"), kind = stringField(record, "kind");
      var first = stringField(record, "first"), second = optionalString(record, "second");
      var radius = numberField(record, "radius"), startAngle = numberField(record, "startAngle");
      var endAngle = numberField(record, "endAngle"), clockwise = boolField(record, "clockwise");
      var construction = boolField(record, "construction");
      switch kind {
        case "line":
          if (second == null) throw "line sketch entity has no second point";
          sketch.addEntity(SketchEntity.line(id, first, second, construction));
        case "circle": sketch.addEntity(SketchEntity.circle(id, first, radius, construction));
        case "arc": sketch.addEntity(SketchEntity.arc(id, first, radius, startAngle, endAngle,
          clockwise, construction));
        default: throw "unsupported sketch entity kind: " + kind;
      }
    }
    for (record in arrayField(source, "constraints"))
      sketch.addConstraint(SketchConstraint.raw(stringField(record, "id"),
        stringField(record, "kind"), stringField(record, "first"), optionalString(record, "second"),
        optionalString(record, "third"), numberField(record, "value")));
    return new SketchDraftRecord(featureIndex, sketch);
  }

  static function vector(value:Vector):Array<Float> return [value.x, value.y, value.z];

  static function vectorField(value:Dynamic, name:String):Vector {
    var components = arrayField(value, name, 3);
    if (components.length != 3)
      throw "sketch plane vectors need three coordinates";
    return new Vector(numberValue(components[0]), numberValue(components[1]), numberValue(components[2]));
  }

  static function requiredField(value:Dynamic, name:String):Dynamic {
    if (value == null || !Reflect.hasField(value, name))
      throw "missing sketch draft field: " + name;
    return Reflect.field(value, name);
  }

  static function stringField(value:Dynamic, name:String):String {
    var data = requiredField(value, name);
    if (!Std.isOfType(data, String)) throw "sketch draft field must be text: " + name;
    var result:String = cast data;
    if (result.length == 0 || result.length > 1024 || containsNul(result))
      throw "invalid sketch draft text: " + name;
    return result;
  }

  static function optionalString(value:Dynamic, name:String):Null<String> {
    if (value == null || !Reflect.hasField(value, name) || Reflect.field(value, name) == null)
      return null;
    return stringField(value, name);
  }

  static function numberField(value:Dynamic, name:String):Float
    return numberValue(requiredField(value, name));

  static function positiveField(value:Dynamic, name:String):Float {
    var result = numberField(value, name);
    if (result <= 0) throw "sketch solver setting must be positive: " + name;
    return result;
  }

  static function numberValue(value:Dynamic):Float {
    if (!Std.isOfType(value, Float) && !Std.isOfType(value, Int))
      throw "sketch draft value must be numeric";
    var result:Float = cast value;
    if (!Math.isFinite(result)) throw "sketch draft value must be finite";
    return result;
  }

  static function integerField(value:Dynamic, name:String):Int {
    var data = requiredField(value, name);
    if (!Std.isOfType(data, Int)) throw "sketch draft field must be an integer: " + name;
    return cast data;
  }

  static function boundedInteger(value:Dynamic, name:String, minimum:Int, maximum:Int):Int {
    var result = integerField(value, name);
    if (result < minimum || result > maximum) throw "sketch draft field is out of range: " + name;
    return result;
  }

  static function boolField(value:Dynamic, name:String):Bool {
    var data = requiredField(value, name);
    if (!Std.isOfType(data, Bool)) throw "sketch draft field must be boolean: " + name;
    return cast data;
  }

  static function arrayField(value:Dynamic, name:String, ?exact:Int):Array<Dynamic> {
    var data = requiredField(value, name);
    if (!Std.isOfType(data, Array)) throw "sketch draft field must be an array: " + name;
    var result:Array<Dynamic> = cast data;
    if (result.length > MAX_ITEMS || (exact != null && result.length != exact))
      throw "invalid sketch draft array length: " + name;
    return result;
  }

  static function containsNul(value:String):Bool {
    for (index in 0...value.length) if (value.charCodeAt(index) == 0) return true;
    return false;
  }
}

class SketchDraftRecord {
  public final featureIndex:Int;
  public final sketch:ConstrainedSketch;
  public function new(featureIndex:Int, sketch:ConstrainedSketch) {
    this.featureIndex = featureIndex;
    this.sketch = sketch;
  }
}
