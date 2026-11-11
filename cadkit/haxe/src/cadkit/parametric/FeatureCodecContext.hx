package cadkit.parametric;

import cadkit.modeling.Vector;

/** What a `FeatureCodec` may use from the document being saved or restored. */
class FeatureCodecContext {
  final document:Document;
  final remapDocument:Null<String -> String>;

  /** `remapDocument` maps a saved document id to the one being decoded into (a clone gets a new id). */
  public function new(document:Document, ?remapDocument:String -> String) {
    this.document = document;
    this.remapDocument = remapDocument;
  }

  public function encodeElementReference(value:ElementReference):Dynamic
    return {document: value.documentId.value, element: value.elementId.value};

  public function decodeElementReference(value:Dynamic):ElementReference {
    if (value == null) throw new ParametricError("document element reference is missing");
    var documentId = string(value, "document");
    return new ElementReference(new DocumentId(remapDocument == null ? documentId : remapDocument(documentId)), new ElementId(string(value, "element")));
  }

  /** A required array field of a payload object. */
  public function array(record:Dynamic, name:String):Array<Dynamic> {
    var value:Dynamic = Reflect.field(record, name);
    if (value == null) throw new ParametricError('document field is missing: $name');
    return cast value;
  }

  /** An earlier feature of the document being decoded, by saved id. */
  public function feature(id:Int):Feature {
    var found = document.featureById(id);
    if (found == null) throw new ParametricError('feature dependency is missing: $id');
    return found;
  }

  /** A feature's saved id. */
  public function featureId(feature:Feature):Int {
    if (feature.document != document) throw new ParametricError("dependency belongs to another document");
    return feature.id.toInt();
  }

  public function encodeVector(value:Vector):Dynamic return {x: value.x, y: value.y, z: value.z};

  public function decodeVector(value:Dynamic):Vector {
    if (value == null) throw new ParametricError("document vector is missing");
    return new Vector(number(value, "x"), number(value, "y"), number(value, "z"));
  }

  /** A required finite number field of a payload object. */
  public function number(record:Dynamic, name:String):Float {
    var value:Dynamic = Reflect.field(record, name);
    if (value == null || !(Std.isOfType(value, Float) || Std.isOfType(value, Int)))
      throw new ParametricError('document field is not a number: $name');
    var result:Float = cast value;
    if (!Math.isFinite(result)) throw new ParametricError('document field is not finite: $name');
    return result;
  }

  /** A required string field of a payload object. */
  public function string(record:Dynamic, name:String):String {
    var value:Dynamic = Reflect.field(record, name);
    if (value == null || !Std.isOfType(value, String)) throw new ParametricError('document field is not a string: $name');
    return cast value;
  }

  /** A required whole-number field of a payload object. */
  public function integer(record:Dynamic, name:String):Int {
    var value = number(record, name);
    if (Math.ffloor(value) != value || Math.abs(value) > 2147483647) throw new ParametricError('document field is not an integer: $name');
    return Std.int(value);
  }

  /** A required boolean field of a payload object. */
  public function boolean(record:Dynamic, name:String):Bool {
    var value:Dynamic = Reflect.field(record, name);
    if (value == null || !Std.isOfType(value, Bool)) throw new ParametricError('document field is not a boolean: $name');
    return cast value;
  }

  /** A required field of a payload object, whatever its type. */
  public function field(record:Dynamic, name:String):Dynamic {
    var value:Dynamic = Reflect.field(record, name);
    if (value == null) throw new ParametricError('document field is missing: $name');
    return value;
  }

  /** A face or edge selection rule as saved. */
  public function encodeSelection(selection:Null<SelectionRecipe>):Dynamic {
    if (selection == null) return null;
    return {
      kind: selection.kind,
      geometry: selection.geometry,
      parallel: selection.parallel == null ? null : encodeVector(selection.parallel),
      position: selection.position,
      axis: encodeVector(selection.axis),
      expectedCount: selection.expectedCount,
      tolerance: selection.tolerance
    };
  }

  public function decodeSelection(record:Dynamic):SelectionRecipe {
    if (record == null) throw new ParametricError("document selection is missing");
    var parallel:Dynamic = Reflect.field(record, "parallel");
    return new SelectionRecipe(string(record, "kind"), string(record, "geometry"), parallel == null ? null : decodeVector(parallel),
      string(record, "position"), decodeVector(field(record, "axis")), integer(record, "expectedCount"), number(record, "tolerance"));
  }
}
