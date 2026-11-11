package cadkit.parametric;

import cadkit.parametric.features.BooleanFeature;

/** One history row: a feature as a view lists it. */
typedef TimelineRow = {
  var id:Int;
  var label:String;
  var group:String;
  /** The feature's serialization type, for an icon or a filter. */
  var kind:String;
  var active:Bool;
  /** Left out of the shown history by the rollback marker. */
  var rolledBack:Bool;
  /** The row this one is a part of: a cutting or joining tool belongs to the boolean that applies it; -1 for a top-level step. */
  var parent:Int;
  /** Indices of the output shape's faces this step created and that still exist. */
  var faces:Array<Int>;
}

/**
 * A UI-free view of a document's history: one row per feature in authoring order (with the rollback marker applied), with each feature's group, the step
 * that owns it, and the output faces it created. Face ownership comes from the creating feature's tag in the element
 * name (`f12:box.-y`); a tool's faces belong to the boolean that consumed the tool, so a cut owns the walls it made.
 */
class TimelineModel {
  public static function of(document:Document):Array<TimelineRow> {
    var parents = toolOwners(document);
    var faces = new Map<Int, Array<Int>>();
    var output = document.displayedFeature();
    var shape = output == null ? null : output.currentShape();
    if (shape != null) {
      var names = shape.elementNames(CadKit.ShapeKind.Face);
      for (index in 0...names.length) {
        var creator = creatorOf(names[index]);
        if (creator < 0) continue;
        var step = parents.exists(creator) ? parents.get(creator) : creator;
        var owned = faces.get(step);
        if (owned == null) {
          owned = [];
          faces.set(step, owned);
        }
        owned.push(index);
      }
    }
    var rows:Array<TimelineRow> = [];
    for (index in 0...document.featureCount()) {
      var feature = document.featureAt(index);
      var id = feature.id.toInt();
      var owned = faces.get(id);
      rows.push({
        id: id,
        label: feature.label != "" ? feature.label : defaultLabel(feature),
        group: feature.group,
        kind: feature.serializationType(),
        active: feature.active,
        rolledBack: document.rollbackIndex >= 0 && index > document.rollbackIndex,
        parent: parents.exists(id) ? parents.get(id) : -1,
        faces: owned == null ? [] : owned
      });
    }
    return rows;
  }

  /** The feature id in a face name's leading `f<id>:` tag, or -1. */
  /** A registered type's own row name; otherwise the saved type. */
  static function defaultLabel(feature:Feature):String {
    var type = feature.serializationType();
    var codec = DocumentCodec.isBuiltInType(type) ? null : FeatureTypeRegistry.codec(type);
    return codec == null ? type : codec.defaultLabel(feature);
  }

  static function creatorOf(name:String):Int {
    if (name.length < 3 || name.charAt(0) != "f") return -1;
    var end = 1;
    while (end < name.length && name.charCodeAt(end) >= 48 && name.charCodeAt(end) <= 57) end++;
    if (end == 1 || end >= name.length || name.charAt(end) != ":") return -1;
    return Std.parseInt(name.substring(1, end));
  }

  /**
   * For every feature that exists only to feed a boolean's second operand, or a union's parts, the first such boolean or union. The walk stops at a
   * boolean: a tool that was itself built by booleans (a body made, then joined) keeps those steps of its own.
   */
  static function toolOwners(document:Document):Map<Int, Int> {
    var owners = new Map<Int, Int>();
    for (index in 0...document.featureCount()) {
      var feature = document.featureAt(index);
      if (!feature.active) continue;
      // A union is made of its parts: they are its sub-steps, so the union's row carries their faces.
      var union = Std.downcast(feature, cadkit.parametric.features.UnionAllFeature);
      if (union != null) {
        var parts = new Map<Int, Bool>();
        for (part in union.parts) if (part.active) collectTool(part, parts);
        for (id in parts.keys()) if (!owners.exists(id)) owners.set(id, union.id.toInt());
        continue;
      }
      var boolean = Std.downcast(feature, BooleanFeature);
      if (boolean == null) continue;
      var base = new Map<Int, Bool>();
      collect(boolean.first, base);
      var tool = new Map<Int, Bool>();
      collectTool(boolean.second, tool);
      for (id in tool.keys()) if (!base.exists(id) && !owners.exists(id)) owners.set(id, boolean.id.toInt());
    }
    return owners;
  }

  static function collectTool(feature:Feature, into:Map<Int, Bool>):Void {
    if (Std.isOfType(feature, BooleanFeature)) return;
    var id = feature.id.toInt();
    if (into.exists(id)) return;
    into.set(id, true);
    for (dependency in feature.dependencyFeatures()) if (dependency.active) collectTool(dependency, into);
  }

  static function collect(feature:Feature, into:Map<Int, Bool>):Void {
    var id = feature.id.toInt();
    if (into.exists(id)) return;
    into.set(id, true);
    for (dependency in feature.dependencyFeatures()) if (dependency.active) collect(dependency, into);
  }
}
