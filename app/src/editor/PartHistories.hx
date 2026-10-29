package app.editor;

import app.CadDocumentSession;
import app.CadPartModel;
import cadkit.parametric.TimelineModel;
import materia.project.SceneSourceMap;

/**
 * Recorded modeling histories of a generated project's parts. A part is opened as a live (but not authored) CAD session
 * the first time its history is needed; rolling it back shows an earlier shape, in the frame of the part's static
 * geometry, and rewrites that part's step rows to match.
 */
class PartHistories {
  final entries:Map<String, PartHistory> = new Map();

  public function new() {
  }

  /** Remember the history of scene object `objectId`; `origin` and `scale` are the frame of its static geometry. */
  public function register(objectId:String, graph:String, origin:Array<Float>, scale:Float):Void {
    var previous = entries.get(objectId);
    if (previous != null) previous.dispose();
    entries.set(objectId, new PartHistory(graph, origin, scale));
  }

  public function has(objectId:String):Bool return entries.exists(objectId);

  /** The part's live session, opened on first use. */
  public function session(objectId:String):CadDocumentSession {
    var entry = entries.get(objectId);
    if (entry == null) throw "No recorded history for " + objectId;
    return entry.open();
  }

  public function clear():Void {
    for (entry in entries) entry.dispose();
    entries.clear();
  }

  /** The step with document feature id `featureId` as an index into the document, or -1. */
  public function featureIndex(objectId:String, featureId:Int):Int {
    var document = session(objectId).document;
    for (index in 0...document.featureCount()) if (document.featureAt(index).id.toInt() == featureId) return index;
    return -1;
  }

  /** The index the view ends at: the marker, or the last feature when the whole history is shown. */
  public function shownIndex(objectId:String):Int {
    var document = session(objectId).document;
    return document.rollbackIndex < 0 ? document.featureCount() - 1 : document.rollbackIndex;
  }

  /**
   * Make `map`'s step rows for this part describe the shape being shown: faces follow the shown shape, and steps after
   * the marker are labelled as rolled back and own no faces. The first call remembers the original rows.
   */
  public function restyleSteps(objectId:String, occurrence:String, map:SceneSourceMap):Void {
    var entry = entries.get(objectId);
    if (entry == null) return;
    var document = entry.open().document;
    var rows = new Map<Int, cadkit.parametric.TimelineModel.TimelineRow>();
    for (row in TimelineModel.of(document)) rows.set(row.id, row);
    for (feature in map.features) {
      if (feature.occurrence != occurrence) continue;
      var original = entry.original.get(feature.id);
      if (original == null) {
        original = {label: feature.label, faces: feature.faces};
        entry.original.set(feature.id, original);
      }
      var hash = feature.id.lastIndexOf("#");
      var id = hash < 0 ? null : Std.parseInt(feature.id.substr(hash + 1));
      var row = id == null ? null : rows.get(id);
      if (document.rollbackIndex < 0) {
        feature.label = original.label;
        feature.faces = original.faces;
      } else if (row != null && !row.rolledBack) {
        feature.label = original.label;
        feature.faces = row.faces;
      } else {
        feature.label = "Rolled back · " + original.label;
        feature.faces = [];
      }
    }
  }
}

private class PartHistory {
  final graph:String;
  final origin:Array<Float>;
  final scale:Float;
  var session:Null<CadDocumentSession> = null;
  /** The step rows as the project wrote them, by source id. */
  public final original:Map<String, {label:String, faces:Array<Int>}> = new Map();

  public function new(graph:String, origin:Array<Float>, scale:Float) {
    this.graph = graph;
    this.origin = origin;
    this.scale = scale;
  }

  public function open():CadDocumentSession {
    var opened = session;
    if (opened != null) return opened;
    var model = CadPartModel.decodeHistory(graph, origin, scale);
    try {
      opened = new CadDocumentSession(model);
    } catch (error:Dynamic) {
      model.close();
      throw error;
    }
    session = opened;
    return opened;
  }

  public function dispose():Void {
    var opened = session;
    if (opened != null) opened.close();
    session = null;
  }
}
