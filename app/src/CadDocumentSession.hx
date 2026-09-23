package app;

import cadkit.Shape;
import cadkit.parametric.TopologyFingerprint;
import nativekit.scene.GeometryData;

/** Long-lived editor owner for one authored CadKit document and its published viewport result. */
class CadDocumentSession {
  public final model:CadPlateModel;
  public var revision(default, null):Int = 0;
  public var diagnostics(default, null):Array<String> = [];
  public var selectedTopology:Null<TopologyFingerprint> = null;
  public var previewResult(default, null):Null<Shape> = null;

  var publishedShape:Null<Shape>;
  var publishedGeometry:Null<GeometryData>;
  var closed:Bool = false;

  public function new(model:CadPlateModel) {
    if (model == null)
      throw "CAD document sessions require a model";
    this.model = model;
    publishCurrent();
  }

  public function geometry():GeometryData {
    ensureOpen();
    if (publishedGeometry == null)
      throw "CAD document has no published geometry";
    return publishedGeometry;
  }

  /** Return an owning snapshot that remains valid across the next recompute. */
  public function copyPublishedShape():Shape {
    ensureOpen();
    if (publishedShape == null)
      throw "CAD document has no published shape";
    return publishedShape.cloneShape();
  }

  public function encode():String {
    ensureOpen();
    return model.encode();
  }

  /** Run an authored edit, stage its render result, then clear CadKit-local history. */
  public function perform(edit:CadDocumentSession->Void):Void {
    ensureOpen();
    var authored = false;
    try {
      edit(this);
      authored = true;
      publishCurrent();
      model.document.clearHistory();
      diagnostics = [];
    } catch (error:Dynamic) {
      diagnostics = [Std.string(error)];
      if (authored && model.document.canUndo()) {
        try {
          model.document.undo();
          model.document.recompute();
          publishCurrent();
          model.document.clearHistory();
        } catch (_:Dynamic) {}
      }
      throw error;
    }
  }

  public function setPreview(shape:Null<Shape>):Void {
    ensureOpen();
    if (previewResult != null)
      previewResult.close();
    previewResult = shape == null ? null : shape.cloneShape();
  }

  public function cancelPreview():Void {
    if (previewResult != null) {
      previewResult.close();
      previewResult = null;
    }
  }

  function publishCurrent():Void {
    var source = model.document.result();
    var nextShape = source.cloneShape();
    var nextGeometry:GeometryData;
    try {
      nextGeometry = model.geometryFor(nextShape);
    } catch (error:Dynamic) {
      nextShape.close();
      throw error;
    }
    if (publishedShape != null)
      publishedShape.close();
    publishedShape = nextShape;
    publishedGeometry = nextGeometry;
    revision++;
  }

  function ensureOpen():Void {
    if (closed)
      throw "CAD document session is closed";
  }

  public function close():Void {
    if (closed)
      return;
    closed = true;
    cancelPreview();
    if (publishedShape != null)
      publishedShape.close();
    publishedShape = null;
    publishedGeometry = null;
    selectedTopology = null;
    model.close();
  }
}
