package app;

import CadKit;
import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.EvaluationCancelled;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.modeling.Plane;
import cadkit.sketch.ConstrainedSketch;
import nativekit.scene.GeometryData;

/** Long-lived editor owner for one authored CadKit document and its published viewport result. */
class CadDocumentSession {
  public final model:CadSessionModel;
  public final document:Document;
  public var revision(default, null):Int = 0;
  public var diagnostics(default, null):Array<String> = [];
  public var selectedTopology:Null<TopologyFingerprint> = null;
  public var previewResult(default, null):Null<Shape> = null;
  public var collisionBounds(default, null):Null<CadCollisionBounds> = null;
  public var lastPublicationSeconds(default, null):Float = 0;

  var publishedShape:Null<Shape>;
  var publishedGeometry:Null<GeometryData>;
  var closed:Bool = false;
  var evaluationGeneration:Int = 0;
  var previewGeneration:Int = 0;

  public function new(model:CadSessionModel) {
    if (model == null)
      throw "CAD document sessions require a model";
    this.model = model;
    document = model.getDocument();
    var output = document.outputFeatureOrNull();
    if (output != null && output.currentShape() != null)
      publishCurrent(beginEvaluation());
  }

  public function geometry():GeometryData {
    ensureOpen();
    return publishedGeometry == null ? new GeometryData() : publishedGeometry;
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

  /** Open an isolated editable draft for a constrained-sketch feature. */
  public function beginSketchEdit(featureIndex:Int):CadSketchEditSession {
    ensureOpen();
    var candidate = document.featureAt(featureIndex);
    if (!Std.isOfType(candidate, ConstrainedSketchFeature))
      throw "selected CAD feature is not a constrained sketch";
    return new CadSketchEditSession(this, cast candidate);
  }

  /** Start an unattached sketch draft; no feature exists until a valid profile is applied. */
  public function beginNewSketchEdit(?plane:Plane, units:String = "mm"):CadSketchEditSession {
    ensureOpen();
    return new CadSketchEditSession(this, null, new ConstrainedSketch(plane, units));
  }

  public function performanceMetrics():CadPerformanceMetrics {
    ensureOpen();
    var nativeResources = CadKit.resourceCountsGetChecked();
    return {
      revision:revision,
      recomputeAttempts:document.recomputeAttemptCount,
      recomputeSeconds:document.lastRecomputeSeconds,
      evaluatedFeatures:document.lastRecomputeFeatureCount,
      sketchSolveSeconds:document.lastSketchSolveSeconds,
      sketchSolveCount:document.lastSketchSolveCount,
      sketchProfileSeconds:document.lastSketchProfileSeconds,
      tessellationSeconds:model.tessellationSeconds(),
      geometryConversionSeconds:model.geometryConversionSeconds(),
      publicationSeconds:lastPublicationSeconds,
      nativeShapeHandles:nativeResources.get_shapeCount(),
      nativeMeshHandles:nativeResources.get_meshCount(),
      nativeOperationHandles:nativeResources.get_operationCount()
    };
  }

  /** Run an authored edit, stage its render result, then clear CadKit-local history. */
  public function perform(edit:CadDocumentSession->Void):Void {
    ensureOpen();
    var ticket = beginEvaluation();
    var previousCancellationCheck = document.evaluationCancellationCheck;
    document.evaluationCancellationCheck = function() {
      return !isEvaluationCurrent(ticket) ||
        (previousCancellationCheck != null && previousCancellationCheck());
    };
    var authored = false;
    try {
      edit(this);
      authored = true;
      ensureEvaluationCurrent(ticket);
      var output = document.outputFeatureOrNull();
      if (output == null || output.currentShape() == null)
        publishEmpty(ticket);
      else
        publishCurrent(ticket);
      document.evaluationCancellationCheck = previousCancellationCheck;
      document.clearHistory();
      diagnostics = [];
    } catch (error:Dynamic) {
      document.evaluationCancellationCheck = previousCancellationCheck;
      diagnostics = [Std.string(error)];
      if (authored && document.undo()) {
        try {
          document.recompute();
          document.clearHistory();
        } catch (_:Dynamic) {}
      }
      throw error;
    }
  }

  /** Start a newer authored evaluation and return its cancellation ticket. */
  public function beginEvaluation():Int {
    ensureOpen();
    evaluationGeneration++;
    return evaluationGeneration;
  }

  /** Supersede any in-flight evaluation without touching the last published result. */
  public function cancelPendingEvaluation():Void {
    if (!closed)
      evaluationGeneration++;
  }

  public function isEvaluationCurrent(ticket:Int):Bool
    return !closed && ticket == evaluationGeneration;

  /** Start a preview generation. Older preview tickets can no longer publish. */
  public function beginPreview():Int {
    ensureOpen();
    previewGeneration++;
    return previewGeneration;
  }

  /** Retain and publish a preview only while its generation is still current. */
  public function publishPreview(ticket:Int, shape:Shape):Bool {
    ensureOpen();
    if (ticket != previewGeneration)
      return false;
    var candidate = shape.cloneShape();
    if (ticket != previewGeneration || closed) {
      candidate.close();
      return false;
    }
    if (previewResult != null)
      previewResult.close();
    previewResult = candidate;
    return true;
  }

  public function setPreview(shape:Null<Shape>):Void {
    ensureOpen();
    if (shape == null) {
      cancelPreview();
      return;
    }
    var ticket = beginPreview();
    publishPreview(ticket, shape);
  }

  public function cancelPreview(?ticket:Int):Void {
    if (ticket != null && ticket != previewGeneration)
      return;
    previewGeneration++;
    if (previewResult != null) {
      previewResult.close();
      previewResult = null;
    }
  }

  function publishCurrent(ticket:Int):Void {
    var publicationStarted = Sys.time();
    var nextShape:Null<Shape> = null;
    var nextGeometry:Null<GeometryData> = null;
    var nextCollisionBounds:Null<CadCollisionBounds> = null;
    try {
      ensureEvaluationCurrent(ticket);
      var source = document.result();
      nextShape = source.cloneShape();
      nextGeometry = model.geometryFor(cast nextShape);
      nextCollisionBounds = model.collisionBoundsFor(cast nextShape);
      ensureEvaluationCurrent(ticket);
    } catch (error:Dynamic) {
      lastPublicationSeconds = Sys.time() - publicationStarted;
      if (nextShape != null)
        nextShape.close();
      throw error;
    }

    var priorShape = publishedShape;
    publishedShape = nextShape;
    publishedGeometry = cast nextGeometry;
    collisionBounds = nextCollisionBounds;
    revision++;
    lastPublicationSeconds = Sys.time() - publicationStarted;
    if (priorShape != null)
      priorShape.close();
  }

  function publishEmpty(ticket:Int):Void {
    var publicationStarted = Sys.time();
    ensureEvaluationCurrent(ticket);
    var priorShape = publishedShape;
    publishedShape = null;
    publishedGeometry = null;
    collisionBounds = null;
    revision++;
    lastPublicationSeconds = Sys.time() - publicationStarted;
    if (priorShape != null)
      priorShape.close();
  }

  function ensureEvaluationCurrent(ticket:Int):Void {
    if (!isEvaluationCurrent(ticket))
      throw new EvaluationCancelled();
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
    collisionBounds = null;
    selectedTopology = null;
    model.close();
  }
}
