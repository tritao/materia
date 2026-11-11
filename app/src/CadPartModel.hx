package app;

import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ParametricError;
import cadkit.parametric.TopologyFingerprint;
import nativekit.scene.GeometryData;

/** Generic authored CAD document adapter, including a valid featureless starting state. */
class CadPartModel implements CadSessionModel {
  public final document:Document;
  /** A project part's recorded history shows about the part's own centre and in the project's units; see `decodeHistory`. */
  var historyOrigin:Null<Array<Float>> = null;
  var historyScale:Float = CadSceneGeometry.METRES_PER_MILLIMETRE;
  var lastTessellationSeconds:Float = 0;
  var lastGeometryConversionSeconds:Float = 0;

  function new(document:Document) {
    if (document == null) throw "generic CAD parts require a document";
    this.document = document;
  }

  public static function createEmpty():CadPartModel return new CadPartModel(new Document());

  public static function decode(graph:String):CadPartModel return new CadPartModel(DocumentCodec.decode(graph));

  /** The recorded steps of a project part, shown in the frame of that part's static geometry: `origin` and `scale`. */
  public static function decodeHistory(graph:String, origin:Array<Float>, scale:Float):CadPartModel {
    var model = decode(graph);
    model.historyOrigin = origin;
    model.historyScale = scale;
    return model;
  }

  public function getDocument():Document return document;

  public function encode():String return DocumentCodec.encode(document);

  public function close():Void document.close();

  public function geometryFor(source:Shape, ?preview:Bool):GeometryData {
    var started = Sys.time();
    var mesh:cadkit.Mesh;
    try {
      mesh = source.tessellate(preview == true ? 0.5 : 0.1, preview == true ? 0.75 : 0.35);
      lastTessellationSeconds = Sys.time() - started;
    } catch (error:Dynamic) {
      lastTessellationSeconds = Sys.time() - started;
      throw error;
    }
    var conversionStarted = Sys.time();
    try {
      var result = CadSceneGeometry.fromMesh(mesh, source, historyOrigin, historyScale);
      lastGeometryConversionSeconds = Sys.time() - conversionStarted;
      return result;
    } catch (error:Dynamic) {
      lastGeometryConversionSeconds = Sys.time() - conversionStarted;
      throw error;
    }
  }

  public function collisionBoundsFor(source:Shape):CadCollisionBounds return CadCollisionBounds.fromKernelBounds(
    source.bounds(),
    CadSceneGeometry.METRES_PER_MILLIMETRE
  );

  public function tessellationSeconds():Float return lastTessellationSeconds;

  public function geometryConversionSeconds():Float return lastGeometryConversionSeconds;

  public function sceneDimensions():CadModelDimensions {
    var shown = document.displayedFeature();
    if (shown == null || shown.currentShape() == null) return {width: 0.05, height: 0.05, depth: 0.01};
    return CadSceneGeometry.dimensionsOf(document.displayedShape());
  }

  public function setSceneDimensions(width:Float, height:Float,
    depth:Float):Void throw new ParametricError("Generic CAD dimensions are edited through their authored features");

  public function featureNames():Array<String> return [for (index in 0
    ...document.featureCount()) if (document.featureAt(index).active) document.featureAt(index).serializationType()];

  /** Face numbers belong to the shape on screen, which is the earlier one while the timeline is rolled back. */
  public function faceFingerprint(index:Int):TopologyFingerprint return CadModelTopology.faceFingerprint(
    document.displayedShape(),
    index
  );

  public function remapFace(fingerprint:TopologyFingerprint):Int return CadModelTopology.remapFace(
    document.displayedShape(),
    fingerprint
  );

  public function exportStep(path:String):Void document.result().exportStep(path);
}
