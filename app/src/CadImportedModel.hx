package app;

import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.features.ImportedShapeFeature;
import nativekit.scene.GeometryData;
import sys.io.File;

/** Adapter for a self-contained STEP-import feature graph in an editor session. */
class CadImportedModel implements CadSessionModel {
  public final document:Document;
  var lastTessellationSeconds:Float = 0;
  var lastGeometryConversionSeconds:Float = 0;

  function new(document:Document) this.document = document;

  public static function create(path:String):CadImportedModel {
    var stepText = File.getContent(path);
    var document = new Document();
    try {
      document.setOutput(document.add(new ImportedShapeFeature(stepText)));
      document.recompute();
      return new CadImportedModel(document);
    } catch (error:Dynamic) {
      document.close();
      throw error;
    }
  }

  public static function decode(graph:String):CadImportedModel
    return new CadImportedModel(DocumentCodec.decode(graph));

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
      var result = CadSceneGeometry.fromMesh(mesh, source);
      lastGeometryConversionSeconds = Sys.time() - conversionStarted;
      return result;
    } catch (error:Dynamic) {
      lastGeometryConversionSeconds = Sys.time() - conversionStarted;
      throw error;
    }
  }

  public function collisionBoundsFor(source:Shape):CadCollisionBounds
    return CadCollisionBounds.fromKernelBounds(source.bounds(),
      CadSceneGeometry.METRES_PER_MILLIMETRE);

  public function tessellationSeconds():Float return lastTessellationSeconds;

  public function geometryConversionSeconds():Float return lastGeometryConversionSeconds;

  public function sceneDimensions():CadModelDimensions {
    var bounds = collisionBoundsFor(document.result());
    return {width: bounds.halfExtents.x * 2, height: bounds.halfExtents.y * 2,
      depth: bounds.halfExtents.z * 2};
  }

  public function setSceneDimensions(width:Float, height:Float, depth:Float):Void
    throw "Imported STEP geometry has fixed dimensions; edit it with authored features";

  public function featureNames():Array<String>
    return [for (index in 0...document.featureCount())
      if (document.featureAt(index).active) document.featureAt(index).serializationType()];

  public function faceFingerprint(index:Int):TopologyFingerprint
    return CadModelTopology.faceFingerprint(document.result(), index);

  public function remapFace(fingerprint:TopologyFingerprint):Int
    return CadModelTopology.remapFace(document.result(), fingerprint);

  public function exportStep(path:String):Void document.result().exportStep(path);
}
