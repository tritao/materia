package app;

import cadkit.Geometry;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.HoleFeature;

typedef CadPlateParameters = {
  var width:Float;
  var height:Float;
  var thickness:Float;
  var holeDiameter:Float;
  var holeX:Float;
  var holeY:Float;
}

/** Typed application boundary around a CadKit mounting-plate feature graph. */
class CadPlateModel {
  public static inline var WIDTH:String = "plate.width";
  public static inline var HEIGHT:String = "plate.height";
  public static inline var THICKNESS:String = "plate.thickness";
  public static inline var HOLE_DIAMETER:String = "hole.diameter";
  public static inline var HOLE_X:String = "hole.offsetx";
  public static inline var HOLE_Y:String = "hole.offsety";

  final document:Document;

  function new(document:Document) this.document = document;

  public static function create(widthMetres:Float, heightMetres:Float, thicknessMetres:Float,
      holeDiameterMetres:Float, holeXMetres:Float = 0.0, holeYMetres:Float = 0.0):CadPlateModel {
    var scale = 1.0 / CadSceneGeometry.METRES_PER_MILLIMETRE;
    var width = widthMetres * scale, height = heightMetres * scale;
    var thickness = thicknessMetres * scale, diameter = holeDiameterMetres * scale;
    var holeX = holeXMetres * scale, holeY = holeYMetres * scale;
    var result = new Document();
    try {
      var base = result.add(new BoxFeature(width, height, thickness));
      var top = new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1);
      var hole = result.add(HoleFeature.plain(base, top, Vector.X(), "through-all",
        holeX, holeY, diameter));
      var cut = result.add(new BooleanFeature(base, hole, BooleanOperation.Cut));
      result.setOutput(cut);
      result.defineTypedParameter(WIDTH, width, ParameterKind.Length, "mm").bind(base.width);
      result.defineTypedParameter(HEIGHT, height, ParameterKind.Length, "mm").bind(base.depth);
      result.defineTypedParameter(THICKNESS, thickness, ParameterKind.Length, "mm").bind(base.height);
      result.defineTypedParameter(HOLE_DIAMETER, diameter, ParameterKind.Length, "mm").bind(hole.diameter);
      result.defineTypedParameter(HOLE_X, holeX, ParameterKind.Length, "mm").bind(hole.x);
      result.defineTypedParameter(HOLE_Y, holeY, ParameterKind.Length, "mm").bind(hole.y);
      result.recompute();
      return new CadPlateModel(result);
    } catch (error:Dynamic) {
      result.close();
      throw error;
    }
  }

  public static function decode(graph:String):CadPlateModel
    return new CadPlateModel(DocumentCodec.decode(graph));

  public function encode():String return DocumentCodec.encode(document);

  public function parameters():CadPlateParameters return {
    width: document.parameter(WIDTH).valueIn("m"),
    height: document.parameter(HEIGHT).valueIn("m"),
    thickness: document.parameter(THICKNESS).valueIn("m"),
    holeDiameter: document.parameter(HOLE_DIAMETER).valueIn("m"),
    holeX: document.parameter(HOLE_X).valueIn("m"),
    holeY: document.parameter(HOLE_Y).valueIn("m")
  };

  /** All parameter changes are one CadKit transaction; failed recompute restores the graph. */
  public function setMetres(name:String, value:Float):Void {
    setMetreValues([{name:name, value:value}]);
  }

  public function setMetreValues(values:Array<{name:String, value:Float}>):Void {
    var transaction = document.beginTransaction();
    try {
      for (entry in values) document.parameter(entry.name).set(entry.value, "m");
      document.recompute();
      transaction.commit();
    } catch (error:Dynamic) {
      transaction.cancel();
      throw error;
    }
  }

  public function geometry():nativekit.scene.GeometryData {
    var values = parameters();
    var centred = document.result().translate(Geometry.vec3(
      -values.width * 500.0, -values.height * 500.0, -values.thickness * 500.0));
    try {
      var result = CadSceneGeometry.fromMesh(centred.tessellate(0.1, 0.35));
      centred.close();
      return result;
    } catch (error:Dynamic) {
      centred.close();
      throw error;
    }
  }

  public function exportStep(path:String):Void {
    var values = parameters();
    var centred:Shape = document.result().translate(Geometry.vec3(
      -values.width * 500.0, -values.height * 500.0, -values.thickness * 500.0));
    try { centred.exportStep(path); centred.close(); }
    catch (error:Dynamic) { centred.close(); throw error; }
  }

  public function close():Void document.close();
}
