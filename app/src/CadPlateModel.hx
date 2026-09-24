package app;

import cadkit.Geometry;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.TopologyFingerprint;
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

class CadPlateHoleEdit {
  public final fingerprint:TopologyFingerprint;
  public final x:Float;
  public final y:Float;
  public final diameter:Float;
  public var previousOutputId:Int;
  public var holeFeatureId:Int;
  public var cutFeatureId:Int;

  public function new(fingerprint:TopologyFingerprint, x:Float, y:Float, diameter:Float,
      previousOutputId:Int, holeFeatureId:Int, cutFeatureId:Int) {
    this.fingerprint = fingerprint;
    this.x = x;
    this.y = y;
    this.diameter = diameter;
    this.previousOutputId = previousOutputId;
    this.holeFeatureId = holeFeatureId;
    this.cutFeatureId = cutFeatureId;
  }

  public function updateFrom(other:CadPlateHoleEdit):Void {
    previousOutputId = other.previousOutputId;
    holeFeatureId = other.holeFeatureId;
    cutFeatureId = other.cutFeatureId;
  }
}

/** Typed application boundary around a CadKit mounting-plate feature graph. */
class CadPlateModel implements CadSessionModel {
  public static inline var WIDTH:String = "plate.width";
  public static inline var HEIGHT:String = "plate.height";
  public static inline var THICKNESS:String = "plate.thickness";
  public static inline var HOLE_DIAMETER:String = "hole.diameter";
  public static inline var HOLE_X:String = "hole.offsetx";
  public static inline var HOLE_Y:String = "hole.offsety";

  public final document:Document;
  var lastTessellationSeconds:Float = 0;
  var lastGeometryConversionSeconds:Float = 0;

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

  public function getDocument():Document return document;

  public function sceneDimensions():CadModelDimensions {
    var values=parameters();
    return {width:values.width,height:values.height,depth:values.thickness};
  }

  public function setSceneDimensions(width:Float,height:Float,depth:Float):Void {
    setMetreValues([
      {name:WIDTH,value:width},
      {name:HEIGHT,value:height},
      {name:THICKNESS,value:depth}
    ]);
  }

  public function featureNames():Array<String> {
    var result:Array<String> = [];
    for (index in 0...document.featureCount())
      if (document.featureAt(index).active)
        result.push(document.featureAt(index).serializationType());
    return result;
  }

  /** Read tree labels from the persisted graph without allocating CAD shapes per UI frame. */
  public static function featureNamesInGraph(graph:String):Array<String> {
    var root:Dynamic=haxe.Json.parse(graph);
    var features:Array<Dynamic> = cast Reflect.field(root,"features");
    var result:Array<String> = [];
    for(feature in features)result.push(Std.string(Reflect.field(feature,"type")));
    return result;
  }

  public function faceFingerprint(index:Int):TopologyFingerprint {
    return CadModelTopology.faceFingerprint(document.result(),index);
  }

  public function remapFace(fingerprint:TopologyFingerprint):Int {
    return CadModelTopology.remapFace(document.result(),fingerprint);
  }

  /** Adds a through hole on the selected top face. */
  public function addThroughHole(faceIndex:Int,xMetres:Float,yMetres:Float,diameterMetres:Float):CadPlateHoleEdit {
    var face=faceFingerprint(faceIndex);
    if(face.dz<0.98)throw new ParametricError("Select the plate's top planar face");
    var parameters=parameters();
    if(!Math.isFinite(diameterMetres)||diameterMetres<=0||
        !Math.isFinite(xMetres)||!Math.isFinite(yMetres)||
        Math.abs(xMetres)+diameterMetres/2>=parameters.width/2||
        Math.abs(yMetres)+diameterMetres/2>=parameters.height/2)
      throw new ParametricError("Hole must fit inside the plate");
    var source=document.outputFeature();
    var top=new SelectionRecipe("face","plane",Vector.Z(),"max",Vector.Z(),1);
    var scale=1.0/CadSceneGeometry.METRES_PER_MILLIMETRE;
    var transaction=document.beginTransaction();
    try {
      var hole=document.add(HoleFeature.plain(source,top,Vector.X(),"through-all",
        xMetres*scale,yMetres*scale,diameterMetres*scale));
      document.trackFeatureCreation(hole);
      var cut=document.add(new BooleanFeature(source,hole,BooleanOperation.Cut));
      document.trackFeatureCreation(cut);
      document.setOutputTracked(cut);
      document.recompute();
      transaction.commit();
      return new CadPlateHoleEdit(face,xMetres,yMetres,diameterMetres,
        source.id.toInt(),hole.id.toInt(),cut.id.toInt());
    } catch(error:Dynamic) {
      transaction.cancel();
      throw error;
    }
  }

  public function setHoleEditActive(edit:CadPlateHoleEdit, active:Bool):Void {
    var hole=document.featureById(edit.holeFeatureId);
    var cut=document.featureById(edit.cutFeatureId);
    if(active&&(hole==null||cut==null)) {
      var face=remapFace(edit.fingerprint);
      if(face<0)throw new ParametricError("selected face could not be restored");
      var replayed=addThroughHole(face,edit.x,edit.y,edit.diameter);
      edit.updateFrom(replayed);
      return;
    }
    var previous=document.featureById(edit.previousOutputId);
    if(hole==null||cut==null||previous==null)
      throw new ParametricError("hole edit references are unresolved");
    var transaction=document.beginTransaction();
    try {
      document.setFeatureActive(hole,active);
      document.setFeatureActive(cut,active);
      document.setOutputTracked(active?cut:previous);
      document.recompute();
      transaction.commit();
    } catch(error:Dynamic) {
      transaction.cancel();
      throw error;
    }
  }

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
    return geometryFor(document.result());
  }

  public function geometryFor(source:Shape, ?preview:Bool):nativekit.scene.GeometryData {
    var values = parameters();
    var centred = source.translate(Geometry.vec3(
      -values.width * 500.0, -values.height * 500.0, -values.thickness * 500.0));
    var tessellationStarted = Sys.time();
    var mesh:cadkit.Mesh;
    try {
      mesh=centred.tessellate(preview == true ? 0.5 : 0.1, preview == true ? 0.75 : 0.35);
      lastTessellationSeconds = Sys.time() - tessellationStarted;
    } catch (error:Dynamic) {
      lastTessellationSeconds = Sys.time() - tessellationStarted;
      centred.close();
      throw error;
    }
    var conversionStarted = Sys.time();
    try {
      var result = CadSceneGeometry.fromMesh(mesh, source);
      lastGeometryConversionSeconds = Sys.time() - conversionStarted;
      centred.close();
      return result;
    } catch (error:Dynamic) {
      lastGeometryConversionSeconds = Sys.time() - conversionStarted;
      centred.close();
      throw error;
    }
  }

  public function collisionBoundsFor(source:Shape):CadCollisionBounds {
    var values = parameters();
    return CadCollisionBounds.fromKernelBounds(source.bounds(),
      CadSceneGeometry.METRES_PER_MILLIMETRE,
      -values.width * 0.5, -values.height * 0.5, -values.thickness * 0.5);
  }

  public function tessellationSeconds():Float return lastTessellationSeconds;

  public function geometryConversionSeconds():Float return lastGeometryConversionSeconds;

  public function exportStep(path:String):Void {
    var values = parameters();
    var centred:Shape = document.result().translate(Geometry.vec3(
      -values.width * 500.0, -values.height * 500.0, -values.thickness * 500.0));
    try { centred.exportStep(path); centred.close(); }
    catch (error:Dynamic) { centred.close(); throw error; }
  }

  public function close():Void document.close();
}
