package app;

import cadkit.Shape;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ParametricError;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.PocketFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import nativekit.scene.GeometryData;

/** Reusable parametric L bracket built from authored, constrained sketches. */
class CadBracketModel implements CadSessionModel {
  public static inline var WIDTH:String = "bracket.width";
  public static inline var HEIGHT:String = "bracket.height";
  public static inline var DEPTH:String = "bracket.depth";
  public static inline var WALL:String = "bracket.wall";
  public static inline var HOLE_RADIUS:String = "bracket.hole-radius";

  public final document:Document;
  var lastTessellationSeconds:Float = 0;
  var lastGeometryConversionSeconds:Float = 0;
  final anchorX:Float;
  final anchorZ:Float;
  final holeSketch:ConstrainedSketchFeature;
  final filletFeature:FilletFeature;

  function new(document:Document, anchorX:Float, anchorZ:Float,
      holeSketch:ConstrainedSketchFeature, filletFeature:FilletFeature) {
    this.document = document;
    this.anchorX = anchorX;
    this.anchorZ = anchorZ;
    this.holeSketch = holeSketch;
    this.filletFeature = filletFeature;
  }

  public static function create(widthMetres:Float, heightMetres:Float, depthMetres:Float):CadBracketModel {
    var width = widthMetres * 1000.0;
    var height = heightMetres * 1000.0;
    var depth = depthMetres * 1000.0;
    var wall = Math.min(6.0, Math.min(width, height) * 0.15);
    validateDimensions(width, height, depth, wall);

    var anchorX = -width / 2.0;
    var anchorZ = -height / 2.0;
    var document = new Document();
    try {
      var profile = bracketProfile(width, height, wall, anchorX, anchorZ);
      var base = document.add(new ConstrainedSketchFeature(profile));
      document.defineTypedParameter(WIDTH, width, ParameterKind.Length, "mm")
        .bind(base.dimension("width"));
      document.defineTypedParameter(HEIGHT, height, ParameterKind.Length, "mm")
        .bind(base.dimension("height"));
      document.defineTypedParameter(WALL, wall, ParameterKind.Length, "mm");
      document.parameter(WALL).bind(base.dimension("wall-right"));
      document.parameter(WALL).bind(base.dimension("wall-top"));

      var extrusion = document.add(ExtrudeFeature.along(base, depth, Vector.Y(), false, true));
      document.defineTypedParameter(DEPTH, depth, ParameterKind.Length, "mm")
        .bind(extrusion.amount);

      var faceSupport = new SelectionRecipe("face", "plane", Vector.Y(), "min", Vector.Y(), 1);
      var holeSketchModel = new ConstrainedSketch(Plane.XY(), "mm");
      var center = holeCenter(width, height, wall, anchorX, anchorZ);
      holeSketchModel.addPoint(new SketchPoint("hole-center", center.x, center.y));
      holeSketchModel.addEntity(SketchEntity.circle("mounting-hole", "hole-center", wall * 0.25));
      holeSketchModel.addConstraint(SketchConstraint.fixed("hole-anchor", "hole-center"));
      holeSketchModel.addConstraint(SketchConstraint.radius("hole-radius", "mounting-hole", wall * 0.25));
      var holeSketch = document.add(new ConstrainedSketchFeature(holeSketchModel, extrusion,
        faceSupport, Vector.X()));
      document.defineTypedParameter(HOLE_RADIUS, wall * 0.25, ParameterKind.Length, "mm")
        .bind(holeSketch.dimension("hole-radius"));

      var pocket = document.add(PocketFeature.throughAll(extrusion, holeSketch));
      var outerEdges = new SelectionRecipe("edge", "line", Vector.Y(), "ends", Vector.X(), 4);
      var filletFeature = document.add(new FilletFeature(pocket, Math.min(1.0, wall * 0.12), null, null, outerEdges));
      document.setOutput(filletFeature);
      document.recompute();
      return new CadBracketModel(document, anchorX, anchorZ, holeSketch, filletFeature);
    } catch (error:Dynamic) {
      document.close();
      throw error;
    }
  }

  public static function decode(graph:String):CadBracketModel {
    var document = DocumentCodec.decode(graph);
    try {
      if (document.featureCount() < 5)
        throw "CAD bracket graph is missing its modeling features";
      var base:ConstrainedSketchFeature = cast document.featureAt(0);
      var hole:ConstrainedSketchFeature = cast document.featureAt(2);
      var fillet:Null<FilletFeature> = null;
      for(index in 0...document.featureCount()) {
        var candidate=document.featureAt(index);
        if(candidate.serializationType()=="fillet")fillet=cast candidate;
      }
      if(fillet==null)throw "CAD bracket graph is missing its edge fillet";
      var points = base.sketch().points();
      var anchor:Null<SketchPoint> = null;
      for (point in points) if (point.id == "p0") anchor = point;
      if (anchor == null)
        throw "CAD bracket graph is missing its profile anchor";
      return new CadBracketModel(document, anchor.x, anchor.y, hole, fillet);
    } catch (error:Dynamic) {
      document.close();
      throw error;
    }
  }

  public function getDocument():Document return document;
  public function encode():String return DocumentCodec.encode(document);
  public function close():Void document.close();

  public function featureNames():Array<String> {
    var result:Array<String> = [];
    for (index in 0...document.featureCount())
      if (document.featureAt(index).active)
        result.push(document.featureAt(index).serializationType());
    return result;
  }

  public function faceFingerprint(index:Int):TopologyFingerprint
    return CadModelTopology.faceFingerprint(document.result(),index);

  public function remapFace(fingerprint:TopologyFingerprint):Int
    return CadModelTopology.remapFace(document.result(),fingerprint);

  public function sceneDimensions():CadModelDimensions return {
    width: document.parameter(WIDTH).valueIn("m"),
    height: document.parameter(HEIGHT).valueIn("m"),
    depth: document.parameter(DEPTH).valueIn("m")
  };

  public function wallThickness():Float return document.parameter(WALL).valueIn("m");

  public function holeRadius():Float return document.parameter(HOLE_RADIUS).valueIn("m");

  public function setHoleRadius(value:Float):Void {
    var nextMm=value*1000.0;
    var wall=document.parameter(WALL).valueIn("mm");
    if(!Math.isFinite(nextMm)||nextMm<=0||nextMm>=wall/2.0)
      throw new ParametricError("Bracket hole radius must be positive and smaller than half the wall thickness");
    var previous=document.parameter(HOLE_RADIUS).valueIn("mm");
    if(nextMm==previous)return;
    var transaction=document.beginTransaction();
    try {
      document.parameter(HOLE_RADIUS).set(value,"m");
      document.recompute();
      transaction.commit();
    } catch(error:Dynamic) {
      transaction.cancel();
      throw error;
    }
  }

  public function setWallThickness(value:Float):Void {
    var nextMm=value*1000.0;
    var width=document.parameter(WIDTH).valueIn("mm");
    var height=document.parameter(HEIGHT).valueIn("mm");
    var depth=document.parameter(DEPTH).valueIn("mm");
    validateDimensions(width,height,depth,nextMm);
    var previous=document.parameter(WALL).valueIn("mm");
    if(nextMm==previous)return;
    var transaction=document.beginTransaction();
    try {
      document.parameter(WALL).set(value,"m");
      document.parameter(HOLE_RADIUS).set(nextMm*0.25,"mm");
      filletFeature.radius.set(Math.min(1.0,nextMm*0.12));
      var center=holeCenter(width,height,nextMm,anchorX,anchorZ);
      var authoredCenter:Null<SketchPoint> = null;
      for(point in holeSketch.sketch().points())if(point.id=="hole-center")authoredCenter=point;
      if(authoredCenter==null||authoredCenter.x!=center.x||authoredCenter.y!=center.y)
        holeSketch.replacePoint(new SketchPoint("hole-center",center.x,center.y));
      document.recompute();
      transaction.commit();
    } catch(error:Dynamic) {
      transaction.cancel();
      throw error;
    }
  }

  public function setSceneDimensions(width:Float, height:Float, depth:Float):Void {
    var widthMm = width * 1000.0;
    var heightMm = height * 1000.0;
    var depthMm = depth * 1000.0;
    var wall = document.parameter(WALL).valueIn("mm");
    validateDimensions(widthMm, heightMm, depthMm, wall);
    var transaction = document.beginTransaction();
    try {
      document.parameter(WIDTH).set(width, "m");
      document.parameter(HEIGHT).set(height, "m");
      document.parameter(DEPTH).set(depth, "m");
      var center = holeCenter(widthMm, heightMm, wall, anchorX, anchorZ);
      var authoredCenter:Null<SketchPoint> = null;
      for (point in holeSketch.sketch().points()) if (point.id == "hole-center") authoredCenter = point;
      if (authoredCenter == null || authoredCenter.x != center.x || authoredCenter.y != center.y)
        holeSketch.replacePoint(new SketchPoint("hole-center", center.x, center.y));
      document.recompute();
      transaction.commit();
    } catch (error:Dynamic) {
      transaction.cancel();
      throw error;
    }
  }

  public function geometryFor(source:Shape):GeometryData {
    var tessellationStarted = Sys.time();
    var mesh:cadkit.Mesh;
    try {
      mesh = source.tessellate(0.1, 0.35);
      lastTessellationSeconds = Sys.time() - tessellationStarted;
    } catch (error:Dynamic) {
      lastTessellationSeconds = Sys.time() - tessellationStarted;
      throw error;
    }
    var conversionStarted = Sys.time();
    try {
      var result = CadSceneGeometry.fromMesh(mesh);
      lastGeometryConversionSeconds = Sys.time() - conversionStarted;
      return result;
    } catch (error:Dynamic) {
      lastGeometryConversionSeconds = Sys.time() - conversionStarted;
      throw error;
    }
  }

  public function tessellationSeconds():Float return lastTessellationSeconds;

  public function geometryConversionSeconds():Float return lastGeometryConversionSeconds;

  public function exportStep(path:String):Void document.result().exportStep(path);

  static function bracketProfile(width:Float, height:Float, wall:Float,
      anchorX:Float, anchorZ:Float):ConstrainedSketch {
    var sketch = new ConstrainedSketch(Plane.XZ(), "mm");
    var points = [
      new SketchPoint("p0", anchorX, anchorZ),
      new SketchPoint("p1", anchorX + width, anchorZ),
      new SketchPoint("p2", anchorX + width, anchorZ + wall),
      new SketchPoint("p3", anchorX + wall, anchorZ + wall),
      new SketchPoint("p4", anchorX + wall, anchorZ + height),
      new SketchPoint("p5", anchorX, anchorZ + height)
    ];
    for (point in points) sketch.addPoint(point);
    for (index in 0...points.length)
      sketch.addEntity(SketchEntity.line("edge-" + index, points[index].id,
        points[(index + 1) % points.length].id));
    for (id in ["edge-0", "edge-2", "edge-4"])
      sketch.addConstraint(SketchConstraint.horizontal("horizontal-" + id, id));
    for (id in ["edge-1", "edge-3", "edge-5"])
      sketch.addConstraint(SketchConstraint.vertical("vertical-" + id, id));
    sketch.addConstraint(SketchConstraint.fixed("anchor", "p0"));
    sketch.addConstraint(SketchConstraint.distance("width", "p0", "p1", width));
    sketch.addConstraint(SketchConstraint.distance("wall-right", "p1", "p2", wall));
    sketch.addConstraint(SketchConstraint.distance("height", "p5", "p0", height));
    sketch.addConstraint(SketchConstraint.distance("wall-top", "p4", "p5", wall));
    return sketch;
  }

  static function holeCenter(width:Float, height:Float, wall:Float,
      anchorX:Float, anchorZ:Float):{x:Float, y:Float} {
    var baseArea = width * wall;
    var webArea = wall * (height - wall);
    var totalArea = baseArea + webArea;
    var centroidX = (baseArea * (anchorX + width / 2.0)
      + webArea * (anchorX + wall / 2.0)) / totalArea;
    var centroidZ = (baseArea * (anchorZ + wall / 2.0)
      + webArea * (anchorZ + wall + (height - wall) / 2.0)) / totalArea;
    var targetX = anchorX + wall / 2.0;
    var targetZ = anchorZ + wall + (height - wall) / 2.0;
    return {x: targetX - centroidX, y: targetZ - centroidZ};
  }

  static function validateDimensions(width:Float, height:Float, depth:Float, wall:Float):Void {
    if (!Math.isFinite(width) || !Math.isFinite(height) || !Math.isFinite(depth) || !Math.isFinite(wall)
        || width <= wall * 2 || height <= wall * 2 || depth <= 0 || wall <= 0)
      throw new ParametricError("Bracket width and height must exceed twice the wall thickness, and depth must be positive");
  }
}
