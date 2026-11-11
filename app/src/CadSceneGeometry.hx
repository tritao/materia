package app;

import cadkit.Mesh;
import cadkit.Shape;
import cadkit.threads.Threads;
import nativekit.scene.GeometryData;

/** Converts CadKit's millimetre mesh convention into SceneKit metres. */
class CadSceneGeometry {
  public static inline var METRES_PER_MILLIMETRE:Float = 0.001;
  /** Smallest scene size along an axis, in metres: a flat shape (a sketch profile) still has a serializable depth. */
  public static inline var MIN_SCENE_SIZE:Float = 0.000001;

  /** A shape's scene size in metres from its exact bounds; flat axes get MIN_SCENE_SIZE. */
  public static function dimensionsOf(shape:Shape):CadModelDimensions {
    var bounds = shape.bounds(), minimum = bounds.get_min(), maximum = bounds.get_max();
    inline function size(low:Float, high:Float):Float return Math.max(
      (high - low) * METRES_PER_MILLIMETRE,
      MIN_SCENE_SIZE
    );
    return {
      width: size(minimum.get_x(), maximum.get_x()),
      height: size(minimum.get_y(), maximum.get_y()),
      depth: size(minimum.get_z(), maximum.get_z())
    };
  }

  public static function mountingPlate(
    widthMetres:Float,
    heightMetres:Float,
    thicknessMetres:Float,
    holeDiameterMetres:Float
  ):GeometryData {
    var plate = CadPlateModel.create(widthMetres, heightMetres, thicknessMetres, holeDiameterMetres);
    try {
      var result = plate.geometry();
      plate.close();
      return result;
    } catch (error:Dynamic) {
      plate.close();
      throw error;
    }
  }

  public static function mountingPlateGraph(graph:String):GeometryData {
    var plate = CadPlateModel.decode(graph);
    try {
      var result = plate.geometry();
      plate.close();
      return result;
    } catch (error:Dynamic) {
      plate.close();
      throw error;
    }
  }

  /**
   * `origin`, when given, is the point the geometry is expressed about (a project part is centred on its full bounds, so
   * an earlier, smaller shape of it must use the same point), and `scale` converts model units to metres.
   */
  public static function fromMesh(mesh:Mesh,
    ?source:Shape, ?origin:Array<Float>, scale:Float = METRES_PER_MILLIMETRE):GeometryData {
    var minimum = [1e300, 1e300, 1e300], maximum = [-1e300, -1e300, -1e300];
    if (mesh.vertexCount <= 0
      || mesh.vertices.length != mesh.vertexCount * 24) throw "CadKit returned an inconsistent tessellation";
    for (index in 0...mesh.vertexCount) for (axis in 0...3) {
      var value = mesh.vertices.getDouble(index * 24 + axis * 8);
      if (!Math.isFinite(value)) throw "CadKit returned a non-finite vertex";
      minimum[axis] = Math.min(minimum[axis], value);
      maximum[axis] = Math.max(maximum[axis], value);
    }
    if (source != null) {
      var bounds = source.bounds(), low = bounds.get_min(), high = bounds.get_max();
      var sourceMinimum = [low.get_x(), low.get_y(), low.get_z()];
      var sourceMaximum = [high.get_x(), high.get_y(), high.get_z()];
      for (axis in 0...3) {
        minimum[axis] = Math.min(minimum[axis], sourceMinimum[axis]);
        maximum[axis] = Math.max(maximum[axis], sourceMaximum[axis]);
      }
    }
    var part:materia.project.SceneArtifact.SceneArtifactPart = {threads: source == null
      ? [] : Threads.renderData(source), details : source == null ? [] : cadkit.threads.Details.renderData(source),
      closed : source != null && source.hasSolidBoundary(),
      id : "cad", name : "cad", red : 0.7, green : 0.7, blue : 0.7, vertexCount : mesh.vertexCount,
      indexCount : mesh.indexCount, vertices : mesh.vertices, normals : mesh.normals, indices : mesh.indices,
      edgeSegments : mesh.edgeSegments, edgeIds : mesh.edgeIds, faceRanges : [for (range in mesh.faceRanges) {
      faceIndex:range.faceIndex, firstIndex:range.firstIndex, indexCount:range.indexCount
    }
    ]};
    return origin == null ? CadPreviewGeometry.fromArtifact(part,
      minimum, maximum, scale, false, true) : CadPreviewGeometry.fromArtifact(
        part,
        minimum,
        maximum,
        scale,
        true,
        false,
        origin
      );
  }

}
