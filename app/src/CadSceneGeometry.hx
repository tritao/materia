package app;

import cadkit.Mesh;
import cadkit.Shape;
import nativekit.scene.GeometryData;

/** Converts CadKit's millimetre mesh convention into SceneKit metres. */
class CadSceneGeometry {
  public static inline var METRES_PER_MILLIMETRE:Float = 0.001;

  public static function mountingPlate(widthMetres:Float, heightMetres:Float,
      thicknessMetres:Float, holeDiameterMetres:Float):GeometryData {
    var plate = CadPlateModel.create(widthMetres, heightMetres, thicknessMetres, holeDiameterMetres);
    try { var result = plate.geometry(); plate.close(); return result; }
    catch (error:Dynamic) { plate.close(); throw error; }
  }

  public static function mountingPlateGraph(graph:String):GeometryData {
    var plate = CadPlateModel.decode(graph);
    try { var result = plate.geometry(); plate.close(); return result; }
    catch (error:Dynamic) { plate.close(); throw error; }
  }

  public static function fromMesh(mesh:Mesh, ?source:Shape):GeometryData {
    var minimum = [1e300, 1e300, 1e300], maximum = [-1e300, -1e300, -1e300];
    if (mesh.vertexCount <= 0 || mesh.vertices.length != mesh.vertexCount * 24)
      throw "CadKit returned an inconsistent tessellation";
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
    var part:materia.project.SceneArtifact.SceneArtifactPart = {
      id: "cad", name: "cad", red: 0.7, green: 0.7, blue: 0.7,
      vertexCount: mesh.vertexCount, indexCount: mesh.indexCount,
      vertices: mesh.vertices, normals: mesh.normals, indices: mesh.indices,
      edgeSegments: mesh.edgeSegments, edgeIds: mesh.edgeIds,
      faceRanges: [for (range in mesh.faceRanges) {
        faceIndex: range.faceIndex, firstIndex: range.firstIndex, indexCount: range.indexCount
      }]
    };
    return CadPreviewGeometry.fromArtifact(part, minimum, maximum,
      METRES_PER_MILLIMETRE, false, true);
  }

}
