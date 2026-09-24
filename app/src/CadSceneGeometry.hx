package app;

import cadkit.Mesh;
import cadkit.Shape;
import CadKit;
import haxe.io.Bytes;
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
    var vertexBytes:Bytes = mesh.vertices;
    var normalBytes:Bytes = mesh.normals;
    var indexBytes:Bytes = mesh.indices;
    if (vertexBytes.length != mesh.vertexCount * 24 || normalBytes.length != mesh.vertexCount * 24 ||
        indexBytes.length != mesh.indexCount * 4 || mesh.indexCount % 3 != 0)
      throw "CadKit returned an inconsistent tessellation";
    var geometry = new GeometryData();
    var positions = Bytes.alloc(mesh.vertexCount * 12);
    var normals = Bytes.alloc(mesh.vertexCount * 12);
    var minX = 1e300, minY = 1e300, minZ = 1e300;
    var maxX = -1e300, maxY = -1e300, maxZ = -1e300;
    for (index in 0...mesh.vertexCount) {
      var offset = index * 24;
      var x = vertexBytes.getDouble(offset) * METRES_PER_MILLIMETRE;
      var y = vertexBytes.getDouble(offset + 8) * METRES_PER_MILLIMETRE;
      var z = vertexBytes.getDouble(offset + 16) * METRES_PER_MILLIMETRE;
      geometry.addVertex(x, y, z);
      positions.setFloat(index * 12, x);
      positions.setFloat(index * 12 + 4, y);
      positions.setFloat(index * 12 + 8, z);
      normals.setFloat(index * 12, normalBytes.getDouble(offset));
      normals.setFloat(index * 12 + 4, normalBytes.getDouble(offset + 8));
      normals.setFloat(index * 12 + 8, normalBytes.getDouble(offset + 16));
      minX = Math.min(minX, x); minY = Math.min(minY, y); minZ = Math.min(minZ, z);
      maxX = Math.max(maxX, x); maxY = Math.max(maxY, y); maxZ = Math.max(maxZ, z);
    }
    for (triangle in 0...Std.int(mesh.indexCount / 3)) {
      var offset = triangle * 12;
      geometry.addTriangle(indexBytes.getInt32(offset), indexBytes.getInt32(offset + 4),
        indexBytes.getInt32(offset + 8));
    }
    // SceneKit format/semantic values are stable C ABI constants: float3 and normal.
    geometry.addStream(1, 2, positions, mesh.vertexCount, 12);
    geometry.addStream(2, 2, normals, mesh.vertexCount, 12);
    for (range in mesh.faceRanges)
      geometry.addSubelement(Std.int(range.firstIndex / 3), Std.int(range.indexCount / 3), range.faceIndex);
    if (source != null) {
      appendBrepEdges(geometry, source);
      var kernelBounds = source.bounds();
      var minimum = kernelBounds.get_min();
      var maximum = kernelBounds.get_max();
      minX = Math.min(minX, minimum.get_x() * METRES_PER_MILLIMETRE);
      minY = Math.min(minY, minimum.get_y() * METRES_PER_MILLIMETRE);
      minZ = Math.min(minZ, minimum.get_z() * METRES_PER_MILLIMETRE);
      maxX = Math.max(maxX, maximum.get_x() * METRES_PER_MILLIMETRE);
      maxY = Math.max(maxY, maximum.get_y() * METRES_PER_MILLIMETRE);
      maxZ = Math.max(maxZ, maximum.get_z() * METRES_PER_MILLIMETRE);
    }
    geometry.setBounds(minX, minY, minZ, maxX, maxY, maxZ);
    return geometry;
  }

  static function appendBrepEdges(geometry:GeometryData, source:Shape):Void {
    var edgeCount = source.subshapeCount(CadKit.ShapeKind.Edge);
    for (edgeIndex in 0...edgeCount) {
      var edge = source.subshape(CadKit.ShapeKind.Edge, edgeIndex);
      try {
        if (edge.edgeLength() > 1e-9) {
          var curve = edge.curveKind();
          var steps = curve == CadKit.CurveKind.Line ? 1 :
            (curve == CadKit.CurveKind.Circle || curve == CadKit.CurveKind.Ellipse ? 48 : 64);
          var prior = edge.positionAt(0.0);
          for (step in 1...(steps + 1)) {
            var next = edge.positionAt(step / steps);
            geometry.addStrokeSegment(
              prior.get_x() * METRES_PER_MILLIMETRE,
              prior.get_y() * METRES_PER_MILLIMETRE,
              prior.get_z() * METRES_PER_MILLIMETRE,
              next.get_x() * METRES_PER_MILLIMETRE,
              next.get_y() * METRES_PER_MILLIMETRE,
              next.get_z() * METRES_PER_MILLIMETRE);
            prior = next;
          }
        }
      } catch (error:Dynamic) {
        edge.close();
        throw error;
      }
      edge.close();
    }
  }
}
