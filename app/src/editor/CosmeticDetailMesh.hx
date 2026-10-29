package app.editor;

import app.CadGeometryData.CadDetailGeometry;
import haxe.io.Bytes;
import materia.project.SurfaceDetailRenderData.SurfaceDetailKinds;
import nativekit.scene.GeometryData;

/**
 * A detailed face's overlay geometry: the base mesh's own triangles for that face, with texture coordinates made from the
 * face's axis frame, u round the axis (times the tooth count for a knurl) and v along it. Vertices are not shared, so a
 * triangle that crosses the seam gets its own unwrapped u.
 */
class CosmeticDetailMesh {
  public static function build(base:GeometryData, detail:CadDetailGeometry):GeometryData {
    var face = base.subelementGeometry(detail.source.faceIndex);
    if (face == null) throw 'Detail ${detail.source.faceName} has no base face triangles';
    var positions = face.stream(1), normals = face.stream(2);
    if (positions == null || normals == null || positions.format != 2
      || normals.format != 2) throw 'Detail ${detail.source.faceName} needs Float32 position and normal streams';
    var positionStride = positions.stride == 0 ? 12 : positions.stride,
      normalStride = normals.stride == 0 ? 12 : normals.stride;
    var triangles = face.triangleCount();
    var count = triangles * 3;
    var positionBytes = Bytes.alloc(count * 12),
      normalBytes = Bytes.alloc(count * 12), uvBytes = Bytes.alloc(count * 8);
    var low = [1e300, 1e300, 1e300], high = [-1e300, -1e300, -1e300];
    var repeats = detail.source.kind == SurfaceDetailKinds.KNURL ? detail.source.count : 1;
    var result = new GeometryData();
    for (triangle in 0...triangles) {
      var us:Array<Float> = [], vs:Array<Float> = [];
      for (corner in 0...3) {
        var index = face.indexAt(triangle * 3 + corner);
        var x = positions.bytes.getFloat(index * positionStride),
          y = positions.bytes.getFloat(index * positionStride + 4),
          z = positions.bytes.getFloat(index * positionStride + 8);
        var local = detail.frame.toLocal(new cadkit.modeling.Vector(x, y, z));
        var angle = Math.atan2(local.y, local.x) / (2 * Math.PI);
        us.push(angle < 0 ? angle + 1 : angle);
        vs.push(local.z / detail.length);
      }
      // A triangle across the seam takes its low corners round to the high side.
      var spread = Math.max(us[0], Math.max(us[1], us[2])) - Math.min(us[0], Math.min(us[1], us[2]));
      for (corner in 0...3) {
        var out = triangle * 3 + corner, index = face.indexAt(triangle * 3 + corner);
        for (axis in 0...3) {
          var value = positions.bytes.getFloat(index * positionStride + axis * 4);
          positionBytes.setFloat(out * 12 + axis * 4, value);
          normalBytes.setFloat(out * 12 + axis * 4, normals.bytes.getFloat(index * normalStride + axis * 4));
          low[axis] = Math.min(low[axis], value);
          high[axis] = Math.max(high[axis], value);
        }
        var u = spread > 0.5 && us[corner] < 0.5 ? us[corner] + 1 : us[corner];
        uvBytes.setFloat(out * 8, u * repeats);
        uvBytes.setFloat(out * 8 + 4, vs[corner]);
      }
      result.addTriangle(triangle * 3, triangle * 3 + 1, triangle * 3 + 2);
    }
    result.addStream(1, 2, positionBytes, count, 12);
    result.addStream(2, 2, normalBytes, count, 12);
    result.addStream(4, 1, uvBytes, count, 8);
    result.addSubelement(0, result.triangleCount(), detail.source.faceIndex);
    if (count > 0) result.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
    return result;
  }
}
