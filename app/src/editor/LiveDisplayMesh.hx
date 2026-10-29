package app.editor;

import app.CadGeometryData.CadDetailGeometry;
import haxe.io.Bytes;
import nativekit.scene.GeometryData;

/** A display's picture: a quad in the face's plane, a hair proud of it, with texture coordinates running over the picture. */
class LiveDisplayMesh {
  public static function build(detail:CadDetailGeometry):GeometryData {
    var frame = detail.frame;
    var halfWidth = detail.width / 2, halfHeight = detail.height / 2, lift = detail.width * 0.0002;
    var corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
    var positions = Bytes.alloc(4 * 12), normals = Bytes.alloc(4 * 12), uvs = Bytes.alloc(4 * 8);
    var low = [1e300, 1e300, 1e300], high = [-1e300, -1e300, -1e300];
    for (i in 0...4) {
      var point = frame.toWorld(new cadkit.modeling.Vector(corners[i][0] * halfWidth,
        corners[i][1] * halfHeight, lift));
      var values = [point.x, point.y, point.z];
      for (axis in 0...3) {
        positions.setFloat(i * 12 + axis * 4, values[axis]);
        low[axis] = Math.min(low[axis], values[axis]);
        high[axis] = Math.max(high[axis], values[axis]);
      }
      normals.setFloat(i * 12, frame.normal.x);
      normals.setFloat(i * 12 + 4, frame.normal.y);
      normals.setFloat(i * 12 + 8, frame.normal.z);
      uvs.setFloat(i * 8, (corners[i][0] + 1) / 2);
      // Image rows run top-down.
      uvs.setFloat(i * 8 + 4, (1 - corners[i][1]) / 2);
    }
    var result = new GeometryData();
    result.addTriangle(0, 1, 2);
    result.addTriangle(0, 2, 3);
    result.addStream(1, 2, positions, 4, 12);
    result.addStream(2, 2, normals, 4, 12);
    result.addStream(4, 1, uvs, 4, 8);
    result.addSubelement(0, result.triangleCount(), detail.source.faceIndex);
    result.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
    return result;
  }
}
