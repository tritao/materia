package app.editor;

import NativeKitScene;

import app.CadGeometryData.CadDetailGeometry;
import haxe.io.Bytes;
import nativekit.scene.GeometryData;

/** A display's picture: a quad in the face's plane, a hair proud of it, with texture coordinates running over the picture. */
class LiveDisplayMesh {
  /** The segments round a disc's rim. */
  static inline var RIM:Int = 32;

  /** A fan's points for a disc inscribed in the square: the centre, then the rim counter-clockwise. */
  static function disc(rim:Int):Array<Array<Float>> {
    var points:Array<Array<Float>> = [[0.0, 0.0]];
    for (i in 0...rim) points.push([Math.cos(2 * Math.PI * i / rim), Math.sin(2 * Math.PI * i / rim)]);
    return points;
  }

  public static function build(detail:CadDetailGeometry):GeometryData {
    var frame = detail.frame;
    var halfWidth = detail.width / 2, halfHeight = detail.height / 2, lift = detail.width * 0.0002;
    var corners = detail.source.round ? disc(RIM) : [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
    var count = corners.length;
    var positions = Bytes.alloc(count * 12), normals = Bytes.alloc(count * 12), uvs = Bytes.alloc(count * 8);
    var low = [1e300, 1e300, 1e300], high = [-1e300, -1e300, -1e300];
    for (i in 0...count) {
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
    if (detail.source.round) {
      for (i in 1...count) result.addTriangle(0, i, i == count - 1 ? 1 : i + 1);
    } else {
      result.addTriangle(0, 1, 2);
      result.addTriangle(0, 2, 3);
    }
    result.addStream(NksceneVertexSemantic.Position, NksceneVertexFormat.Float32x3, positions, 4, 12);
    result.addStream(NksceneVertexSemantic.Normal, NksceneVertexFormat.Float32x3, normals, 4, 12);
    result.addStream(NksceneVertexSemantic.Texcoord0, NksceneVertexFormat.Float32x2, uvs, 4, 8);
    result.addSubelement(0, result.triangleCount(), detail.source.faceIndex);
    result.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
    return result;
  }
}
