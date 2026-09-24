package app;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;

/** Face data shared by box construction, shading, and perspective edge cues. */
class BoxGeometry {
  public static function faces(width:Float, height:Float, depth:Float):Array<BoxGeometryFace> {
    var hw = width / 2.0, hh = height / 2.0, hd = depth / 2.0;
    return [
      new BoxGeometryFace([[-hw, -hh, -hd], [-hw, hh, -hd], [hw, hh, -hd], [hw, -hh, -hd]], [0.0, 0.0, -1.0]),
      new BoxGeometryFace([[-hw, -hh, hd], [hw, -hh, hd], [hw, hh, hd], [-hw, hh, hd]], [0.0, 0.0, 1.0]),
      new BoxGeometryFace([[-hw, -hh, -hd], [hw, -hh, -hd], [hw, -hh, hd], [-hw, -hh, hd]], [0.0, -1.0, 0.0]),
      new BoxGeometryFace([[-hw, hh, -hd], [-hw, hh, hd], [hw, hh, hd], [hw, hh, -hd]], [0.0, 1.0, 0.0]),
      new BoxGeometryFace([[-hw, -hh, -hd], [-hw, -hh, hd], [-hw, hh, hd], [-hw, hh, -hd]], [-1.0, 0.0, 0.0]),
      new BoxGeometryFace([[hw, -hh, -hd], [hw, hh, -hd], [hw, hh, hd], [hw, -hh, hd]], [1.0, 0.0, 0.0])
    ];
  }

  public static function create(width:Float, height:Float, depth:Float):GeometryData {
    var geometry = new GeometryData();
    var faces = faces(width, height, depth);
    var vertexCount = faces.length * 4;
    var positions = Bytes.alloc(vertexCount * 12);
    var normals = Bytes.alloc(vertexCount * 12);
    var vertex = 0;
    for (face in faces) {
      var first = vertex;
      for (corner in face.corners) {
        geometry.addVertex(corner[0], corner[1], corner[2]);
        positions.setFloat(vertex * 12, corner[0]);
        positions.setFloat(vertex * 12 + 4, corner[1]);
        positions.setFloat(vertex * 12 + 8, corner[2]);
        normals.setFloat(vertex * 12, face.normal[0]);
        normals.setFloat(vertex * 12 + 4, face.normal[1]);
        normals.setFloat(vertex * 12 + 8, face.normal[2]);
        vertex++;
      }
      geometry.addTriangle(first, first + 1, first + 2);
      geometry.addTriangle(first, first + 2, first + 3);
    }
    geometry.addStream(1, 2, positions, vertexCount, 12);
    geometry.addStream(2, 2, normals, vertexCount, 12);
    geometry.setBounds(-width / 2.0, -height / 2.0, -depth / 2.0,
      width / 2.0, height / 2.0, depth / 2.0);
    return geometry;
  }
}

class BoxGeometryFace {
  public final corners:Array<Array<Float>>;
  public final normal:Array<Float>;

  public function new(corners:Array<Array<Float>>, normal:Array<Float>) {
    this.corners = corners;
    this.normal = normal;
  }
}
