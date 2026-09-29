package stockkit;

import haxe.io.Bytes;

/**
  A display mesh of part of a stock, as byte streams ready for a renderer:
  float xyz positions and normals per vertex, uint32 triangle indices, RGBA8
  colours per vertex, and the source move of each triangle for picking: a
  column mesh (`Stock.mesh`) or a contoured surface (`Stock.contour`).
**/
class StockMesh {
  public final vertexCount:Int;
  public final triangleCount:Int;
  public final positions:Bytes;
  public final normals:Bytes;
  public final indices:Bytes;
  public final colors:Bytes;
  public final triangleSources:Bytes;

  public function new(vertexCount:Int, triangleCount:Int, positions:Bytes, normals:Bytes,
      indices:Bytes, colors:Bytes, triangleSources:Bytes) {
    this.vertexCount = vertexCount;
    this.triangleCount = triangleCount;
    this.positions = positions;
    this.normals = normals;
    this.indices = indices;
    this.colors = colors;
    this.triangleSources = triangleSources;
  }

  /** The source (history index) of the move that made `triangle`'s surface, or `Stock.ORIGINAL`. */
  public function sourceAt(triangle:Int):Int {
    if (triangle < 0 || triangle >= triangleCount) throw "stock mesh triangle out of range";
    var source = triangleSources.getInt32(4 * triangle);
    return source == -1 ? Stock.ORIGINAL : source;
  }
}
