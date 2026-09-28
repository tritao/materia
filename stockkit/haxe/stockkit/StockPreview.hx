package stockkit;

/**
  Keeps display meshes of a stock in chunks of tiles and rebuilds only the
  chunks whose tiles changed since the last `update`: contoured surfaces for
  a tri-dexel stock, else columns. A chunk also depends on the tiles just
  past its +x and +y edges, whose rays it reads. Headless: hand the meshes to
  a renderer.
**/
class StockPreview {
  public final stock:Stock;
  /** Tiles per chunk side. */
  public final chunkTiles:Int;
  public final chunksX:Int;
  public final chunksY:Int;
  public var coloring(default, null):StockColoring;
  /** Current mesh of each chunk, row by row; null until the first update. */
  public final meshes:Array<Null<StockMesh>>;
  var seen:Array<Null<haxe.Int64>> = [];
  var colorsChanged = true;

  public function new(stock:Stock, coloring:StockColoring, chunkTiles:Int = 4) {
    if (chunkTiles < 1) throw "stock preview chunks need at least one tile";
    this.stock = stock;
    this.coloring = coloring;
    this.chunkTiles = chunkTiles;
    chunksX = Std.int((stock.tilesX() + chunkTiles - 1) / chunkTiles);
    chunksY = Std.int((stock.tilesY() + chunkTiles - 1) / chunkTiles);
    meshes = [for (_ in 0...chunksX * chunksY) null];
  }

  public function setColoring(coloring:StockColoring):Void {
    this.coloring = coloring;
    colorsChanged = true;
  }

  /** Rebuilds changed chunks and returns their indices (chunk (cx, cy) is cy * chunksX + cx). */
  public function update():Array<Int> {
    var revisions = stock.tileRevisions();
    var tilesX = stock.tilesX(), tilesY = stock.tilesY();
    var changedTile = [for (k in 0...revisions.length)
      colorsChanged || seen.length == 0 || seen[k] != revisions[k]];
    var rayColors = switch coloring {
      case ByDeviation(target, tolerance, onTarget, leftover, gouge) if (colorsChanged || changedTile.indexOf(true) >= 0):
        deviationColors(target, tolerance, onTarget, leftover, gouge);
      case _: null;
    };
    var palette = switch coloring {
      case BySource(color, _): [for (move in stock.history) color(move)];
      case _: null;
    };
    var rebuilt:Array<Int> = [];
    for (cy in 0...chunksY)
      for (cx in 0...chunksX) {
        var x0 = cx * chunkTiles, y0 = cy * chunkTiles;
        var x1 = Std.int(Math.min(tilesX, x0 + chunkTiles)), y1 = Std.int(Math.min(tilesY, y0 + chunkTiles));
        var dirty = false;
        // The chunk's tiles plus one more column and row for its walls.
        for (ty in y0...Std.int(Math.min(tilesY, y1 + 1)))
          for (tx in x0...Std.int(Math.min(tilesX, x1 + 1)))
            if (changedTile[ty * tilesX + tx]) dirty = true;
        if (!dirty && meshes[cy * chunksX + cx] != null) continue;
        meshes[cy * chunksX + cx] = switch coloring {
          case BySource(_, original) if (stock.lattice.triDexel):
            stock.contour(x0, y0, x1 - x0, y1 - y0, palette, original);
          case BySource(_, original):
            stock.mesh(x0, y0, x1 - x0, y1 - y0, true, false, palette, original, rayColors);
          case ByDeviation(_, _, _, _, _) if (stock.lattice.triDexel):
            stock.contour(x0, y0, x1 - x0, y1 - y0, null, 0, rayColors);
          case ByDeviation(_, _, _, _, _):
            stock.mesh(x0, y0, x1 - x0, y1 - y0, false, false, null, 0, rayColors);
        };
        rebuilt.push(cy * chunksX + cx);
      }
    seen = [for (revision in revisions) revision];
    colorsChanged = false;
    return rebuilt;
  }

  /**
    The move that made the surface under a picked triangle of chunk `chunk`,
    or null for untouched stock: the link from a click on the preview to the
    operation and source line (`move.opIndex`, `move.provenance`).
  **/
  public function pick(chunk:Int, triangle:Int):Null<CutMove> {
    var mesh = meshes[chunk];
    if (mesh == null) throw "stock preview chunk has no mesh yet";
    var source = mesh.sourceAt(triangle);
    return source == Stock.ORIGINAL ? null : stock.history[source];
  }

  function deviationColors(target:Stock, tolerance:Float, onTarget:Int, leftover:Int,
      gouge:Int):Array<Int> {
    var comparison = stock.compare(target);
    return [for (k in 0...comparison.gouge.length)
      comparison.largestGouge[k] > tolerance ? gouge
      : comparison.largestLeftover[k] > tolerance ? leftover : onTarget];
  }
}
