package stockkit;

/** The chunks one refresh rebuilt: chunk `chunks[k]` now has mesh `meshes[k]`. */
class StockPreviewUpdate {
  public final chunks:Array<Int>;
  public final meshes:Array<StockMesh>;
  public final chunksX:Int;
  public final chunksY:Int;

  public function new(chunks:Array<Int>, meshes:Array<StockMesh>, chunksX:Int, chunksY:Int) {
    this.chunks = chunks;
    this.meshes = meshes;
    this.chunksX = chunksX;
    this.chunksY = chunksY;
  }
}
