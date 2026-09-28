package stockkit;

/** How a stock preview colours its surfaces. Colours are 0xRRGGBBAA. */
enum StockColoring {
  /** Surfaces take the colour of the move that made them; untouched stock takes `original`. */
  BySource(color:CutMove->Int, original:Int);
  /**
    Against a target stock on the same lattice: gouges deeper than
    `tolerance` in `gouge`, leftover thicker than `tolerance` in `leftover`,
    the rest in `onTarget`. Contoured meshes judge each surface by its own
    ray (walls by X and Y rays, floors by Z rays); column meshes judge each
    Z ray as a whole.
  **/
  ByDeviation(target:Stock, tolerance:Float, onTarget:Int, leftover:Int, gouge:Int);
}
