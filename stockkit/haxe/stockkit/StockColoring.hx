package stockkit;

/** How a stock preview colours its surfaces. Colours are 0xRRGGBBAA. */
enum StockColoring {
  /** Surfaces take the colour of the move that made them; untouched stock takes `original`. */
  BySource(color:CutMove->Int, original:Int);
  /**
    Per ray against a target on the same grid: gouges deeper than `tolerance`
    in `gouge`, leftover thicker than `tolerance` in `leftover`, the rest in
    `onTarget`.
  **/
  ByDeviation(target:Stock, tolerance:Float, onTarget:Int, leftover:Int, gouge:Int);
}
