package stockkit;

/**
  What one move did to the stock, in cubic metres: the material its flutes
  removed, and the stock left after that cut which its shank and holder
  overlapped. Contact is measured after the move's own cut, so a climbing
  move whose shank meets material its flutes remove later is not reported.
**/
class MoveOutcome {
  /** Index of the move in the stock's history. */
  public final source:Int;
  public final move:CutMove;
  public final removed:Float;
  public final shankContact:Float;
  public final holderContact:Float;

  public function new(source:Int, move:CutMove, removed:Float, shankContact:Float,
      holderContact:Float) {
    this.source = source;
    this.move = move;
    this.removed = removed;
    this.shankContact = shankContact;
    this.holderContact = holderContact;
  }
}
