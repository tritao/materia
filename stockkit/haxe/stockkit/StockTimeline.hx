package stockkit;

/**
  Scrubs through a program: the stock after any number of its moves. Cuts
  the whole program once, keeping a snapshot every `interval` moves; seeking
  restores the nearest snapshot at or before the position and cuts forward.
**/
class StockTimeline {
  public final stock:Stock;
  public final moves:Array<CutMove>;
  public final interval:Int;
  /** Moves cut into the stock so far. */
  public var position(default, null):Int;
  final snapshots:Array<StockSnapshot> = [];

  /** `stock` must be fresh (no history); it ends at the program's end. */
  public function new(stock:Stock, moves:Array<CutMove>, interval:Int = 2000) {
    if (interval < 1) throw "timeline snapshot interval must be positive";
    if (stock.history.length != 0) throw "timeline needs a stock no move has cut yet";
    this.stock = stock;
    this.moves = moves;
    this.interval = interval;
    snapshots.push(stock.snapshot());
    var at = 0;
    while (at < moves.length) {
      var end = Std.int(Math.min(moves.length, at + interval));
      stock.cut(moves.slice(at, end));
      at = end;
      if (at % interval == 0 && at < moves.length) snapshots.push(stock.snapshot());
    }
    position = moves.length;
  }

  /** Shows the stock after the first `target` moves. */
  public function seek(target:Int):Void {
    if (target < 0 || target > moves.length) throw "timeline position is outside the program";
    if (target == position) return;
    if (target < position || Std.int(target / interval) > Std.int(position / interval)) {
      var snapshot = Std.int(Math.min(snapshots.length - 1, target / interval));
      stock.restore(snapshots[snapshot]);
      position = snapshot * interval;
    }
    if (target > position) stock.cut(moves.slice(position, target));
    position = target;
  }

  public function dispose():Void {
    for (snapshot in snapshots) snapshot.dispose();
  }
}
