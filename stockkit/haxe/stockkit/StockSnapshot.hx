package stockkit;

import StockKitNative;

/**
  A stock as it was, sharing tiles with it until either side changes them.
  Restoring it also restores the move history it had.
**/
class StockSnapshot {
  public final grid:StockGrid;
  public final history:Array<CutMove>;
  final owner:Ownedsk_snapshot_handle;
  var disposed = false;

  /** Made by `Stock.snapshot`. */
  public function new(grid:StockGrid, history:Array<CutMove>, owner:Ownedsk_snapshot_handle) {
    this.grid = grid;
    this.history = history;
    this.owner = owner;
  }

  public function borrow():sk_snapshot_handle {
    if (disposed) throw "stock snapshot has been disposed";
    return owner.borrow();
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }
}
