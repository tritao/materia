package stockkit;
import StockKitNative;

/** Immutable copy-on-write radial layers. Survives destruction of the original stock. */
class RingSnapshot {
  final owner:Ownedsk_ring_snapshot_handle;
  var disposed = false;
  public function new(owner:Ownedsk_ring_snapshot_handle) this.owner = owner;
  public function borrow():sk_ring_snapshot_handle {
    if (disposed) throw "ring snapshot has been disposed";
    return owner.borrow();
  }
  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }
}
