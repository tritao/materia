package nativekit.sim;

import NativeKitSim;

/** Immutable simulation state captured after a fixed tick. */
class SimSnapshot {
    final owner:Ownednksim_snapshot;
    var disposed:Bool = false;

    @:allow(SimWorld)
    private function new(owner:Ownednksim_snapshot) {
        this.owner = owner;
    }

    public function bodyCount():Int {
        ensureLive();
        var result = NativeKitSim.nksim_snapshot_get_body_count(owner.borrow());
        SimWorld.check(result.status, "snapshot.bodyCount");
        return haxe.Int64.toInt(result.out_count);
    }

    public function bodyAt(index:Int):BodyState {
        ensureLive();
        if (index < 0)
            throw "Simulation snapshot body index cannot be negative";
        var value = new nksim_body_state();
        value.set_struct_size(nksim_body_state.size());
        var result = NativeKitSim.nksim_snapshot_get_body(owner.borrow(), index, value);
        SimWorld.check(result.status, "snapshot.bodyAt");
        return BodyState.fromNative(value);
    }

    public function dispose():Void {
        if (disposed)
            return;
        owner.close();
        disposed = true;
    }

    public function isDisposed():Bool
        return disposed;

    function ensureLive():Void {
        if (disposed)
            throw "Simulation snapshot has been disposed";
    }
}
