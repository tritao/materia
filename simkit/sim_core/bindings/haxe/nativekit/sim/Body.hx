package nativekit.sim;

import NativeKitSim;
import NativeKitScene;

/** Owns one body handle. The body remains bound to its scene occurrence. */
class Body {
    final world:SimWorld;
    final value:nksim_body;
    var disposed:Bool = false;

    @:allow(SimWorld)
    private function new(world:SimWorld, value:nksim_body) {
        this.world = world;
        this.value = value;
    }

    public function nativeHandle():nksim_body {
        ensureLive();
        return value;
    }

    public function state():BodyState {
        ensureLive();
        var state = new nksim_body_state();
        state.set_struct_size(nksim_body_state.size());
        var result = NativeKitSim.nksim_body_get_state(world.nativeHandle(), value, state);
        SimWorld.check(result.status, "body.state");
        return BodyState.fromNative(state);
    }

    public function dispose():Void {
        if (disposed)
            return;
        NativeKitSim.nksim_body_destroy(world.nativeHandle(), value);
        disposed = true;
    }

    public function isDisposed():Bool
        return disposed;

    function ensureLive():Void {
        if (disposed)
            throw "Simulation body has been disposed";
        world.ensureLive();
    }
}
