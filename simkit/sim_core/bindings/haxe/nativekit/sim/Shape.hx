package nativekit.sim;

import NativeKitSim;

/** Owns one simulation collision shape resource. */
class Shape {
    final world:SimWorld;
    final value:nksim_shape;
    var disposed:Bool = false;

    @:allow(SimWorld)
    private function new(world:SimWorld, value:nksim_shape) {
        this.world = world;
        this.value = value;
    }

    public function nativeHandle():nksim_shape {
        ensureLive();
        return value;
    }

    public function dispose():Void {
        if (disposed)
            return;
        NativeKitSim.nksim_shape_destroy(world.nativeHandle(), value);
        disposed = true;
    }

    public function isDisposed():Bool
        return disposed;

    function ensureLive():Void {
        if (disposed)
            throw "Simulation shape has been disposed";
        world.ensureLive();
    }
}
