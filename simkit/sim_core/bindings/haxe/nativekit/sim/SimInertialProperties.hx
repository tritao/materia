package nativekit.sim;

import NativeKitSim;

/** Local centre in metres and centroidal row-major inertia in kg m². */
class SimInertialProperties {
    final center:Array<Float>;
    final tensor:Array<Float>;
    public function new(centerOfMass:Array<Float>, inertiaTensor:Array<Float>) {
        if (centerOfMass == null || centerOfMass.length != 3 || inertiaTensor == null || inertiaTensor.length != 9)
            throw "Object inertial properties need three centre coordinates and nine tensor entries";
        for (value in centerOfMass) if (!Math.isFinite(value)) throw "Object centre of mass must be finite";
        for (value in inertiaTensor) if (!Math.isFinite(value)) throw "Object inertia must be finite";
        center = centerOfMass.copy(); tensor = inertiaTensor.copy();
    }
    @:allow(SimSession)
    private function writeNative(desc:nksim_object_desc):Void {
        desc.set_has_inertial_properties(1);
        for (i in 0...3) desc.set_center_of_mass(i, center[i]);
        for (i in 0...9) desc.set_inertia_tensor(i, tensor[i]);
    }
}
