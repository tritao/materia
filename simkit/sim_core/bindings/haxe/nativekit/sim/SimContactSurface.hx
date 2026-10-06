package nativekit.sim;

import NativeKitSim;

/** Contact response for an environment shape. Lengths are seconds and friction is dimensionless. */
class SimContactSurface {
    public final friction:Array<Float>;
    public final frictionDimensions:Int;
    public final contactTimeConstant:Float;
    public final contactDampingRatio:Float;

    public function new(?friction:Array<Float>, frictionDimensions:Int = 0,
            contactTimeConstant:Float = 0.0, contactDampingRatio:Float = 0.0) {
        this.friction = friction == null ? [0.0, 0.0, 0.0] : friction.copy();
        this.frictionDimensions = frictionDimensions;
        this.contactTimeConstant = contactTimeConstant;
        this.contactDampingRatio = contactDampingRatio;
        var error = validate();
        if (error != null) throw error;
    }

    public function writeNative(target:nksim_surface):Void {
        target.set_struct_size(nksim_surface.size());
        for (axis in 0...3) target.set_friction(axis, friction[axis]);
        target.set_friction_dimensions(frictionDimensions);
        target.set_contact_time_constant(contactTimeConstant);
        target.set_contact_damping_ratio(contactDampingRatio);
        target.set_contact_filter(NativeKitSimConstants.NKSIM_CONTACT_LAYERS);
    }

    function validate():Null<String> {
        if (friction == null || friction.length != 3) return "contact friction needs three values";
        for (value in friction)
            if (!Math.isFinite(value) || value < 0.0) return "contact friction must be finite and non-negative";
        if (frictionDimensions != 0 && frictionDimensions != 1 && frictionDimensions != 3 &&
                frictionDimensions != 4 && frictionDimensions != 6)
            return "friction dimensions must be 0, 1, 3, 4 or 6";
        if (!Math.isFinite(contactTimeConstant) || contactTimeConstant < 0.0 ||
                !Math.isFinite(contactDampingRatio) || contactDampingRatio < 0.0)
            return "contact time constant and damping ratio must be finite and non-negative";
        return null;
    }
}
