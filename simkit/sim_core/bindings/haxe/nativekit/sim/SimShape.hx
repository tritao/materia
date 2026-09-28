package nativekit.sim;

import NativeKitSim;

/** A primitive collision shape for session objects and actor parts. */
class SimShape {
    public final type:ShapeType;
    final a:Float;
    final b:Float;
    final c:Float;

    function new(type:ShapeType, a:Float, b:Float, c:Float) {
        this.type = type;
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public static function box(halfX:Float, halfY:Float, halfZ:Float):SimShape
        return new SimShape(ShapeType.Box, halfX, halfY, halfZ);

    public static function sphere(radius:Float):SimShape
        return new SimShape(ShapeType.Sphere, radius, 0.0, 0.0);

    /** A solid cylinder along local +Z, of full height `height`. */
    public static function cylinder(radius:Float, height:Float):SimShape
        return new SimShape(ShapeType.Cylinder, radius, height, 0.0);

    /**
     * An infinite plane through the object's origin facing its local +Z, such
     * as a ground plane; only static objects may be planes.
     */
    public static function plane():SimShape
        return new SimShape(ShapeType.Plane, 0.0, 0.0, 1.0);

    /** A capsule along local +Z; length is between its hemisphere centres. */
    public static function capsule(radius:Float, length:Float):SimShape
        return new SimShape(ShapeType.Capsule, radius, length, 0.0);

    public function toNative():nksim_shape_desc {
        var value = new nksim_shape_desc();
        value.set_struct_size(nksim_shape_desc.size());
        value.set_type(type);
        value.set_parameters(0, a);
        value.set_parameters(1, b);
        value.set_parameters(2, c);
        return value;
    }
}
