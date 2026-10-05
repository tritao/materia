package nativekit.sim;

import NativeKitSim;

/** A collision shape for session objects; actors currently use primitives. */
class SimShape {
    public final type:ShapeType;
    final a:Float;
    final b:Float;
    final c:Float;
    final vertices:Array<Float>;

    function new(type:ShapeType, a:Float, b:Float, c:Float, ?vertices:Array<Float>) {
        this.type = type;
        this.a = a;
        this.b = b;
        this.c = c;
        this.vertices = vertices == null ? [] : vertices.copy();
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

    /** Hull coordinates are XYZ triples in metres, in the object's local frame. */
    public static function convex(vertices:Array<Float>):SimShape {
        if (vertices == null || vertices.length < 12 || vertices.length > 192 || vertices.length % 3 != 0)
            throw "A session convex hull needs 4 to 64 vertices";
        for (value in vertices) if (!Math.isFinite(value)) throw "Convex hull coordinates must be finite";
        return new SimShape(ShapeType.Convex, 0, 0, 0, vertices);
    }

    public function convexVertices():Array<Float> return vertices.copy();

    /** Radius of the smallest sphere about the shape's origin that encloses it (infinite for a plane). */
    public function boundingRadius():Float
        return switch type {
            case Box: Math.sqrt(a * a + b * b + c * c);
            case Sphere: a;
            case Capsule: a + b * 0.5;
            case Cylinder: Math.sqrt(a * a + b * b * 0.25);
            case Convex:
                var radius = 0.0;
                for (i in 0...Std.int(vertices.length / 3)) {
                    var x = vertices[3 * i], y = vertices[3 * i + 1], z = vertices[3 * i + 2];
                    radius = Math.max(radius, Math.sqrt(x * x + y * y + z * z));
                }
                radius;
            default: Math.POSITIVE_INFINITY;
        };

    /** Half extents of a local box enclosing this finite primitive; null for an infinite plane. */
    public function boundingHalfExtents():Null<{x:Float, y:Float, z:Float}> return switch type {
        case Box: {x: a, y: b, z: c};
        case Sphere: {x: a, y: a, z: a};
        case Cylinder: {x: a, y: a, z: b * 0.5};
        case Capsule: {x: a, y: a, z: b * 0.5 + a};
        case Convex:
            var extents = [0.0, 0.0, 0.0];
            for (i in 0...vertices.length) extents[i % 3] = Math.max(extents[i % 3], Math.abs(vertices[i]));
            {x: extents[0], y: extents[1], z: extents[2]};
        case _: null;
    };

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
