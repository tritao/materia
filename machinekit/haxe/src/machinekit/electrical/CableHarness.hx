package machinekit.electrical;

import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;

/** Nominal cable layout geometry from cable_assembly.py's validated geometry plan.
 * Units mm. Housings are drawing-sized envelopes; contacts and hidden wire are omitted.
 * This is not a manufacturing model or a source of electrical/mass properties.
 */
class CableHarness {
    final plan:Dynamic;

    public function new(text:String) {
        plan = haxe.Json.parse(text);
        if (plan.schema_version != 1 || plan.units != "mm" || plan.manufacturing_qualified != false)
            throw "Expected reference cable geometry plan v1 in mm";
        var wires:Array<Dynamic> = plan.wires, ends:Array<Dynamic> = plan.ends;
        if (wires == null || wires.length == 0 || ends == null || ends.length == 0)
            throw "Cable plan needs wires and ends";
        var ids:Map<String,Bool> = [];
        for (wire in wires) {
            checkId(wire.id, ids);
            positive(wire.diameter_mm);
            if (wire.geometry_status != "nominal_route_envelope") throw "Unsupported wire geometry status";
            var points:Array<Dynamic> = wire.route;
            if (points == null || points.length < 2) throw "Wire route needs two points";
            var first = point(points[0]), last = point(points[points.length-1]);
            var direction = last.subtract(first), length = direction.length();
            positive(length);
            var previous = 0.0;
            for (i in 1...points.length) {
                var delta = point(points[i]).subtract(first), distance = delta.dot(direction)/length;
                if (delta.cross(direction).length() > 1e-6 || distance <= previous || distance > length+1e-6)
                    throw "Geometry v1 requires a straight ordered route";
                previous = distance;
            }
        }
        for (end in ends) {
            checkId(end.id, ids);
            var dimensions:Array<Dynamic> = end.dimensions_mm;
            if (dimensions == null || dimensions.length != 3) throw "Expected housing dimensions";
            for (dimension in dimensions) positive(dimension);
            if (end.geometry_status != "nominal_envelope") throw "Unsupported housing geometry status";
            point(end.origin);
            var exit = point(end.exit_direction), up = point(end.up);
            if (Math.abs(exit.length()-1)>1e-8 || Math.abs(up.length()-1)>1e-8 || Math.abs(exit.dot(up))>1e-8)
                throw "Connector frame must be orthonormal";
        }
    }

    static function checkId(id:Dynamic, ids:Map<String,Bool>):Void {
        if (!Std.isOfType(id,String) || id == "" || ids.exists(id)) throw "Missing or duplicate cable geometry ID";
        ids.set(id,true);
    }
    static function positive(value:Dynamic):Void {
        if ((!Std.isOfType(value,Float) && !Std.isOfType(value,Int)) || !Math.isFinite(value) || value<=0)
            throw "Expected finite positive cable dimension";
    }
    static function point(raw:Dynamic):Vector {
        var values:Array<Dynamic> = raw;
        if (values == null || values.length != 3) throw "Expected cable 3-vector";
        for (value in values)
            if ((!Std.isOfType(value,Float) && !Std.isOfType(value,Int)) || !Math.isFinite(value))
                throw "Expected finite cable coordinates";
        return new Vector(values[0],values[1],values[2]);
    }

    /** Independently owned compound with one solid per wire and connector envelope. */
    public function geometry():Part {
        var parts:Array<Part> = [];
        try {
            var wires:Array<Dynamic> = plan.wires, ends:Array<Dynamic> = plan.ends;
            for (wire in wires) {
                var route:Array<Dynamic> = wire.route;
                var start = point(route[0]), delta = point(route[route.length-1]).subtract(start);
                parts.push(Part.cylinderAlong(wire.diameter_mm/2,start,delta,delta.length()));
            }
            for (end in ends) {
                var dimensions:Array<Float> = end.dimensions_mm;
                var exit = point(end.exit_direction), up = point(end.up), right = exit.cross(up);
                // Local +Y is exit direction; housing extends behind the rear-face datum.
                var centre = point(end.origin).subtract(exit.scale(dimensions[1]/2));
                var blank = Part.box(dimensions[0],dimensions[1],dimensions[2],Center,Center,Center);
                try { parts.push(blank.placed(new Location(new Plane(centre,right,up)))); }
                catch (error:Dynamic) {blank.close();throw error;}
                blank.close();
            }
            var result = new Part(Shape.compound([for (part in parts) part.shape]));
            for (part in parts) part.close();
            return result;
        } catch (error:Dynamic) {for (part in parts) part.close();throw error;}
    }
}
