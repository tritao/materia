package machinekit.motion;

import cadkit.modeling.*;
import machinekit.component.Solids;

/** 23HS22-2804S exterior reconstructed from the pinned 2018 supplier STEP.
 * See tools/suppliers/stepperonline-23hs22.json for provenance and measured datums.
 * Mechanical frame: face Z=0, housing toward -Z, shaft along +Z.
 * Omits lead exit/wires, casing screws and small chamfers; no guessed screw access bores.
 */
class StepperOnline23Geometry {
    public static inline final REFERENCE_SHA256:String="2683a79867bfd965b62b3fc73803ce65c430efbecf5678f793347f19f188f793";
    public static inline final PROFILE:String="stepperonline-23hs22-cad2018-v1";
    static function outline():Curve {
        var edges:Array<Curve> = [];
        function point(x:Float,y:Float,q:Int):Vector {
            for(i in 0...q){var previous=x;x=-y;y=previous;}
            return new Vector(x,y,0);
        }
        // Exact cylindrical side faces in the reference: outer R4 at (24.5,15.57),
        // concave R4 about the mounting-hole centre (23.57,23.57).
        for(q in 0...4) {
            function p(x:Float,y:Float):Vector return point(x,y,q);
            edges.push(Curve.line(p(28.5,0),p(28.5,15.57)));
            edges.push(Curve.arc(p(28.5,15.57),p(24.5+4/Math.sqrt(2),15.57+4/Math.sqrt(2)),p(24.5,19.57)));
            edges.push(Curve.line(p(24.5,19.57),p(23.57,19.57)));
            var diagonal=23.57-4/Math.sqrt(2);
            edges.push(Curve.arc(p(23.57,19.57),p(diagonal,diagonal),p(19.57,23.57)));
            edges.push(Curve.line(p(19.57,23.57),p(19.57,24.5)));
            edges.push(Curve.arc(p(19.57,24.5),p(15.57+4/Math.sqrt(2),24.5+4/Math.sqrt(2)),p(15.57,28.5)));
            edges.push(Curve.line(p(15.57,28.5),p(0,28.5)));
        }
        try{var wire=Curve.wire(edges);for(edge in edges)edge.close();return wire;}
        catch(error:Dynamic){for(edge in edges)edge.close();throw error;}
    }
    public static function build(length:Float):Part {
        if(!Math.isFinite(length) || length<16)throw "StepperOnline-profile motor must be at least 16 mm long";
        return Solids.building([],owned -> {
            var wire=outline(),section=Sketch.face(wire,[],Plane.XY());wire.close();
            var housing=section.extrude(-length);section.close();owned.push(housing);
            var rectangle=Sketch.rectangle(57,57),blank=rectangle.extrude(-5);rectangle.close();owned.push(blank);
            var flange=blank.fillet(blank.edges().parallel(Axis.Z()),4);owned.push(flange);
            var holes:Array<Part> = [];
            for(x in [-23.57,23.57])for(y in [-23.57,23.57]) {
                var hole=Part.cylinderSpan(2.55,-5.1,0.1,x,y);owned.push(hole);holes.push(hole);
            }
            flange=Solids.cut(flange,holes);owned.push(flange);
            var pilot=Part.cylinderSpan(19.05,0,1.6);owned.push(pilot);
            var shaft=Part.cylinderSpan(3.175,0,21);owned.push(shaft);
            var flat=Part.prism([new Vector(2.675,-4),new Vector(8,-4),new Vector(8,4),new Vector(2.675,4)],6,21.1);owned.push(flat);
            shaft=Solids.cut(shaft,[flat]);owned.push(shaft);
            var fused=Solids.union([housing,flange,pilot,shaft]);owned.push(fused);
            var refined=fused.refined();fused.close();return refined;
        });
    }
}
