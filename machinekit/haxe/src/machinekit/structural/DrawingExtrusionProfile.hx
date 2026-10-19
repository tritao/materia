package machinekit.structural;

import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import machinekit.structural.FrameAssembly.StructuralProfile;

/** Extrudes an explicit drawing section at any length, retaining line, bulge-arc and circle geometry.
 * Input loops are canonical XY: outer counterclockwise, holes clockwise, no inferred slot dimensions.
 */
class DrawingExtrusionProfile implements StructuralProfile {
    final section:Dynamic;
    final configurationHash:String;
    public function new(section:Dynamic) {
        this.section=haxe.Json.parse(haxe.Json.stringify(section));
        // Geometry identity excludes supplier, price, download URL and other provenance fields.
        var geometryText=haxe.Json.stringify({loops:section.loops,circles:section.circles,
            dimensions:section.dimensions_mm,area:section.area_mm2});
        var digest=haxe.crypto.Sha256.make(haxe.io.Bytes.ofString(geometryText)),hex=new StringBuf();
        for(i in 0...digest.length) hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
        configurationHash=hex.toString();
    }
    public function profileDesignation():String return "DRAWING-" + configurationHash;
    public function profileDescription():String return "Drawing-derived extrusion; interfaces unqualified";
    public function sectionBounds():SectionBounds {
        var dimensions:Array<Float> = section.dimensions_mm;
        return {minX:-dimensions[0]/2,maxX:dimensions[0]/2,minY:-dimensions[1]/2,maxY:dimensions[1]/2};
    }

    static function loop(vertices:Array<Dynamic>):Curve {
        var edges:Array<Curve> = [];
        try {
            for (i in 0...vertices.length) {
                var a:Array<Float> = vertices[i], b:Array<Float> = vertices[(i+1)%vertices.length];
                var start=new Vector(a[0],a[1]),end=new Vector(b[0],b[1]);
                edges.push(Math.abs(a[2])<1e-12 ? Curve.line(start,end) : Curve.arc(start,
                    new Vector((a[0]+b[0])/2+(b[1]-a[1])*a[2]/2,(a[1]+b[1])/2-(b[0]-a[0])*a[2]/2),end));
            }
            var wire=Curve.wire(edges);
            for(edge in edges) edge.close();
            return wire;
        } catch(error:Dynamic) {for(edge in edges) edge.close();throw error;}
    }

    public function geometry(length:Float):Part {
        if (!Math.isFinite(length) || length<=0) throw "Drawing extrusion requires a positive length";
        var loops:Array<Dynamic> = section.loops, circles:Array<Dynamic> = section.circles;
        var wires:Array<Curve> = [];
        try {
            for(vertices in loops) wires.push(loop(vertices));
            for(circle in circles) wires.push(Curve.circle(circle.radius,
                new Plane(new Vector(circle.x,circle.y),Vector.X(),Vector.Z().scale(-1))));
            var face=Sketch.face(wires[0],wires.slice(1));
            var part:Part;
            try part=face.extrude(length) catch(error:Dynamic) {face.close();throw error;}
            face.close();
            for(wire in wires) wire.close();
            wires=[];
            if (!part.valid() || part.solidCount()!=1 ||
                Math.abs(part.volume()/length-section.area_mm2)>Math.max(0.001,section.area_mm2*0.00001)) {
                part.close();throw "Drawing extrusion does not reproduce section area";
            }
            return part;
        } catch(error:Dynamic) {for(wire in wires) wire.close();throw error;}
    }
}
