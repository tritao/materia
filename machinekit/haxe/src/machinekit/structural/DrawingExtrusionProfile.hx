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
        this.section=validatedSection(section);
        // Geometry identity excludes supplier, price, download URL and other provenance fields.
        var geometryText=haxe.Json.stringify({loops:this.section.loops,circles:this.section.circles,
            dimensions:this.section.dimensions_mm,area:this.section.area_mm2});
        var digest=haxe.crypto.Sha256.make(haxe.io.Bytes.ofString(geometryText)),hex=new StringBuf();
        for(i in 0...digest.length) hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
        configurationHash=hex.toString();
    }
    /** Geometry only, embedded in recipe assemblies; no download paths or supplier metadata. */
    public function sectionText():String {
        var text=haxe.Json.stringify(section);
        if(text.length>4000000) throw "Drawing section payload exceeds portable recipe limit";
        return text;
    }
    public static function fromText(text:String):DrawingExtrusionProfile {
        if(text==null || text.length>4000000) throw "Invalid drawing section payload size";
        return new DrawingExtrusionProfile(haxe.Json.parse(text));
    }
    public static function defaultSectionText():String
        return '{"dimensions_mm":[20,20],"loops":[[[-10,-10,0],[10,-10,0],[10,10,0],[-10,10,0]]],"circles":[],"area_mm2":400}';

    static function number(raw:Dynamic):Float {
        if((!Std.isOfType(raw,Float) && !Std.isOfType(raw,Int)) || !Math.isFinite(raw))
            throw "Drawing section requires finite numeric coordinates";
        return raw;
    }
    static function array(raw:Dynamic):Array<Dynamic> {
        if(!Std.isOfType(raw,Array)) throw "Drawing section requires arrays";
        return cast raw;
    }
    static function validatedSection(raw:Dynamic):Dynamic {
        if(raw==null) throw "Missing drawing section";
        var dims=array(raw.dimensions_mm);
        if(dims.length!=2 || number(dims[0])<=0 || number(dims[1])<=0) throw "Invalid drawing section dimensions";
        var dimensions:Array<Float> = [number(dims[0]),number(dims[1])];
        var rawLoops=array(raw.loops),rawCircles=array(raw.circles);
        if(rawLoops.length==0 || rawLoops.length>1024 || rawCircles.length>1024) throw "Invalid drawing section contour count";
        var loops:Array<Array<Array<Float>>> = [],circles:Array<{x:Float,y:Float,radius:Float}> = [];
        var count=0;
        for(rawLoop in rawLoops) {
            var vertices=array(rawLoop),loop:Array<Array<Float>> = [];
            count+=vertices.length;
            if(vertices.length<3 || count>100000) throw "Invalid drawing section vertex count";
            for(rawVertex in vertices) {
                var v=array(rawVertex);
                if(v.length!=3) throw "Drawing vertices require x, y and bulge";
                var x=number(v[0]),y=number(v[1]),bulge=number(v[2]);
                if(Math.abs(x)>dimensions[0]/2+0.02 || Math.abs(y)>dimensions[1]/2+0.02)
                    throw "Drawing vertex exceeds section bounds";
                loop.push([x,y,bulge]);
            }
            for(i in 0...loop.length) {
                var a=loop[i],b=loop[(i+1)%loop.length];
                if(Math.abs(a[0]-b[0])+Math.abs(a[1]-b[1])<1e-10) throw "Drawing section has a degenerate edge";
            }
            loops.push(loop);
        }
        for(c in rawCircles) {
            var x=number(c.x),y=number(c.y),radius=number(c.radius);
            if(radius<=0 || Math.abs(x)+radius>dimensions[0]/2+0.02 || Math.abs(y)+radius>dimensions[1]/2+0.02)
                throw "Invalid drawing section circle";
            circles.push({x:x,y:y,radius:radius});
        }
        var area=number(raw.area_mm2);
        if(area<=0 || area>dimensions[0]*dimensions[1]+0.01) throw "Invalid drawing section area";
        return {dimensions_mm:dimensions,loops:loops,circles:circles,area_mm2:area};
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
            var bounds=part.shape.bounds(),dimensions:Array<Float> = section.dimensions_mm;
            if (!part.valid() || part.solidCount()!=1 ||
                Math.abs(bounds.get_min().get_x()+dimensions[0]/2)>0.02 || Math.abs(bounds.get_max().get_x()-dimensions[0]/2)>0.02 ||
                Math.abs(bounds.get_min().get_y()+dimensions[1]/2)>0.02 || Math.abs(bounds.get_max().get_y()-dimensions[1]/2)>0.02 ||
                Math.abs(part.volume()/length-section.area_mm2)>Math.max(0.001,section.area_mm2*0.00001)) {
                part.close();throw "Drawing extrusion does not reproduce section area";
            }
            return part;
        } catch(error:Dynamic) {for(wire in wires) wire.close();throw error;}
    }
}
