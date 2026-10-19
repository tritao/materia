package machinekit.motion;

import cadkit.modeling.*;
import machinekit.component.Solids;

/** Adapted from Gumyr's bd_warehouse, Apache-2.0.
 * See machinekit/third-party/bd_warehouse/NOTICE for pinned source and changes.
 */
class OpenBuildsNema23Geometry {
    static function outline(z:Float):Curve {
        var edges:Array<Curve> = [];
        function point(x:Float,y:Float,quarter:Int):Vector {
            for(i in 0...quarter) {var previous=x;x=-y;y=previous;}
            return new Vector(x,y,z);
        }
        // One quadrant, reflected about its diagonal in the upstream sketch.
        for(q in 0...4) {
            function p(x:Float,y:Float):Vector return point(x,y,q);
            edges.push(Curve.line(p(28.2,0),p(28.2,16.07)));
            edges.push(Curve.arc(p(28.2,16.07),p(25.7+2.5/Math.sqrt(2),16.07+2.5/Math.sqrt(2)),p(25.7,18.57)));
            edges.push(Curve.line(p(25.7,18.57),p(23.57,18.57)));
            var diagonal=23.57-5/Math.sqrt(2);
            edges.push(Curve.arc(p(23.57,18.57),p(diagonal,diagonal),p(18.57,23.57)));
            edges.push(Curve.line(p(18.57,23.57),p(18.57,25.7)));
            edges.push(Curve.arc(p(18.57,25.7),p(16.07+2.5/Math.sqrt(2),25.7+2.5/Math.sqrt(2)),p(16.07,28.2)));
            edges.push(Curve.line(p(16.07,28.2),p(0,28.2)));
        }
        try {var wire=Curve.wire(edges);for(edge in edges)edge.close();return wire;}
        catch(e:Dynamic){for(edge in edges)edge.close();throw e;}
    }
    static function housing(z:Float,depth:Float,inset:Float=0):Part {
        var wire=outline(z),boundary=inset==0 ? wire : wire.offset(-inset);
        try {
            var sketch=Sketch.face(boundary,[],Plane.XY().offset(z));
            var result=sketch.extrude(-depth);sketch.close();
            if(boundary!=wire)boundary.close();wire.close();return result;
        } catch(e:Dynamic){if(boundary!=wire)boundary.close();wire.close();throw e;}
    }
    public static function build(length:Float,refine:Bool=true):Part {
        if(!Math.isFinite(length) || length<=0)throw "Invalid OpenBuilds motor length";
        return Solids.building([],owned -> {
            var cap=Math.min(length/2,11),flangeDepth=Math.min(length,4.8);
            var pieces=[housing(0,cap),housing(-length+cap,cap)];
            for(p in pieces)owned.push(p);
            // Upstream's inset stack starts at the front plane and overlaps the caps.
            var middle=housing(0,length-cap,0.1);owned.push(middle);pieces.push(middle);
            var flangeSketch=Sketch.rectangle(56.4,56.4);
            var flangeRaw=flangeSketch.extrude(-flangeDepth);flangeSketch.close();
            var flange=flangeRaw.fillet(flangeRaw.edges().parallel(Axis.Z()),4.6);
            flangeRaw.close();owned.push(flange);
            var holes:Array<Part> = [];
            for(x in [-23.57,23.57])for(y in [-23.57,23.57]) {
                var hole=Part.cylinderSpan(2.55,-flangeDepth-0.1,0.1,x,y);owned.push(hole);holes.push(hole);
            }
            flange=Solids.cut(flange,holes);owned.push(flange);pieces.push(flange);
            var pilot=Part.cylinderSpan(19.05,0,1.6);owned.push(pilot);pieces.push(pilot);
            var shaft=Part.cylinderSpan(3.175,0,20.6);owned.push(shaft);
            var flat=Part.prism([new Vector(-4,2.675),new Vector(4,2.675),new Vector(4,8),new Vector(-4,8)],5.6,20.7);owned.push(flat);
            shaft=Solids.cut(shaft,[flat]);owned.push(shaft);pieces.push(shaft);
            var fused=Solids.union(pieces);owned.push(fused);
            if(!refine)return fused;
            var refined=fused.refined();fused.close();return refined;
        });
    }
}
