package machinekit.tooling;

import cadkit.modeling.Curve;
import cadkit.modeling.Sketch;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Point2;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelPlan.PanelSegment;
import materia.tooling.panel.PanelPlan.PanelOperation;

/** Exact neutral-to-CadKit conversion shared by solid realization and drawings. */
class PanelCad {
    public static function segment(s:PanelSegment):Segment {
        var c=s.center;
        return c==null?Segment.line(new Point2(s.start.x,s.start.y),new Point2(s.end.x,s.end.y)):
            Segment.arc(new Point2(c.x,c.y),PanelContours.distance(c,s.start),Math.atan2(s.start.y-c.y,s.start.x-c.x),s.sweep);
    }
    public static function curves(op:PanelOperation):Array<Segment>
        return [for(s in PanelContours.operation(op))segment(s)];
    public static function prism(op:PanelOperation,z0:Float,z1:Float):Part {
        var edges:Array<Curve> = [];
        var wire:Null<Curve> = null,face:Null<Sketch> = null,body:Null<Part> = null;
        try {
            for(s in curves(op)) {
                var a=new Vector(s.start.x,s.start.y),b=new Vector(s.end.x,s.end.y),mid=s.point(0.5);
                edges.push(s.isLine()?Curve.line(a,b):Curve.arc(a,new Vector(mid.x,mid.y),b));
            }
            wire=Curve.wire(edges);face=Sketch.face(wire);body=face.extrude(z1-z0);
            var placed=body.translated(new Vector(0,0,z0));
            body.close();face.close();wire.close();for(e in edges)e.close();
            return placed;
        } catch(e:Dynamic) {
            if(body!=null)body.close();if(face!=null)face.close();if(wire!=null)wire.close();for(edge in edges)edge.close();throw e;
        }
    }
}
