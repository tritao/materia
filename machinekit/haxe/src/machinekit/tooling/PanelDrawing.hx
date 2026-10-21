package machinekit.tooling;

import cadkit.drawing.Drawing2D;
import cadkit.drawing.DrawingEntity;
import cadkit.drawing.DrawingLayer;
import cadkit.sheetmetal.Point2;
import materia.tooling.panel.PanelPlan;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.casting.CastingProfileGeometry;

/** Rendering stays in MachineKit; existing CadKit SVG/DXF writers consume Drawing2D. */
class PanelDrawing {
    public static function drawing(source:PanelPlan):Drawing2D {
        var p=PanelCodec.snapshot(source),entities:Array<DrawingEntity> = [];
        var bounds=CastingProfileGeometry.bounds(PanelContours.outside(p)),row=0;
        entities.push(Label(Text,new Point2(0,bounds.height+5),2.5,
            p.id+" thickness="+p.stock.measuredThickness+" mm label-side="+p.labelSide));
        for(op in p.operations) {
            var layer=DrawingLayer.Process((op.kind:String).toUpperCase().split("-").join("_"));
            for(segment in PanelCad.curves(op))entities.push(Curve(layer,segment));
            entities.push(Label(Text,new Point2(0,bounds.height+10+row++*5),2.5,
                op.id+" "+op.kind+" "+op.side+" depth="+op.depth+" tool="+op.toolDiameter+" setup="+op.setupId));
        }
        entities.push(Label(Text,new Point2(p.labelX-15,p.labelY-1),Math.min(2.5,30/(0.6*p.id.length)),p.id));
        entities.push(Label(Text,new Point2(0,-8),2.5,"Finished geometry / mm / no cutter compensation / review drawing"));
        return new Drawing2D(entities);
    }
}
