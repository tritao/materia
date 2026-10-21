package machinekit.tooling;

import cadkit.drawing.Drawing2D;
import cadkit.drawing.DrawingEntity;
import cadkit.drawing.DrawingLayer;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingBounds;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;

typedef InsertDrawingReview = {
	var drawing:Drawing2D;
	var validation:materia.tooling.ToolingValidation;
}

/** Nominal orthographic body views and source hole-entry projections, not CAM
 * contours. The hole schedule retains entry frames, depth, thread and route.
 * CadKit's existing exporters render the same Drawing2D in both formats.
 */
class InsertDrawing {
	public static function review(insert:CastingToolingInsert):InsertDrawingReview {
		var entities:Array<DrawingEntity> = [];
		var dimensions=switch insert.body {
			case Primitive(solid):switch solid {
				case Box(w,d,h):{width:w,depth:d,height:h,round:false};
				case Cylinder(d,h):{width:d,depth:d,height:h,round:true};
			};
			case CrimpedSocket(_,_):throw "Catalogue retention-body drawings use the supplier document: "+insert.id;
		};
		var views=[{axis:2,x:0.0,y:0.0,name:"XY entry projection"},
			{axis:1,x:dimensions.width+20,y:0.0,name:"XZ entry projection"},
			{axis:0,x:2*dimensions.width+dimensions.depth+40,y:0.0,name:"YZ entry projection"}];
		function line(layer:DrawingLayer,x1:Float,y1:Float,x2:Float,y2:Float):Void
			entities.push(Curve(layer,Segment.line(new Point2(x1,y1),new Point2(x2,y2))));
		function circle(layer:DrawingLayer,x:Float,y:Float,radius:Float):Void {
			entities.push(Curve(layer,Segment.arc(new Point2(x,y),radius,0,Math.PI)));
			entities.push(Curve(layer,Segment.arc(new Point2(x,y),radius,Math.PI,Math.PI)));
		}
		for (view in views) {
			var width=view.axis==0 ? dimensions.depth : dimensions.width;
			var height=view.axis==2 ? dimensions.depth : dimensions.height;
			var x=view.x-width/2,y=view.axis==2 ? -height/2 : 0.0;
			if (dimensions.round && view.axis==2) circle(Outline,view.x,view.y,width/2);
			else {
				line(Outline,x,y,x+width,y);line(Outline,x+width,y,x+width,y+height);
				line(Outline,x+width,y+height,x,y+height);line(Outline,x,y+height,x,y);
			}
			entities.push(Label(Text,new Point2(x,y-6),2.5,view.name+" / "+width+" x "+height+" mm"));
		}
		var row=0,ymax=Math.max(dimensions.depth/2,dimensions.height)+10;
		function note(text:String):Void
			entities.push(Label(Text,new Point2(-dimensions.width/2,ymax+5*row++),2.5,text));
		note(insert.id+" / "+insert.material.grade+" / nominal raw insert / mm / review drawing");
		note(dimensions.round ? "Datums: cylinder axis at XY=0; back face Z=0."
			: "Datums: left edge X=-W/2, end Y=-D/2, back Z=0. Entry offsets below are from these datums.");
		note("Raw views exclude post-cast or undecided holes; the schedule includes the complete service hole pattern.");
		var precastHoles=materia.casting.InsertStages.holes(
			{body:insert.body,holes:insert.holes,origin:insert.origin,anchorage:insert.anchorage,interfaces:insert.interfaces},
			materia.casting.InsertStages.InsertStage.BeforeCasting);
		var unsupported:Array<String> = [];
		var manufacturing=switch insert.origin {
			case Fabricated(plan):plan;
			case Catalogue(_):throw "Catalogue drawing is not a made-insert drawing";
		};
		for (hole in insert.holes) {
			var physical=precastHoles.filter(candidate -> candidate.id==hole.id);
			if (physical.length==1) {
				var cut=physical[0];
				note(cut.id+" PRE-CAST machining entry XYZ="+cut.frame.x+","+cut.frame.y+","+cut.frame.z
					+" / drill depth="+cut.depth+" / thread endpoint="+cut.threadDepth);
				if (cut.counterbore!=null) note(cut.id+" PRE-CAST counterbore D"+cut.counterbore.diameter+" x "+cut.counterbore.depth);
				if (cut.relief!=null) note(cut.id+" PRE-CAST relief D"+cut.relief.diameter+" from depth="+cut.relief.start);
				var normal=F.transformVector(cut.frame,0,0,1),axis=CastingBounds.axis(normal);
				var diameter=cut.thread==null ? cut.diameter : cut.thread.majorDiameter;
				if (axis==null) unsupported.push(cut.id);
				else for (view in views) if (view.axis==axis) {
					var x=view.x+(axis==0 ? cut.frame.y : cut.frame.x),y=axis==2 ? cut.frame.y : cut.frame.z;
					circle(Process("HOLE_ENTRY"),x,y,diameter/2);
					if (cut.thread!=null) circle(Process("TAP_DRILL"),x,y,cut.diameter/2);
					if (cut.counterbore!=null) circle(Process("COUNTERBORE"),x,y,cut.counterbore.diameter/2);
				}
			}
			var normal=F.transformVector(hole.frame,0,0,1);
			var routes=manufacturing.holeRoutes.filter(choice -> choice.holeId==hole.id);
			var datumX=dimensions.round ? hole.frame.x : hole.frame.x+dimensions.width/2;
			var datumY=dimensions.round ? hole.frame.y : hole.frame.y+dimensions.depth/2;
			note(hole.id+" datum offsets XYZ="+datumX+","+datumY+","+hole.frame.z
				+" axis="+normal.x+","+normal.y+","+normal.z+" / "+(hole.thread==null ? "D"+hole.diameter : hole.thread.designation)
				+" / depth="+hole.depth+" / "+hole.extent+" / route="+(routes.length==0 ? "undecided" : (routes[0].route:String)));
			for (requirement in manufacturing.mountingRequirements) if (requirement.holeId==hole.id)
				note(hole.id+" derived from "+requirement.componentPath+"/"+requirement.connectorId);
			if (hole.thread!=null) note(hole.id+" tap drill="+hole.tapDrill+" / full-thread endpoint="+hole.threadDepth+" / policy="+hole.limits.policyId);
			if (hole.counterbore!=null) note(hole.id+" counterbore D"+hole.counterbore.diameter+" x "+hole.counterbore.depth);
			if (hole.relief!=null) note(hole.id+" bore relief D"+hole.relief.diameter+" from depth="+hole.relief.start);
		}
		for (requirement in insert.interfaces) {
			var frame=materia.casting.InterfaceRequirement.InterfaceRequirements.geometryFrame(requirement.geometry);
			note(requirement.id+" finished reference XYZ="+frame.x+","+frame.y+","+frame.z+" / "+requirement.route
				+" / stock or layer="+requirement.allowanceMm+" / finishing error allocation="
				+(requirement.finishingProcessErrorMm==null ? "pending" : Std.string(requirement.finishingProcessErrorMm))
				+" / foundation datum="+requirement.datumReference);
		}
		if (unsupported.length>0) note("Oblique entry projections not rendered: "+unsupported.join(", "));
		return {drawing:new Drawing2D(entities),validation:{rule:"fabrication.insert.drawing."+insert.id,affectedIds:[insert.id],
			status:unsupported.length>0 ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported : materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending,
			severity:materia.tooling.ToolingDiagnostic.ToolingSeverity.Warning,
			message:unsupported.length>0 ? "Oblique hole entry views require exact ellipse projection: "+unsupported.join(", ")
				: "Review and release the nominal insert drawing, datum dimensions and inspection limits before making the insert.",evidence:[]}};
	}
}
