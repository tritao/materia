package machinekit.structural;

import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Scope;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import materia.casting.CastingProfile.CastingSegment;
import materia.casting.CastingProfileGeometry;
import materia.casting.CastingProfiles;

/** Shared exact source/panel boundary. Drawings use these same line/arc records. */
class CastingProfileCad {
	public static function segment(record:CastingSegment):Segment {
		var center=record.center;
		return center==null
			? Segment.line(new Point2(record.start.x,record.start.y),new Point2(record.end.x,record.end.y))
			: Segment.arc(new Point2(center.x,center.y),CastingProfiles.distance(center,record.start),
				Math.atan2(record.start.y-center.y,record.start.x-center.x),record.sweep);
	}

	public static function wire(profile:Array<CastingSegment>):Curve {
		CastingProfileGeometry.validate(profile);
		return Scope.run(scope -> {
			var edges:Array<Curve> = [];
			for (record in profile) {
				var curve=segment(record),start=new Vector(curve.start.x,curve.start.y),end=new Vector(curve.end.x,curve.end.y);
				var midpoint=curve.point(0.5);
				edges.push(scope.own(curve.isLine() ? Curve.line(start,end) : Curve.arc(start,new Vector(midpoint.x,midpoint.y),end)));
			}
			return scope.own(Curve.wire(edges));
		});
	}

	public static function drafted(sections:materia.casting.CastingDraftSections.DraftedCastingSections):Part {
		return Scope.run(scope -> {
			var inner=scope.own(wire(sections.inner)),opening=scope.own(wire(sections.opening));
			var top=scope.own(opening.placed(cadkit.modeling.Location.translation(new Vector(0,0,sections.depth))));
			var body=scope.own(Part.loft([inner,top],true));
			return scope.own(machinekit.assembly.PosedParts.moved(body,sections.pose));
		});
	}

	public static function prism(profile:Array<CastingSegment>,z0:Float,z1:Float):Part {
		if (!Math.isFinite(z0+z1) || z1<=z0) throw "Invalid casting profile extrusion";
		return Scope.run(scope -> {
			var boundary=scope.own(wire(profile)),face=scope.own(Sketch.face(boundary));
			var body=scope.own(face.extrude(z1-z0));
			return scope.own(body.translated(new Vector(0,0,z0)));
		});
	}
}
