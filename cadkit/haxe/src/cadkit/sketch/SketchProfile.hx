package cadkit.sketch;

import cadkit.modeling.Curve;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.sketch.ProfileError;

private class ProfileSegment {
	public final entity:SketchEntity;
	public final start:Array<Float>;
	public final end:Array<Float>;
	public function new(entity:SketchEntity,start:Array<Float>,end:Array<Float>){this.entity=entity;this.start=start;this.end=end;}
}

/** Validates solved boundaries and converts them through Cadkit's existing native constructors. */
class SketchProfile {
	public static function build(authored:ConstrainedSketch, solved:SolvedSketch):Sketch {
		var tolerance = Math.max(authored.settings.tolerance * 10, 1e-7);
		var segments:Array<ProfileSegment> = []; var circles:Array<SketchEntity> = [];
		for (entity in authored.entities()) {
			if (entity.construction) continue;
			if (entity.kind == "circle") circles.push(entity);
			else if (entity.kind == "line") segments.push(new ProfileSegment(entity, solved.point(entity.first), solved.point(entity.second)));
			else if (entity.kind == "arc") {
				var center=solved.point(entity.first), radius=solved.radius(entity.id);
				segments.push(new ProfileSegment(entity,
					[center[0]+radius*Math.cos(entity.startAngle),center[1]+radius*Math.sin(entity.startAngle)],
					[center[0]+radius*Math.cos(entity.endAngle),center[1]+radius*Math.sin(entity.endAngle)]));
			}
		}
		if (segments.length == 0 && circles.length == 0) throw new ProfileError("empty", [], "sketch has no profile geometry");
		var loops = connectedLoops(segments, tolerance);
		for (loop in loops) rejectSelfIntersection(loop, tolerance);

		var boundaries:Array<{curve:Curve, sample:Array<Float>, area:Float, ids:Array<String>}> = [];
		for (loop in loops) boundaries.push(makeLoop(loop, authored.plane, solved));
		for (circle in circles) {
			var center=solved.point(circle.first), localPlane=new Plane(authored.plane.toWorld(new Vector(center[0],center[1])),authored.plane.xDirection,authored.plane.normal);
			boundaries.push({curve:Curve.circle(solved.radius(circle.id),localPlane),sample:center,area:Math.PI*solved.radius(circle.id)*solved.radius(circle.id),ids:[circle.id]});
		}
		// Each boundary is assigned to the smallest larger boundary containing its sample.
		var parent:Array<Int> = []; for (_ in boundaries) parent.push(-1);
		for (i in 0...boundaries.length) {
			var best=-1; var bestArea=1e300;
			for (j in 0...boundaries.length) if(i!=j && boundaries[j].area>boundaries[i].area+tolerance && containsBoundary(boundaries[j],boundaries[i].sample,authored,solved))
				if(boundaries[j].area<bestArea){best=j;bestArea=boundaries[j].area;}
			parent[i]=best;
		}
		var result:Null<Sketch> = null;
		try {
			for(i in 0...boundaries.length) if(parent[i]<0) {
				var holes:Array<Curve> = []; for(j in 0...boundaries.length) if(parent[j]==i) holes.push(boundaries[j].curve);
				var face=Sketch.face(boundaries[i].curve,holes,authored.plane);
				if(result==null) result=face; else { var combined=result.combine(face); result.close(); face.close(); result=combined; }
			}
			if(result==null) throw new ProfileError("ambiguous",allIds(boundaries),"profile has no outer boundary");
			for(boundary in boundaries) boundary.curve.close();
			return result;
		} catch(error:Dynamic) {
			for(boundary in boundaries) boundary.curve.close();
			if(result!=null) result.close();
			if(Std.isOfType(error,ProfileError)) throw error;
			throw new ProfileError("invalid",allIds(boundaries),"native profile construction failed: "+Std.string(error));
		}
	}

	private static function connectedLoops(segments:Array<ProfileSegment>,t:Float):Array<Array<ProfileSegment>> {
		var loops:Array<Array<ProfileSegment>> = []; var used:Array<Bool> = []; for(_ in segments)used.push(false);
		for(i in 0...segments.length) if(!used[i]) {
			var loop:Array<ProfileSegment> = []; var current=i; var start=segments[i].start; var end=segments[i].end;
			while(true){used[current]=true;loop.push(segments[current]);if(near(end,start,t))break;var found=-1;var reversed=false;var count=0;
				for(j in 0...segments.length)if(!used[j]){if(near(segments[j].start,end,t)){found=j;reversed=false;count++;}else if(near(segments[j].end,end,t)){found=j;reversed=true;count++;}}
				if(count==0)throw new ProfileError("open",ids(loop),"open boundary near entity "+segments[current].entity.id);
				if(count>1)throw new ProfileError("branching",ids(loop),"branching boundary near entity "+segments[current].entity.id);
				current=found;if(reversed){var old=segments[current];segments[current]=new ProfileSegment(old.entity,old.end,old.start);}end=segments[current].end;
			}
			loops.push(loop);
		}
		return loops;
	}

	private static function makeLoop(loop:Array<ProfileSegment>,plane:Plane,solved:SolvedSketch):{curve:Curve,sample:Array<Float>,area:Float,ids:Array<String>} {
		var curves:Array<Curve> = []; var area=0.0;
		try { for(segment in loop){area+=segment.start[0]*segment.end[1]-segment.end[0]*segment.start[1];
			if(segment.entity.kind=="line")curves.push(Curve.line(world(plane,segment.start),world(plane,segment.end)));
			else {var e=segment.entity,c=solved.point(e.first),r=solved.radius(e.id);var delta=e.endAngle-e.startAngle;if(e.clockwise){while(delta>=0)delta-=2*Math.PI;}else while(delta<=0)delta+=2*Math.PI;var mid=e.startAngle+delta/2;curves.push(Curve.arc(world(plane,segment.start),world(plane,[c[0]+r*Math.cos(mid),c[1]+r*Math.sin(mid)]),world(plane,segment.end)));}}
			var wire=Curve.wire(curves);for(curve in curves)curve.close();return {curve:wire,sample:loop[0].start,area:Math.abs(area/2),ids:ids(loop)};
		}catch(error:Dynamic){for(curve in curves)curve.close();throw error;}
	}

	private static function rejectSelfIntersection(loop:Array<ProfileSegment>,t:Float):Void { for(i in 0...loop.length)for(j in i+1...loop.length){if(j==i+1||(i==0&&j==loop.length-1))continue;if(intersects(loop[i].start,loop[i].end,loop[j].start,loop[j].end,t))throw new ProfileError("self-intersecting",[loop[i].entity.id,loop[j].entity.id],"self-intersecting profile boundary");} }
	private static function intersects(a:Array<Float>,b:Array<Float>,c:Array<Float>,d:Array<Float>,t:Float):Bool { var ab=[b[0]-a[0],b[1]-a[1]],cd=[d[0]-c[0],d[1]-c[1]],den=cross(ab,cd);if(Math.abs(den)<t)return false;var ac=[c[0]-a[0],c[1]-a[1]],u=cross(ac,cd)/den,v=cross(ac,ab)/den;return u>t&&u<1-t&&v>t&&v<1-t; }
	private static function containsBoundary(boundary:{curve:Curve,sample:Array<Float>,area:Float,ids:Array<String>},p:Array<Float>,authored:ConstrainedSketch,solved:SolvedSketch):Bool {
		if(boundary.ids.length==1){var e:Null<SketchEntity> = null;for(candidate in authored.entities())if(candidate.id==boundary.ids[0])e=candidate;if(e!=null&&e.kind=="circle"){var c=solved.point(e.first),r=solved.radius(e.id);return (p[0]-c[0])*(p[0]-c[0])+(p[1]-c[1])*(p[1]-c[1])<r*r;}}
		var vertices:Array<Array<Float>> = [];for(id in boundary.ids)for(e in authored.entities())if(e.id==id&&e.kind=="line")vertices.push(solved.point(e.first));var inside=false;var j=vertices.length-1;for(i in 0...vertices.length){var a=vertices[i],b=vertices[j];if((a[1]>p[1])!=(b[1]>p[1])&&p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0])inside=!inside;j=i;}return inside;
	}
	private static function world(plane:Plane,p:Array<Float>):Vector return plane.toWorld(new Vector(p[0],p[1]));
	private static function near(a:Array<Float>,b:Array<Float>,t:Float):Bool return Math.abs(a[0]-b[0])<=t&&Math.abs(a[1]-b[1])<=t;
	private static function cross(a:Array<Float>,b:Array<Float>):Float return a[0]*b[1]-a[1]*b[0];
	private static function ids(loop:Array<ProfileSegment>):Array<String>{var out:Array<String> = [];for(s in loop)out.push(s.entity.id);return out;}
	private static function allIds(boundaries:Array<{curve:Curve,sample:Array<Float>,area:Float,ids:Array<String>}>):Array<String>{var out:Array<String> = [];for(b in boundaries)for(id in b.ids)out.push(id);return out;}
}
