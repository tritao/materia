package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingRegion;
import materia.casting.CastingProfile.CastingSegment;

typedef CoreDraftSection = {
	var region:CastingRegion;
	var featureId:String;
	var opening:Array<CastingSegment>;
	var pose:AssemblyFrame;
	var depth:Float;
	var degrees:Float;
}

/** Supported ruled draft intent, proven against the whole finished cavity. */
class MoldCoreDrafts {
	static inline var EPS=1e-7;

	public static function hasDraft(region:CastingRegion,id:String):Bool {
		var found:Array<CoreDraftSection> =[];
		collect(region,id,found);
		return found.filter(candidate -> candidate.degrees>0).length>0;
	}

	public static function section(region:CastingRegion,id:String,toleranceMm3:Float):Null<CoreDraftSection> {
		var found:Array<CoreDraftSection> =[];
		collect(region,id,found);
		if (found.length!=1) return null;
		var candidate=found[0],normal=F.transformVector(candidate.pose,0,0,1);
		if (Math.abs(normal.x)>EPS || Math.abs(normal.y)>EPS || normal.z<1-EPS
			|| candidate.degrees<=0 || !tangentArcs(candidate.opening)) return null;
		var missing=MoldRegions.volume(CastingRegion.Difference(region,[candidate.region]),id);
		var excess=MoldRegions.volume(CastingRegion.Difference(candidate.region,[region]),id);
		return missing+excess<=toleranceMm3 ? candidate : null;
	}

	static function collect(region:CastingRegion,id:String,found:Array<CoreDraftSection>):Void {
		switch region {
			case Drafted(featureId,opening,pose,depth,degrees):
				if (featureId==id) found.push({region:region,featureId:featureId,opening:opening,pose:pose,depth:depth,degrees:degrees});
			case Union(children),Intersection(children):
				for (child in children) collect(child,id,found);
			case Difference(body,cuts):
				collect(body,id,found);
				for (cut in cuts) collect(cut,id,found);
			case Placed(_):
		}
	}

	/** Offset tangent arcs retain their angular limits, so their ruled walls
	 * remain exact cones. Non-tangent arc trims require a different construction.
	 * Sharp line/line corners still have exact planar draft faces.
	 */
	static function tangentArcs(profile:Array<CastingSegment>):Bool {
		for (index in 0...profile.length) {
			var a=profile[index],b=profile[(index+1)%profile.length];
			if (a.center==null && b.center==null) continue;
			var u=tangent(a,true),v=tangent(b,false);
			if (Math.abs(u.x*v.y-u.y*v.x)>EPS || u.x*v.x+u.y*v.y<1-EPS) return false;
		}
		return true;
	}

	static function tangent(segment:CastingSegment,end:Bool):{x:Float,y:Float} {
		var center=segment.center,dx:Float,dy:Float;
		if (center==null) {dx=segment.end.x-segment.start.x;dy=segment.end.y-segment.start.y;}
		else {
			var point=end ? segment.end : segment.start,sign=segment.sweep>0 ? 1.0 : -1.0;
			dx=-(point.y-center.y)*sign;dy=(point.x-center.x)*sign;
		}
		var length=Math.sqrt(dx*dx+dy*dy);
		return {x:dx/length,y:dy/length};
	}

	public static function foam(section:CoreDraftSection,cover:Float,glue:Float):CastingRegion {
		var depth=section.depth-cover-glue;
		if (depth<=0) throw "Covering and adhesive consume drafted foam height";
		var radians=section.degrees*Math.PI/180;
		var inset=cover/Math.cos(radians)+glue*Math.tan(radians);
		var opening=materia.casting.CastingProfileOffsets.inward(section.opening,inset);
		return CastingRegion.Drafted(section.featureId,opening,F.compose(section.pose,F.translation(0,0,cover)),depth,section.degrees);
	}
}
